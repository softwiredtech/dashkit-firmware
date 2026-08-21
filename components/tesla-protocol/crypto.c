#include "crypto.h"

#include <string.h>

#include "mbedtls/constant_time.h"
#include "mbedtls/bignum.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"

int tesla_sha1(const uint8_t *data, size_t len, uint8_t out[TESLA_SHA1_LEN])
{
    return mbedtls_sha1(data, len, out);
}

int tesla_sha256(const uint8_t *data, size_t len, uint8_t out[TESLA_SHA256_LEN])
{
    // is224 = 0: SHA-256, not SHA-224.
    return mbedtls_sha256(data, len, out, 0);
}

int tesla_hmac_sha256(const uint8_t *key, size_t key_len,
                      const uint8_t *msg, size_t msg_len,
                      uint8_t out[TESLA_HMAC_LEN])
{
    return mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                           key, key_len, msg, msg_len, out);
}

// HMAC over two non-contiguous parts (the protocol's "M || payload" shape).
int tesla_hmac_sha256_2(const uint8_t *key, size_t key_len,
                        const uint8_t *a, size_t a_len,
                        const uint8_t *b, size_t b_len,
                        uint8_t out[TESLA_HMAC_LEN])
{
    int rc;
    mbedtls_md_context_t md;

    mbedtls_md_init(&md);
    rc = mbedtls_md_setup(&md, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    if (rc == 0) {
        rc = mbedtls_md_hmac_starts(&md, key, key_len);
    }
    if (rc == 0) {
        rc = mbedtls_md_hmac_update(&md, a, a_len);
    }
    if (rc == 0) {
        rc = mbedtls_md_hmac_update(&md, b, b_len);
    }
    if (rc == 0) {
        rc = mbedtls_md_hmac_finish(&md, out);
    }
    mbedtls_md_free(&md);
    return rc;
}

bool tesla_ct_equal(const uint8_t *a, const uint8_t *b, size_t len)
{
    return mbedtls_ct_memcmp(a, b, len) == 0;
}

int tesla_derive_shared_key(const uint8_t priv[TESLA_PRIVKEY_LEN],
                            const uint8_t peer_pub[TESLA_PUBKEY_LEN],
                            tesla_rng_fn f_rng, void *p_rng,
                            uint8_t out[TESLA_SHARED_KEY_LEN])
{
    int rc;
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_mpi z;
    mbedtls_ecp_point peer;
    uint8_t x_buf[TESLA_PRIVKEY_LEN];
    uint8_t digest[TESLA_SHA1_LEN];

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);
    mbedtls_ecp_point_init(&peer);

    // mbedTLS 3.x requires an RNG for ECDH blinding (no internal RNG in the
    // ESP-IDF configuration).
    if (f_rng == NULL) {
        rc = -1;
        goto out;
    }

    rc = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (rc != 0) {
        goto out;
    }

    rc = mbedtls_mpi_read_binary(&d, priv, TESLA_PRIVKEY_LEN);
    if (rc != 0) {
        goto out;
    }

    rc = mbedtls_ecp_point_read_binary(&grp, &peer, peer_pub, TESLA_PUBKEY_LEN);
    if (rc != 0) {
        goto out;
    }

    // read_binary only decodes the point; reject points that do not satisfy
    // the curve equation (invalid-curve hardening, mirroring the reference
    // implementation which rejects off-curve peers).
    rc = mbedtls_ecp_check_pubkey(&grp, &peer);
    if (rc != 0) {
        goto out;
    }

    // z is the X-coordinate of the shared point (ECDH output).
    rc = mbedtls_ecdh_compute_shared(&grp, &z, &peer, &d, f_rng, p_rng);
    if (rc != 0) {
        goto out;
    }

    rc = mbedtls_mpi_write_binary(&z, x_buf, sizeof(x_buf));
    if (rc != 0) {
        goto out;
    }

    rc = mbedtls_sha1(x_buf, sizeof(x_buf), digest);
    if (rc != 0) {
        goto out;
    }

    // K = SHA1(X)[:16]
    memcpy(out, digest, TESLA_SHARED_KEY_LEN);
    rc = 0;

out:
    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);
    mbedtls_mpi_free(&z);
    mbedtls_ecp_point_free(&peer);
    return rc;
}

int tesla_session_info_key(const uint8_t k[TESLA_SHARED_KEY_LEN],
                           uint8_t out[TESLA_HMAC_LEN])
{
    static const char label[] = "session info";
    return tesla_hmac_sha256(k, TESLA_SHARED_KEY_LEN,
                             (const uint8_t *)label, sizeof(label) - 1, out);
}

int tesla_authenticated_command_key(const uint8_t k[TESLA_SHARED_KEY_LEN],
                                    uint8_t out[TESLA_HMAC_LEN])
{
    static const char label[] = "authenticated command";
    return tesla_hmac_sha256(k, TESLA_SHARED_KEY_LEN,
                             (const uint8_t *)label, sizeof(label) - 1, out);
}

int tesla_gcm_encrypt(const uint8_t k[TESLA_SHARED_KEY_LEN],
                      const uint8_t *plaintext, size_t plaintext_len,
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t nonce[TESLA_NONCE_LEN],
                      uint8_t *ciphertext,
                      uint8_t tag[TESLA_GCM_TAG_LEN])
{
    mbedtls_gcm_context gcm;
    int rc;

    mbedtls_gcm_init(&gcm);
    rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, k, 128);
    if (rc == 0) {
        rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, plaintext_len,
                                       nonce, TESLA_NONCE_LEN,
                                       aad, aad_len,
                                       plaintext, ciphertext,
                                       TESLA_GCM_TAG_LEN, tag);
    }
    mbedtls_gcm_free(&gcm);
    return rc;
}

int tesla_gcm_decrypt(const uint8_t k[TESLA_SHARED_KEY_LEN],
                      const uint8_t *ciphertext, size_t ciphertext_len,
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t nonce[TESLA_NONCE_LEN],
                      const uint8_t tag[TESLA_GCM_TAG_LEN],
                      uint8_t *plaintext)
{
    mbedtls_gcm_context gcm;
    int rc;

    mbedtls_gcm_init(&gcm);
    rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, k, 128);
    if (rc == 0) {
        rc = mbedtls_gcm_auth_decrypt(&gcm, ciphertext_len,
                                      nonce, TESLA_NONCE_LEN,
                                      aad, aad_len,
                                      tag, TESLA_GCM_TAG_LEN,
                                      ciphertext, plaintext);
    }
    mbedtls_gcm_free(&gcm);
    return rc;
}
