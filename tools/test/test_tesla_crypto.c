// Host round-trip unit test for the Tesla crypto/session layers
// (components/tesla-protocol/crypto.c and session.c).
//
// Compiled with plain gcc against a host mbedTLS 3.6.2 build; the mbedTLS
// 3.x API is the very thing Phase 0 de-risks, so the test runs against the
// same major.minor as the ESP-IDF 5.4.1 vendored copy. See
// run_tesla_crypto_test.sh for the one-shot build.
//
//   gcc -std=c99 -Wall -Wextra -I components/tesla-protocol
//       -I <mbedtls>/include
//       tools/test/test_tesla_crypto.c
//       components/tesla-protocol/crypto.c
//       components/tesla-protocol/session.c
//       -L <mbedtls>/lib -lmbedcrypto -lmbedtls -lmbedx509
//       -o /tmp/test_tesla_crypto && /tmp/test_tesla_crypto
//
// Known-answer vectors come from Tesla's Apache-2.0 protocol.md (test keys,
// K, SESSION_INFO_KEY, session-info HMAC, AES-GCM command example), the
// vehicle-command metadata_test.go checksum vector, and standard digest
// vectors. The "authenticated command" subkey vector was computed with
// openssl against the published K.

#include "crypto.h"
#include "session.h"

#include <stdio.h>
#include <string.h>

// Required to name the auth-failure error code.
#include "mbedtls/gcm.h"

// ---- tiny test harness (mirrors test_dbc.c) ----
static int g_fail;
#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_fail++; } \
    else { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Decodes hex into out, returns byte length or -1.
static int unhex(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t n = strlen(hex);
    if (n % 2 != 0 || n / 2 > out_cap) return -1;
    for (size_t i = 0; i < n / 2; i++) {
        int hi = hexval(hex[2 * i]);
        int lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return (int)(n / 2);
}

static bool bytes_eq(const uint8_t *a, const uint8_t *b, size_t n)
{
    return memcmp(a, b, n) == 0;
}

// Decodes `want` (hex) and checks it equals buf[0..n). Returns true on match.
static bool check_hex(const uint8_t *buf, size_t n, const char *want)
{
    uint8_t exp[128];
    int len = unhex(want, exp, sizeof(exp));
    return len == (int)n && bytes_eq(buf, exp, n);
}

// Deterministic RNG for ECDH blinding: correctness is unaffected.
static uint32_t g_rng_state = 0x12345678u;
static int dummy_rng(void *ctx, uint8_t *buf, size_t len)
{
    (void)ctx;
    for (size_t i = 0; i < len; i++) {
        g_rng_state = g_rng_state * 1664525u + 1013904223u;
        buf[i] = (uint8_t)(g_rng_state >> 24);
    }
    return 0;
}

// ---- vectors from Tesla's protocol.md ----
static const char VIN[] = "5YJ30123456789ABC";

// c = client private scalar; C = client public; v = vehicle private; V = vehicle public.
static const char CLIENT_PRIV_HEX[] =
    "2538cdc29a97c19c1e99a637d6cf4f8c970c118b56ede1e6323e6d162c4b30db";
static const char CLIENT_PUB_HEX[] =
    "04b2b6bc68c2da0665ce656815594996c62394edd8bea905fe781a754fe6a845a7"
    "14330902f225e9269d466e05b349981fda9d85cc23c6fb444aa73b629105dc6e";
static const char VEHICLE_PRIV_HEX[] =
    "344ee5b466a7cf1eeb12b6f50331db2e5ec5834ef5f4befcfd8cbe55c2528d70";
static const char VEHICLE_PUB_HEX[] =
    "04c7a1f47138486aa4729971494878d33b1a24e39571f748a6e16c5955b3d877d3"
    "a6aaa0e955166474af5d32c410f439a2234137ad1bb085fd4e8813c958f11d97";

// K = SHA1(ECDH X-coord)[:16]; SESSION_INFO_KEY; AUTH_CMD_KEY (openssl).
static const char K_HEX[]          = "1b2fce19967b79db696f909cff89ea9a";
static const char SESSION_KEY_HEX[] =
    "fceb679ee7bca756fcd441bf238bf2f338629b41d9eb9c67be1b32c9672ce300";
static const char AUTH_CMD_KEY_HEX[] =
    "6d3a14d0b6d762e4f076739e2cf6edd291d2e56e3f25bc6a2af5cb26dc753b14";

// Session-info handshake example.
static const char CHALLENGE_HEX[]   = "1588d5a30eabc6f8fc9a951b11f6fd11";
static const char SESSION_META_HEX[] =
    "000106021135594a333031323334353637383941424306101588d5a30eabc6f8fc9a951b11f6fd11ff";
static const char SESSION_INFO_HEX[] =
    "0806124104c7a1f47138486aa4729971494878d33b1a24e39571f748a6e16c5955b3d877d3"
    "a6aaa0e955166474af5d32c410f439a2234137ad1bb085fd4e8813c958f11d97"
    "1a104c463f9cc0d3d26906e982ed224adde6255a0a0000";
static const char SESSION_TAG_HEX[] =
    "996c1fe38331be138f8039c194b14db2198846ed7d8251e6749284d7b32ea002";

// AES-GCM command example ("Turn HVAC on", CarServer.Action 120452020801).
static const char EPOCH_HEX[]      = "4c463f9cc0d3d26906e982ed224adde6";
static const char CMD_META_HEX[] =
    "000105010103021135594a333031323334353637383941424303104c463f9cc0d3d26906e982ed224adde6"
    "040400000a5f050400000007070400000002ff";
static const char NONCE_HEX[]      = "dbf79447fa156674dae1caed";
static const char PLAINTEXT_HEX[]  = "120452020801";
static const char CIPHERTEXT_HEX[] = "38038e8c0f2e";
static const char GCM_TAG_HEX[]    = "c228e0ff64991481db3a7bbc133696c5";

// Metadata SHA-256 checksum vector from vehicle-command metadata_test.go.
// Item set: sig_type=0x05, domain=0x02, personalization="testVIN",
// epoch, expires_at=3700, counter=1338.
static const char META2_EPOCH_HEX[] = "aada928a4f215f55f9e6e45e66b6521e";
static const char META2_SHA256_HEX[] =
    "abab04d804499813382efd74a06791ce2de777439603246dfbaa8392ca05868e";

// Standard digest vectors.
static const char SHA1_ABC_HEX[]   = "a9993e364706816aba3e25717850c26c9cd0d89d";
static const char SHA256_ABC_HEX[] =
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
static const char HMAC_FOX_HEX[] =
    "f7bc83f430538424b13298e6aa6fb143ef4d59a14946175997479dbc2d1a3cd8";

static void test_hashes_and_hmac(void)
{
    uint8_t d[TESLA_SHA256_LEN];

    tesla_sha1((const uint8_t *)"abc", 3, d);
    CHECK(check_hex(d, TESLA_SHA1_LEN, SHA1_ABC_HEX),
          "sha1(\"abc\") = %s", SHA1_ABC_HEX);

    tesla_sha256((const uint8_t *)"abc", 3, d);
    CHECK(check_hex(d, TESLA_SHA256_LEN, SHA256_ABC_HEX),
          "sha256(\"abc\") = %s", SHA256_ABC_HEX);

    static const uint8_t key[] = "key";
    static const char msg[] = "The quick brown fox jumps over the lazy dog";
    tesla_hmac_sha256(key, sizeof(key) - 1, (const uint8_t *)msg, sizeof(msg) - 1, d);
    CHECK(check_hex(d, TESLA_HMAC_LEN, HMAC_FOX_HEX),
          "hmac_sha256(key=\"key\", fox) matches standard vector");

    // Two-part HMAC must equal the one-shot HMAC over the concatenation.
    {
        uint8_t key2[16];
        uint8_t d1[TESLA_HMAC_LEN], d2[TESLA_HMAC_LEN];
        static const uint8_t a[] = "foo";
        static const uint8_t b[] = "bar";
        uint8_t ab[sizeof(a) - 1 + sizeof(b) - 1];
        memset(key2, 0x11, sizeof(key2));
        memcpy(ab, a, sizeof(a) - 1);
        memcpy(ab + sizeof(a) - 1, b, sizeof(b) - 1);
        tesla_hmac_sha256_2(key2, sizeof(key2), a, sizeof(a) - 1, b, sizeof(b) - 1, d1);
        tesla_hmac_sha256(key2, sizeof(key2), ab, sizeof(ab), d2);
        CHECK(bytes_eq(d1, d2, TESLA_HMAC_LEN),
              "hmac_sha256_2(a, b) equals one-shot hmac over a||b");
    }
}

static void test_ct_equal(void)
{
    uint8_t a[16], b[16];
    memset(a, 0xAB, sizeof(a));
    memset(b, 0xAB, sizeof(b));
    CHECK(tesla_ct_equal(a, b, sizeof(a)) == true, "ct_equal: identical -> true");
    b[5] ^= 0x01;
    CHECK(tesla_ct_equal(a, b, sizeof(a)) == false, "ct_equal: single-bit diff -> false");
    CHECK(tesla_ct_equal(a, b, 0) == true, "ct_equal: zero-length -> true");
}

static void test_derive_shared_key(void)
{
    uint8_t priv[TESLA_PRIVKEY_LEN];
    uint8_t pub[TESLA_PUBKEY_LEN];
    uint8_t k[TESLA_SHARED_KEY_LEN];

    CHECK(unhex(CLIENT_PRIV_HEX, priv, sizeof(priv)) == TESLA_PRIVKEY_LEN, "decode client priv");
    CHECK(unhex(VEHICLE_PUB_HEX, pub, sizeof(pub)) == TESLA_PUBKEY_LEN, "decode vehicle pub");

    CHECK(tesla_derive_shared_key(priv, pub, dummy_rng, NULL, k) == 0 &&
          check_hex(k, TESLA_SHARED_KEY_LEN, K_HEX),
          "K = SHA1(ECDH(c, V).x)[:16] matches protocol.md (%s)", K_HEX);

    // Symmetry: the vehicle derives the same K from its private key and the
    // client public key.
    CHECK(unhex(VEHICLE_PRIV_HEX, priv, sizeof(priv)) == TESLA_PRIVKEY_LEN, "decode vehicle priv");
    CHECK(unhex(CLIENT_PUB_HEX, pub, sizeof(pub)) == TESLA_PUBKEY_LEN, "decode client pub");
    CHECK(tesla_derive_shared_key(priv, pub, dummy_rng, NULL, k) == 0 &&
          check_hex(k, TESLA_SHARED_KEY_LEN, K_HEX),
          "K = SHA1(ECDH(v, C).x)[:16] symmetric (same %s)", K_HEX);

    // A garbage public key must fail, not produce output.
    uint8_t bad[TESLA_PUBKEY_LEN];
    memset(bad, 0x00, sizeof(bad));
    CHECK(tesla_derive_shared_key(priv, bad, dummy_rng, NULL, k) != 0,
          "invalid peer public key rejected");

    // A well-formed (0x04-prefixed) but off-curve point must also be
    // rejected, not just unparseable input.
    uint8_t offcurve[TESLA_PUBKEY_LEN];
    CHECK(unhex(CLIENT_PUB_HEX, offcurve, sizeof(offcurve)) == TESLA_PUBKEY_LEN,
          "decode client pub for off-curve test");
    offcurve[64] ^= 0x01;   // corrupt Y: no longer satisfies y^2 = x^3 - 3x + b
    CHECK(tesla_derive_shared_key(priv, offcurve, dummy_rng, NULL, k) != 0,
          "off-curve peer public key rejected");

    // mbedTLS 3.x requires the RNG for ECDH blinding.
    CHECK(tesla_derive_shared_key(priv, pub, NULL, NULL, k) != 0,
          "NULL RNG rejected");
}

static void test_subkeys(void)
{
    uint8_t k[TESLA_SHARED_KEY_LEN];
    uint8_t d[TESLA_HMAC_LEN];

    unhex(K_HEX, k, sizeof(k));

    tesla_session_info_key(k, d);
    CHECK(check_hex(d, TESLA_HMAC_LEN, SESSION_KEY_HEX),
          "SESSION_INFO_KEY = HMAC-SHA256(K, \"session info\") matches protocol.md");

    tesla_authenticated_command_key(k, d);
    CHECK(check_hex(d, TESLA_HMAC_LEN, AUTH_CMD_KEY_HEX),
          "AUTH_CMD_KEY = HMAC-SHA256(K, \"authenticated command\") matches openssl");
}

static void test_metadata_build(void)
{
    // The request metadata for the protocol.md AES-GCM example must serialize
    // byte-for-byte to the documented hex string.
    uint8_t epoch[TESLA_EPOCH_LEN];
    uint8_t challenge[16];
    tesla_metadata_t m;
    const uint8_t *ser;
    size_t ser_len;

    unhex(EPOCH_HEX, epoch, sizeof(epoch));
    unhex(CHALLENGE_HEX, challenge, sizeof(challenge));

    CHECK(tesla_build_request_metadata(&m, TESLA_DOMAIN_INFOTAINMENT,
                                       (const uint8_t *)VIN, strlen(VIN),
                                       epoch, 2655, 7, TESLA_FLAG_ENCRYPT_RESPONSE) == 0,
          "build request metadata");
    ser = tesla_metadata_serialize(&m, &ser_len);
    CHECK(ser != NULL && check_hex(ser, ser_len, CMD_META_HEX),
          "request metadata serializes to protocol.md string");

    // Serialize is idempotent (0xFF appended once).
    const uint8_t *ser2;
    size_t ser2_len;
    ser2 = tesla_metadata_serialize(&m, &ser2_len);
    CHECK(ser2_len == ser_len && bytes_eq(ser, ser2, ser_len),
          "metadata serialize idempotent");

    // Session-info metadata: sig_type=HMAC, VIN, challenge, 0xFF.
    tesla_metadata_init(&m);
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_SIGNATURE_TYPE,
                             (const uint8_t[]){ TESLA_SIG_TYPE_HMAC }, 1) == 0, "add sig type");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_PERSONALIZATION,
                             (const uint8_t *)VIN, strlen(VIN)) == 0, "add vin");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_CHALLENGE, challenge, sizeof(challenge)) == 0,
          "add challenge");
    ser = tesla_metadata_serialize(&m, &ser_len);
    CHECK(ser != NULL && check_hex(ser, ser_len, SESSION_META_HEX),
          "session-info metadata serializes to protocol.md string");
}

static void test_metadata_checksum(void)
{
    // SHA-256 of the metadata item set pins the reference metadata_test.go
    // vector (also validates our TLV encoding independently of protocol.md).
    uint8_t epoch[TESLA_EPOCH_LEN];
    tesla_metadata_t m;
    const uint8_t *ser;
    size_t ser_len;
    uint8_t digest[TESLA_SHA256_LEN];

    unhex(META2_EPOCH_HEX, epoch, sizeof(epoch));
    tesla_metadata_init(&m);
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_SIGNATURE_TYPE,
                             (const uint8_t[]){ 0x05 }, 1) == 0, "add sig type");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_DOMAIN,
                             (const uint8_t[]){ 0x02 }, 1) == 0, "add domain");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_PERSONALIZATION,
                             (const uint8_t *)"testVIN", 7) == 0, "add vin");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_EPOCH, epoch, sizeof(epoch)) == 0, "add epoch");
    CHECK(tesla_metadata_add_u32(&m, TESLA_META_TAG_EXPIRES_AT, 3700) == 0, "add expires");
    CHECK(tesla_metadata_add_u32(&m, TESLA_META_TAG_COUNTER, 1338) == 0, "add counter");
    ser = tesla_metadata_serialize(&m, &ser_len);
    CHECK(ser != NULL && tesla_sha256(ser, ser_len, digest) == 0 &&
          check_hex(digest, TESLA_SHA256_LEN, META2_SHA256_HEX),
          "metadata SHA-256 matches reference metadata_test.go vector");
}

static void test_metadata_rejections(void)
{
    tesla_metadata_t m;
    uint8_t big[256];

    memset(big, 0xAA, sizeof(big));

    // Out-of-order tags.
    tesla_metadata_init(&m);
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_DOMAIN, (const uint8_t[]){ 0x01 }, 1) == 0,
          "add tag 1");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_SIGNATURE_TYPE, (const uint8_t[]){ 0x05 }, 1) != 0,
          "add tag 0 after tag 1 rejected");
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_DOMAIN, (const uint8_t[]){ 0x02 }, 1) != 0,
          "duplicate tag rejected");

    // Value longer than the one-byte TLV length field.
    tesla_metadata_init(&m);
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_PERSONALIZATION, big, sizeof(big)) != 0,
          "256-byte value rejected");

    // Add after finalize.
    tesla_metadata_init(&m);
    size_t len;
    (void)tesla_metadata_serialize(&m, &len);
    CHECK(tesla_metadata_add(&m, TESLA_META_TAG_DOMAIN, (const uint8_t[]){ 0x01 }, 1) != 0,
          "add after finalize rejected");
}

static void test_session_info_tag(void)
{
    uint8_t k[TESLA_SHARED_KEY_LEN];
    uint8_t challenge[16];
    uint8_t info[128];
    uint8_t tag[TESLA_HMAC_LEN];
    int info_len;

    unhex(K_HEX, k, sizeof(k));
    unhex(CHALLENGE_HEX, challenge, sizeof(challenge));
    info_len = unhex(SESSION_INFO_HEX, info, sizeof(info));
    CHECK(info_len > 0, "decode session info");

    CHECK(tesla_session_info_tag(k, (const uint8_t *)VIN, strlen(VIN),
                                 challenge, sizeof(challenge),
                                 info, (size_t)info_len, tag) == 0 &&
          check_hex(tag, TESLA_HMAC_LEN, SESSION_TAG_HEX),
          "session-info HMAC matches protocol.md tag");

    CHECK(tesla_verify_session_info(k, (const uint8_t *)VIN, strlen(VIN),
                                    challenge, sizeof(challenge),
                                    info, (size_t)info_len, tag, sizeof(tag)) == true,
          "verify_session_info accepts valid tag");

    tag[0] ^= 0x01;
    CHECK(tesla_verify_session_info(k, (const uint8_t *)VIN, strlen(VIN),
                                    challenge, sizeof(challenge),
                                    info, (size_t)info_len, tag, sizeof(tag)) == false,
          "verify_session_info rejects tampered tag");
    CHECK(tesla_verify_session_info(k, (const uint8_t *)VIN, strlen(VIN),
                                    challenge, sizeof(challenge),
                                    info, (size_t)info_len, tag, 16) == false,
          "verify_session_info rejects short tag");
}

static void test_gcm_vectors(void)
{
    uint8_t k[TESLA_SHARED_KEY_LEN];
    uint8_t nonce[TESLA_NONCE_LEN];
    uint8_t meta[128];
    uint8_t aad[TESLA_SHA256_LEN];
    uint8_t pt[64];
    uint8_t ct[64];
    uint8_t tag[TESLA_GCM_TAG_LEN];
    uint8_t back[64];
    int meta_len, pt_len;
    int rc;

    unhex(K_HEX, k, sizeof(k));
    unhex(NONCE_HEX, nonce, sizeof(nonce));
    meta_len = unhex(CMD_META_HEX, meta, sizeof(meta));
    pt_len = unhex(PLAINTEXT_HEX, pt, sizeof(pt));
    CHECK(meta_len > 0 && pt_len > 0, "decode AES-GCM example");

    // AAD = SHA256(M).
    CHECK(tesla_sha256(meta, (size_t)meta_len, aad) == 0, "sha256(metadata)");

    rc = tesla_gcm_encrypt(k, pt, (size_t)pt_len, aad, sizeof(aad), nonce, ct, tag);
    CHECK(rc == 0 && check_hex(ct, (size_t)pt_len, CIPHERTEXT_HEX) &&
          check_hex(tag, TESLA_GCM_TAG_LEN, GCM_TAG_HEX),
          "AES-128-GCM encrypt matches protocol.md ciphertext+tag");

    rc = tesla_gcm_decrypt(k, ct, (size_t)pt_len, aad, sizeof(aad), nonce, tag, back);
    CHECK(rc == 0 && bytes_eq(back, pt, (size_t)pt_len),
          "AES-128-GCM decrypt recovers plaintext");

    tag[15] ^= 0x01;
    rc = tesla_gcm_decrypt(k, ct, (size_t)pt_len, aad, sizeof(aad), nonce, tag, back);
    CHECK(rc == MBEDTLS_ERR_GCM_AUTH_FAILED,
          "tampered tag rejected (rc=%d)", rc);
}

// Full sign -> verify -> request-hash -> response-encrypt -> response-decrypt
// round trip, entirely off the car, against the protocol.md key material.
static void test_roundtrip(void)
{
    uint8_t k[TESLA_SHARED_KEY_LEN];
    uint8_t epoch[TESLA_EPOCH_LEN];
    uint8_t nonce[TESLA_NONCE_LEN];
    uint8_t nonce2[TESLA_NONCE_LEN];
    uint8_t pt[64];
    uint8_t ct[64];
    uint8_t ct2[64];
    uint8_t tag[TESLA_GCM_TAG_LEN];
    uint8_t tag2[TESLA_GCM_TAG_LEN];
    uint8_t back[64];
    uint8_t req_hash[TESLA_HMAC_LEN + 1];
    size_t req_hash_len;
    tesla_metadata_t m;
    const uint8_t *ser;
    size_t ser_len;
    uint8_t aad_req[TESLA_SHA256_LEN];
    uint8_t aad_resp[TESLA_SHA256_LEN];
    int pt_len;
    int rc;

    unhex(K_HEX, k, sizeof(k));
    unhex(EPOCH_HEX, epoch, sizeof(epoch));
    unhex(NONCE_HEX, nonce, sizeof(nonce));
    pt_len = unhex(PLAINTEXT_HEX, pt, sizeof(pt));

    // --- Client signs: build the command metadata and encrypt ---
    CHECK(tesla_build_request_metadata(&m, TESLA_DOMAIN_INFOTAINMENT,
                                       (const uint8_t *)VIN, strlen(VIN),
                                       epoch, 2655, 7, TESLA_FLAG_ENCRYPT_RESPONSE) == 0,
          "build request metadata");
    ser = tesla_metadata_serialize(&m, &ser_len);
    CHECK(ser != NULL && tesla_sha256(ser, ser_len, aad_req) == 0,
          "aad_req = sha256(request metadata)");
    rc = tesla_gcm_encrypt(k, pt, (size_t)pt_len, aad_req, sizeof(aad_req), nonce, ct, tag);
    CHECK(rc == 0, "encrypt command");

    // --- Vehicle verifies the command using the same request metadata ---
    rc = tesla_gcm_decrypt(k, ct, (size_t)pt_len, aad_req, sizeof(aad_req), nonce, tag, back);
    CHECK(rc == 0 && bytes_eq(back, pt, (size_t)pt_len),
          "vehicle verifies the command");

    // --- Request hash: [sig_type 0x05] || AES-GCM tag (17 bytes). Truncation
    // is a Vehicle-Security-domain rule; AES-GCM tags are 16 bytes so it is a
    // no-op here either way (see test_request_hash for the HMAC cases). ---
    CHECK(tesla_request_hash(TESLA_SIG_TYPE_AES_GCM_PERSONALIZED, tag, sizeof(tag),
                             false, req_hash, &req_hash_len) == 0 &&
          req_hash_len == 17 && req_hash[0] == TESLA_SIG_TYPE_AES_GCM_PERSONALIZED,
          "request hash = [0x05]||tag, 17 bytes");

    // --- Vehicle encrypts the response using the response metadata ---
    CHECK(tesla_build_response_metadata(&m, TESLA_DOMAIN_INFOTAINMENT,
                                        (const uint8_t *)VIN, strlen(VIN),
                                        7, TESLA_FLAG_ENCRYPT_RESPONSE,
                                        req_hash, req_hash_len, 0) == 0,
          "build response metadata");
    ser = tesla_metadata_serialize(&m, &ser_len);
    CHECK(ser != NULL && tesla_sha256(ser, ser_len, aad_resp) == 0,
          "aad_resp = sha256(response metadata)");
    // The vehicle draws its own nonce for the response.
    unhex("aabbccddeeff001122334455", nonce2, sizeof(nonce2));
    rc = tesla_gcm_encrypt(k, pt, (size_t)pt_len, aad_resp, sizeof(aad_resp),
                           nonce2, ct2, tag2);
    CHECK(rc == 0, "vehicle encrypts response");

    // --- Client decrypts the response ---
    rc = tesla_gcm_decrypt(k, ct2, (size_t)pt_len, aad_resp, sizeof(aad_resp),
                           nonce2, tag2, back);
    CHECK(rc == 0 && bytes_eq(back, pt, (size_t)pt_len),
          "client decrypts response (sign->decrypt round trip)");

    // The response must not authenticate under the request metadata: AAD is
    // bound to the metadata set.
    rc = tesla_gcm_decrypt(k, ct2, (size_t)pt_len, aad_req, sizeof(aad_req),
                           nonce2, tag2, back);
    CHECK(rc == MBEDTLS_ERR_GCM_AUTH_FAILED,
          "response under wrong (request) AAD rejected (rc=%d)", rc);

    // Tamper with the command ciphertext: authentication must fail.
    ct[0] ^= 0x01;
    rc = tesla_gcm_decrypt(k, ct, (size_t)pt_len, aad_req, sizeof(aad_req), nonce, tag, back);
    CHECK(rc == MBEDTLS_ERR_GCM_AUTH_FAILED,
          "tampered ciphertext rejected (rc=%d)", rc);
}

static void test_request_hash(void)
{
    uint8_t tag[32];
    uint8_t h[TESLA_HMAC_LEN + 1];
    size_t h_len;

    memset(tag, 0x42, sizeof(tag));

    // AES-GCM tag (16 bytes): [0x05] || tag = 17 bytes, no truncation effect.
    tesla_request_hash(TESLA_SIG_TYPE_AES_GCM_PERSONALIZED, tag, 16, false, h, &h_len);
    CHECK(h_len == 17 && h[0] == TESLA_SIG_TYPE_AES_GCM_PERSONALIZED &&
          memcmp(h + 1, tag, 16) == 0, "AES-GCM request hash = [0x05]||tag(16)");

    // HMAC tag to VCSEC: truncated to 17 bytes.
    tesla_request_hash(TESLA_SIG_TYPE_HMAC_PERSONALIZED, tag, 32, true, h, &h_len);
    CHECK(h_len == 17 && h[0] == TESLA_SIG_TYPE_HMAC_PERSONALIZED &&
          memcmp(h + 1, tag, 16) == 0, "VCSEC HMAC request hash truncated to 17 bytes");

    // HMAC tag to Infotainment: full [0x08] || tag(32) = 33 bytes.
    tesla_request_hash(TESLA_SIG_TYPE_HMAC_PERSONALIZED, tag, 32, false, h, &h_len);
    CHECK(h_len == 33 && h[0] == TESLA_SIG_TYPE_HMAC_PERSONALIZED &&
          memcmp(h + 1, tag, 32) == 0, "Infotainment HMAC request hash = [0x08]||tag(32)");
}

int main(void)
{
    test_hashes_and_hmac();
    test_ct_equal();
    test_derive_shared_key();
    test_subkeys();
    test_metadata_build();
    test_metadata_checksum();
    test_metadata_rejections();
    test_session_info_tag();
    test_gcm_vectors();
    test_roundtrip();
    test_request_hash();

    printf("\n%s\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED");
    return g_fail ? 1 : 0;
}
