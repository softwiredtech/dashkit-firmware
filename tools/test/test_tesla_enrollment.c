// Host unit test for the Phase 3 present-key enrollment pieces:
// keypair generation (crypto.c) and the ToVCSECMessage enrollment builder
// (protobuf_build.c).
//
// Compiled with plain gcc against a host mbedTLS 3.6.2 build (same major.minor
// as the ESP-IDF 5.4.1 vendored copy). See run_tesla_enrollment_test.sh.
//
// The enrollment message is structural: the builder marshals a
// ToVCSECMessage{SignedMessage{ protobufMessageAsBytes = the addKey
// WhitelistOperation, signatureType = SIGNATURE_TYPE_PRESENT_KEY }} and is sent
// with no appended signature — the car authorizes the enrollment physically
// (NFC-card tap + touchscreen confirm), per the vehicle-command reference's
// SendAddKeyRequestWithRole. The test round-trips the message through the
// nanopb bindings and checks role / form factor / embedded public key.

#include "crypto.h"
#include "protobuf_build.h"
#include "keys.pb.h"
#include "vcsec.pb.h"
#include "universal_message.pb.h"
#include "pb_decode.h"

#include "mbedtls/bignum.h"
#include "mbedtls/ecp.h"

#include <stdio.h>
#include <string.h>

// ---- tiny test harness (mirrors test_tesla_crypto.c) ----
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

// Deterministic RNG for EC blinding in the keygen/derive helpers.
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

// Derive the uncompressed 65-byte public key for a private scalar (host-side,
// used to confirm a generated keypair's pub == priv*G).
static int pub_from_priv(const uint8_t priv[32], uint8_t pub[65])
{
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    size_t olen = 0;
    int rc;

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    rc = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (rc == 0) rc = mbedtls_mpi_read_binary(&d, priv, 32);
    if (rc == 0) rc = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, dummy_rng, NULL);
    if (rc == 0) {
        rc = mbedtls_ecp_point_write_binary(&grp, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                            &olen, pub, 65);
    }
    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    return rc;
}

static void test_keygen(void)
{
    tesla_keypair_t key;
    uint8_t pub[65];

    CHECK(tesla_keypair_generate(&key, dummy_rng, NULL) == 0, "keypair generates");
    CHECK(key.pub[0] == 0x04, "public key is uncompressed (0x04 prefix)");
    CHECK(pub_from_priv(key.priv, pub) == 0, "derive pub from generated priv");
    CHECK(bytes_eq(pub, key.pub, 65), "generated pub == priv*G");
    CHECK(tesla_keypair_generate(NULL, dummy_rng, NULL) != 0,
          "NULL key rejected");
    CHECK(tesla_keypair_generate(&key, NULL, NULL) != 0,
          "NULL RNG rejected");
}

static void test_enrollment(void)
{
    tesla_keypair_t key;
    uint8_t out[320];
    size_t out_len = 0;

    CHECK(tesla_keypair_generate(&key, dummy_rng, NULL) == 0, "gen key for enrollment");

    CHECK(tesla_pb_build_enrollment(&key, Keys_Role_ROLE_CHARGING_MANAGER,
                                    VCSEC_KeyFormFactor_KEY_FORM_FACTOR_ANDROID_DEVICE,
                                    out, sizeof(out), &out_len) == 0,
          "enrollment message builds");
    CHECK(out_len > 0, "enrollment message non-empty");

    // Decode the marshalled ToVCSECMessage and inspect the SignedMessage.
    VCSEC_ToVCSECMessage env;
    memset(&env, 0, sizeof(env));
    pb_istream_t s = pb_istream_from_buffer(out, out_len);
    CHECK(pb_decode(&s, VCSEC_ToVCSECMessage_fields, &env), "ToVCSECMessage decodes");
    CHECK(env.has_signedMessage, "SignedMessage present");
    CHECK(env.signedMessage.signatureType ==
              VCSEC_SignatureType_SIGNATURE_TYPE_PRESENT_KEY,
          "signatureType is PRESENT_KEY");

    // The SignedMessage.protobufMessageAsBytes holds the inner UnsignedMessage.
    VCSEC_UnsignedMessage inner;
    memset(&inner, 0, sizeof(inner));
    pb_istream_t s2 = pb_istream_from_buffer(
        env.signedMessage.protobufMessageAsBytes.bytes,
        env.signedMessage.protobufMessageAsBytes.size);
    CHECK(pb_decode(&s2, VCSEC_UnsignedMessage_fields, &inner),
          "inner UnsignedMessage decodes");
    CHECK(inner.which_sub_message ==
              (pb_size_t)VCSEC_UnsignedMessage_WhitelistOperation_tag,
          "inner is a WhitelistOperation");
    CHECK(inner.sub_message.WhitelistOperation.which_sub_message ==
              (pb_size_t)VCSEC_WhitelistOperation_addKeyToWhitelistAndAddPermissions_tag,
          "inner op is addKeyToWhitelistAndAddPermissions");
    const VCSEC_PermissionChange *pc =
        &inner.sub_message.WhitelistOperation.sub_message
             .addKeyToWhitelistAndAddPermissions;
    CHECK(pc->keyRole == Keys_Role_ROLE_CHARGING_MANAGER,
          "role is CHARGING_MANAGER");
    CHECK(pc->secondsToBeActive == 0, "permanent key (secondsToBeActive == 0)");
    CHECK(pc->key.PublicKeyRaw.size == 65 &&
          bytes_eq(pc->key.PublicKeyRaw.bytes, key.pub, 65),
          "embedded public key matches the enrolled key");
    CHECK(inner.sub_message.WhitelistOperation.has_metadataForKey &&
          inner.sub_message.WhitelistOperation.metadataForKey.keyFormFactor ==
              VCSEC_KeyFormFactor_KEY_FORM_FACTOR_ANDROID_DEVICE,
          "form factor is ANDROID_DEVICE");
}

int main(void)
{
    printf("Tesla BLE Phase 3 enrollment tests (keygen + present-key message)\n");
    printf("------------------------------------------------------------------\n");
    test_keygen();
    test_enrollment();

    if (g_fail == 0) {
        printf("All enrollment tests passed\n");
        return 0;
    }
    printf("%d enrollment test(s) FAILED\n", g_fail);
    return 1;
}
