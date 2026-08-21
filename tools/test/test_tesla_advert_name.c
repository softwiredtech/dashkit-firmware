// Host unit test for the Tesla advertisement-name matcher
// (main/tesla/tesla_advert_name.c).
//
// The matcher itself is pure C and needs no libraries, but the test also
// derives a VIN -> advertisement-name vector (legacy: "S"+first-8-hex of
// SHA1(VIN)+"C"; modern: "Tesla "+last-6-of-VIN), so it links the same host
// mbedTLS 3.6.2 build the Phase 0 crypto test uses — run via
// run_tesla_advert_name_test.sh, which reused that prefix.
//
// Optionally pass a VIN on the command line; the test then prints the two
// exact advertisement names a fake beacon must broadcast for that VIN (and
// verifies the matcher accepts both).
//
//   gcc -std=c99 -Wall -Wextra -I main -I main/tesla -I <mbedtls>/include
//       tools/test/test_tesla_advert_name.c main/tesla/tesla_advert_name.c
//       -L <mbedtls>/lib -lmbedcrypto
//       -o /tmp/test_tesla_advert_name && /tmp/test_tesla_advert_name

#include "tesla_advert_name.h"

#include <stdio.h>
#include <string.h>

#include "mbedtls/sha1.h"

// ---- tiny test harness (mirrors test_tesla_crypto.c) ----
static int g_fail;
#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_fail++; } \
    else { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

// Derive the legacy advertisement name for a VIN into out (must hold 19
// bytes: 'S' + first 16 hex chars of SHA1(VIN) + trailing format char + NUL).
// Matches the real Tesla broadcast: "S" + first 16 hex of SHA1(VIN) + role
// letter (teslabtapi / vehicle-command), e.g. VIN 5YJ3E1EB3MF074051 ->
// Sf9cd80ddffdd5492C. (The short 8-hex form is *not* produced by real cars; it
// was only used by the original fake-beacon tests, and the matcher accepts it
// too for backward compatibility.)
static void legacy_name(const char *vin, char fmt_char, char out[19])
{
    unsigned char digest[20];
    size_t n = strlen(vin);

    mbedtls_sha1((const unsigned char *)vin, n, digest);
    snprintf(out, 19, "S%02X%02X%02X%02X%02X%02X%02X%02X%c",
             digest[0], digest[1], digest[2], digest[3],
             digest[4], digest[5], digest[6], digest[7], fmt_char);
}

// "Tesla " + last 6 characters of the VIN.
static void modern_name(const char *vin, char out[13])
{
    size_t n = strlen(vin);
    // NOTE: this helper is only ever called with a full 17-char VIN. For a
    // shorter input it would emit "Tesla " + a too-short tail, which the
    // matcher correctly rejects — so a failing CHECK below could be misread as
    // a matcher bug. VINs are fixed-length; keep that expectation explicit.
    snprintf(out, 13, "Tesla %s", n >= 6 ? vin + n - 6 : vin);
}

static void check_vin_char(void)
{
    CHECK(tesla_vin_char('0') && tesla_vin_char('9'),
          "digits are VIN characters");
    CHECK(tesla_vin_char('A') && tesla_vin_char('Z'),
          "uppercase letters are VIN characters");
    CHECK(!tesla_vin_char('I') && !tesla_vin_char('O') && !tesla_vin_char('Q'),
          "I/O/Q are excluded from VINs");
    CHECK(!tesla_vin_char('a') && !tesla_vin_char('-') && !tesla_vin_char(' '),
          "lowercase/punct/space are not VIN characters");
}

static void check_legacy(void)
{
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234C", 10) == TESLA_NAME_LEGACY,
          "legacy: S + 8 hex + C");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234R", 10) == TESLA_NAME_LEGACY,
          "legacy: trailing R accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234D", 10) == TESLA_NAME_LEGACY,
          "legacy: trailing D accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234P", 10) == TESLA_NAME_LEGACY,
          "legacy: trailing P accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"SAbCdEf01C", 10) == TESLA_NAME_LEGACY,
          "legacy: mixed-case hex accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234X", 10) == TESLA_NAME_NONE,
          "legacy: unknown trailing letter rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234", 9) == TESLA_NAME_NONE,
          "legacy: wrong length rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd12345C", 11) == TESLA_NAME_NONE,
          "legacy: too long rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd123ZC", 10) == TESLA_NAME_NONE,
          "legacy: non-hex hash field rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sabcd1234c", 10) == TESLA_NAME_NONE,
          "legacy: lowercase trailing role char rejected (case-sensitive)");
    CHECK(tesla_advert_name_format((const uint8_t *)"Xabcd1234C", 10) == TESLA_NAME_NONE,
          "legacy: non-'S' first byte rejected");

    // Real Tesla legacy format: "S" + 16 hex + C/R/D/P (18 chars) — the
    // on-air real-car capture (VIN 5YJ3E1EB3MF074051 -> Sf9cd80ddffdd5492C).
    CHECK(tesla_advert_name_format((const uint8_t *)"Sf9cd80ddffdd5492C", 18) == TESLA_NAME_LEGACY,
          "legacy: S + 16 hex + C (real Tesla 18-char format)");
    CHECK(tesla_advert_name_format((const uint8_t *)"S12Ab9DeF00AbCdEfR", 18) == TESLA_NAME_LEGACY,
          "legacy: 16-hex mixed-case + trailing R accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sf9cd80ddffdd5492X", 18) == TESLA_NAME_NONE,
          "legacy: 16-hex with invalid role char rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sf9cd80ddffdd5492", 17) == TESLA_NAME_NONE,
          "legacy: 16-hex with no trailing role char rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sf9cd80ddffdd5492XX", 19) == TESLA_NAME_NONE,
          "legacy: 18-char length boundary honored (19 rejected)");
    CHECK(tesla_advert_name_format((const uint8_t *)"Sf9cd80ddffdd5492C", 17) == TESLA_NAME_NONE,
          "legacy: 18-char name with truncated length rejected");
}

static void check_modern(void)
{
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla A1B2C3", 12) == TESLA_NAME_MODERN,
          "modern: Tesla + 6 VIN chars");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla 789ABC", 12) == TESLA_NAME_MODERN,
          "modern: mixed alnum tail");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla 5YJ3", 10) == TESLA_NAME_MODERN,
          "modern: shorter 4-char tail accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla 12A34", 11) == TESLA_NAME_MODERN,
          "modern: 5-char tail (mid-boundary) accepted");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla 123456", 12) == TESLA_NAME_MODERN,
          "modern: pure-digit tail accepted (realistic VIN last-6)");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla A1", 8) == TESLA_NAME_NONE,
          "modern: tail too short rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla A1B2C3D4E5F6", 15) == TESLA_NAME_NONE,
          "modern: tail too long rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla ABC!", 10) == TESLA_NAME_NONE,
          "modern: non-VIN tail char rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"Tesla abcdef", 12) == TESLA_NAME_NONE,
          "modern: lowercase tail rejected (VIN chars are uppercase)");
    CHECK(tesla_advert_name_format((const uint8_t *)"tesla A1B2C3", 12) == TESLA_NAME_NONE,
          "modern: lowercase prefix rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"TeslaA1B2C3", 11) == TESLA_NAME_NONE,
          "modern: missing space rejected");
}

static void check_edges(void)
{
    CHECK(tesla_advert_name_format(NULL, 0) == TESLA_NAME_NONE,
          "NULL/0-length input rejected");
    CHECK(tesla_advert_name_format(NULL, 5) == TESLA_NAME_NONE,
          "NULL with nonzero length rejected");
    CHECK(tesla_advert_name_format((const uint8_t *)"", 0) == TESLA_NAME_NONE,
          "empty input rejected");
}

static void check_derive_accept(void)
{
    const char *vin = "5YJ30123456789ABC";
    char legacy[19];
    char modern[13];

    legacy_name(vin, 'C', legacy);
    modern_name(vin, modern);
    CHECK(tesla_advert_name_format((const uint8_t *)legacy, strlen(legacy)) == TESLA_NAME_LEGACY,
          "matcher accepts derived legacy name for sample VIN (%s)", legacy);
    CHECK(tesla_advert_name_format((const uint8_t *)modern, strlen(modern)) == TESLA_NAME_MODERN,
          "matcher accepts derived modern name for sample VIN (%s)", modern);

    /* Round-trip the derived strings against a structurally different name. */
    CHECK(strcmp(legacy, "Sabcd1234C") != 0,
          "derived legacy name differs from guess (expected, confirms derivation ran)");
    CHECK(strcmp(modern, "Tesla 789ABC") == 0,
          "modern derivation = Tesla last-6 of VIN (789ABC)");
}

int main(int argc, char **argv)
{
    const char *vin = argc > 1 ? argv[1] : "5YJ30123456789ABC";
    char legacy[19];
    char modern[13];

    printf("== tesla advert-name matcher ==\n");
    check_vin_char();
    check_legacy();
    check_modern();
    check_edges();
    check_derive_accept();

    if (argc > 1) {
        legacy_name(vin, 'C', legacy);
        modern_name(vin, modern);
        printf("\n%s\n", "== fake-beacon names for this VIN ==");
        printf("  legacy: %s\n", legacy);
        printf("  modern: %s\n", modern);
    }

    printf("\n%s (%d failure%s)\n",
           g_fail ? "RESULT: FAIL" : "RESULT: PASS", g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}