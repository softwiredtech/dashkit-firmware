/* Tesla BLE advertisement-name matcher. See tesla_advert_name.h. */

#include "tesla_advert_name.h"

static int is_hex(unsigned char c)
{
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

int tesla_vin_char(unsigned char c)
{
    if (c >= '0' && c <= '9') {
        return 1;
    }
    if (c < 'A' || c > 'Z') {
        return 0;
    }
    /* VINs never use I, O, or Q. */
    return (c != 'I' && c != 'O' && c != 'Q');
}

/* Legacy: "S" + (8 or 16) hex chars + one of C/R/D/P.
 *
 * The 8-hex form (total length 10) is what the original fake-beacon Phase-1
 * tests used. Real Tesla vehicles advertise "S" + the first 16 hex chars of
 * SHA1(VIN) + role letter (total length 18), per vehicle-command/teslabtapi;
 * e.g. VIN 5YJ3E1EB3MF074051 -> Sf9cd80ddffdd5492C. A real car was seen on-air
 * with the 18-char name (2026-08-19) after this was tightened to accept both,
 * so accept either hex length for both dev beacons and real vehicles.
 */
static int name_is_legacy(const uint8_t *name, size_t len)
{
    size_t hex_len, i;

    if (name[0] != 'S') {
        return 0;
    }
    if (len == 10) {
        hex_len = 8;
    } else if (len == 18) {
        hex_len = 16;
    } else {
        return 0;
    }
    for (i = 0; i < hex_len; i++) {
        if (!is_hex(name[1 + i])) {
            return 0;
        }
    }
    switch (name[len - 1]) {
    case 'C':
    case 'R':
    case 'D':
    case 'P':
        return 1;
    default:
        return 0;
    }
}

/* Modern: "Tesla " + 4..6 VIN-alphabet chars.
 *
 * The documented format is "Tesla <last 6 of VIN>", but some vehicles
 * advertise a shorter suffix, so accept a 4..6 character tail rather than the
 * strict 6. (This tolerance is for shorter suffixes only — the BLE stack
 * delivers at most one name AD element, so a name can never be "split" across
 * Complete+Shortened name types; if you ever see a split name it must be
 * handled in the adapter, not here.) Every byte must be a valid VIN character
 * so random "Tesla ..." names can never false-positive. */
static int name_is_modern(const uint8_t *name, size_t len)
{
    static const char prefix[] = "Tesla ";
    const size_t prefix_len = sizeof(prefix) - 1;  /* 6, excludes the NUL */
    size_t i;

    if (len < prefix_len + 4 || len > prefix_len + 6) {
        return 0;
    }
    for (i = 0; i < prefix_len; i++) {
        if (name[i] != (uint8_t)prefix[i]) {
            return 0;
        }
    }
    for (; i < len; i++) {
        if (!tesla_vin_char(name[i])) {
            return 0;
        }
    }
    return 1;
}

enum tesla_name_format tesla_advert_name_format(const uint8_t *name, size_t len)
{
    if (name == NULL || len == 0) {
        return TESLA_NAME_NONE;
    }
    if (name_is_legacy(name, len)) {
        return TESLA_NAME_LEGACY;
    }
    if (name_is_modern(name, len)) {
        return TESLA_NAME_MODERN;
    }
    return TESLA_NAME_NONE;
}