/*
 * Tesla BLE advertisement-name matcher (Phase 1, scan-only spike).
 *
 * A Tesla advertises under one of two documented formats (vehicle-command
 * protocol.md / ADR 0001):
 *
 *   Legacy (pre ~mid-2023): "S" + first 8 bytes (16 hex chars) of SHA1(VIN) + role
 *     (the trailing letter has been observed as C / R / D / P). On-air real car
 *     (5YJ3E1EB3MF074051 -> Sf9cd80ddffdd5492C). NOTE: an 8-hex (10-char) form is
 *     NOT a real Tesla broadcast — it was only the original dev/test fake beacon.
 *   Modern (since ~mid-2023): "Tesla " + last 6 characters of the VIN.
 *
 * This module only knows the SHAPE of those names (and, for the modern form,
 * the VIN character alphabet). It does not need the vehicle's VIN or any
 * crypto, so it is pure C and compiles into the host test suite unchanged
 * (see tools/test/test_tesla_advert_name.c).
 *
 * The matcher is shape-only by design for the Phase 1 done-when (print the
 * name+MAC of any nearby Tesla from advertisements). Binding a found vehicle
 * to OUR stored VIN (via SHA1(VIN) for the legacy format, or a direct suffix
 * match for the modern format) belongs to Phase 2+ once a VIN is persisted.
 */
#ifndef TESLA_ADVERT_NAME_H
#define TESLA_ADVERT_NAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum tesla_name_format {
    TESLA_NAME_NONE = 0,   /* not a Tesla advertisement name                 */
    TESLA_NAME_LEGACY,     /* S + 16 hex + C/R/D/P (real car); 8-hex is dev beacon only */
    TESLA_NAME_MODERN,     /* "Tesla " + 4..6 VIN-alphabet characters        */
};

/* Classifies a BLE advertisement local name. `name` is the raw bytes of the
 * advertised Name field (length-prefixed, not NUL-terminated) — pure ASCII.
 */
enum tesla_name_format tesla_advert_name_format(const uint8_t *name, size_t len);

/* 1 if c is a valid VIN character: 0-9 or uppercase A-Z excluding I, O, Q. */
int tesla_vin_char(unsigned char c);

#ifdef __cplusplus
}
#endif

#endif /* TESLA_ADVERT_NAME_H */