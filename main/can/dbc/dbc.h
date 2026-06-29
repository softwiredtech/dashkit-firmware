#pragma once

// DBC-driven CAN engine: send and read Tesla CAN messages by name, with the
// bit layout, rolling counter and checksum all taken from the generated tables
// (dbc_generated.{c,h}, produced from dbc/bus_1_tesla_vehicle.dbc). This
// replaces the hand-rolled per-feature pack()/checksum()/hardcoded-id code.
//
// All whitelisted signals are little-endian (Intel); see tools/gen_dbc.py.

#include "can_interface.h"
#include "dbc_generated.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Handles are plain table indices, resolved by name (lock-free, no allocation).
typedef int16_t dbc_msg_t;  // index into g_dbc_messages, or DBC_MSG_INVALID
typedef int16_t dbc_sig_t;  // index into g_dbc_signals, or DBC_SIG_INVALID

#define DBC_MSG_INVALID  ((dbc_msg_t)-1)
#define DBC_SIG_INVALID  ((dbc_sig_t)-1)

// Reset rolling counters. Call once at boot before any can_send.
void dbc_init(void);

// Resolve a message / signal name to a handle (linear scan of the small tables).
dbc_msg_t dbc_msg(const char *name);
dbc_sig_t dbc_sig(dbc_msg_t msg, const char *name);

// Low-level RAW pack/unpack on a data buffer (little-endian / Intel order).
// "Raw" = the integer field as it sits on the wire, BEFORE scale/offset.
// dbc_pack overwrites only the signal's bits; dbc_unpack sign-extends signed
// signals. int64 carries any signal width (incl. 32-bit unsigned) and sign.
// Exposed for host unit tests and advanced callers.
void    dbc_pack(uint8_t *data, dbc_sig_t sig, int64_t value);
int64_t dbc_unpack(const uint8_t *data, dbc_sig_t sig);

// Raw <-> physical conversion: phys = raw * scale + offset. phys_to_raw rounds
// to the nearest integer raw value (llround). Used internally by the scaling
// path; exposed for tests / advanced callers.
double  dbc_raw_to_phys(dbc_sig_t sig, int64_t raw);
int64_t dbc_phys_to_raw(dbc_sig_t sig, double phys);

// ---- Read / send by name (mirrors cantools decode/encode + its `scaling`) ----
// `scaling` selects the value representation, exactly like cantools'
// decode_message/encode_message scaling kwarg:
//   true  -> PHYSICAL value (raw * scale + offset); for telemetry / scaled sigs.
//   false -> RAW value (exact on-wire integer bits); for flags / enums / counters.
// `double` carries both: raw integers are exact up to 2^53.

// Read a signal from the can_manager frame cache. Mux-aware (returns
// ESP_ERR_INVALID_STATE if the live frame carries a different mux). Requires the
// message to be watched (dbc_watch / subscribe).
esp_err_t can_get(uint8_t bus, const char *msg, const char *sig, double *out, bool scaling);

// One-shot send: zeroed frame, pack one signal (auto-writing its mux selector),
// auto counter + checksum, transmit on `bus`. Names resolved internally.
esp_err_t can_send(uint8_t bus, const char *msg, const char *sig, double value, bool scaling);

// Read-modify-write one-shot: like can_send but seeds the frame from the live
// cached bytes so every other signal (including bits no DBC signal covers) is
// preserved. Returns ESP_ERR_NOT_FOUND if no live frame has been received yet.
esp_err_t can_send_live(uint8_t bus, const char *msg, const char *sig, double value, bool scaling);

// Register a message id with the can_manager frame cache so it can be read
// (can_get) or used as an RMW base (can_send_live / can_frame_live).
esp_err_t dbc_watch(uint8_t bus, const char *msg);

// Seed a message's rolling counter from the live bus frame so the next send
// continues the car's sequence: counter is set to the last value seen, and the
// next send emits last+1. Requires the message watched.
esp_err_t dbc_counter_seed_from_bus(uint8_t bus, const char *msg);

// ---- Raw-frame helpers (for payloads spanning bits no DBC signal names) ----
// Only needed when you must write reserved/unnamed bytes directly (e.g. the
// battery-preheat magic payload). Normal signal traffic should use can_send /
// can_send_live. These are stateless: you own the can_frame_t.

// Zero `out` and set its id/dlc from the DBC (a from-scratch base).
esp_err_t can_frame_init(const char *msg, can_frame_t *out);

// Copy the live cached frame into `out` (RMW base). ESP_ERR_NOT_FOUND if none.
esp_err_t can_frame_live(uint8_t bus, const char *msg, can_frame_t *out);

// Apply the message's rolling counter (+1) and checksum to `f`, then transmit.
esp_err_t can_frame_send(uint8_t bus, const char *msg, can_frame_t *f);

#ifdef __cplusplus
}
#endif
