#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_APP_MAX_CONNS 2
#define BLE_APP_NO_CONN UINT16_MAX

typedef struct {
    uint16_t handle;
    bool encrypted;
    bool subscribed;
    bool read_only;
    bool peer_was_bonded;
    bool ping_seen;
    bool pending_connect;
    uint32_t last_ping_s;
    uint32_t connected_s;
} ble_app_conn_t;

static inline bool ble_app_can_receive(const ble_app_conn_t *conn)
{
    return conn->handle != BLE_APP_NO_CONN && conn->encrypted && conn->subscribed;
}

static inline bool ble_app_has_receivers(const ble_app_conn_t *conns)
{
    for (size_t i = 0; i < BLE_APP_MAX_CONNS; i++) {
        if (ble_app_can_receive(&conns[i])) return true;
    }
    return false;
}

// Write permission belongs to each connection, independently of subscriptions
// and of the other client's role.
static inline bool ble_app_can_write(const ble_app_conn_t *conn)
{
    return conn->handle != BLE_APP_NO_CONN && conn->encrypted && !conn->read_only;
}

// Validate a complete DashPilot CAN batch before sending any of its frames.
static inline bool ble_app_valid_batch(const uint8_t *data, size_t len)
{
    if (!data || len < 1 || len > 512 || data[0] == 0) return false;
    size_t offset = 1;
    for (unsigned i = 0; i < data[0]; i++) {
        if (len - offset < 10 || data[offset + 9] > 8) return false;
        size_t frame_len = 10 + data[offset + 9];
        if (frame_len > len - offset) return false;
        offset += frame_len;
    }
    return offset == len;
}

// Input must be validated first. Returns the next batch fitting the ATT payload
// capacity, or zero without advancing when no frame fits.
static inline size_t ble_app_next_packet(const uint8_t *data, size_t len,
                                         size_t *offset, uint8_t *out,
                                         size_t capacity)
{
    if (capacity < 1) return 0;
    size_t pos = 1;
    out[0] = 0;
    while (*offset < len) {
        size_t frame_len = 10 + data[*offset + 9];
        if (frame_len > capacity - pos) break;
        for (size_t i = 0; i < frame_len; i++) out[pos + i] = data[*offset + i];
        pos += frame_len;
        *offset += frame_len;
        out[0]++;
    }
    return out[0] ? pos : 0;
}
