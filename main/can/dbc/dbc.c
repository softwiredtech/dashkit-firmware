#include "dbc.h"
#include "can_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <math.h>
#include <string.h>

static const char *TAG = "dbc";

// Rolling counter state, one slot per message (auto-incremented on send).
static uint32_t s_counter[DBC_MESSAGE_COUNT];

// Latest-frame cache, one slot per DBC message (mirrors opendbc's CANParser
// `vl`): every received frame whose id matches a known message is kept by
// dbc_observe_frame, so reads (can_get) and RMW sends (can_send_live /
// can_frame_live) need no per-message registration.
static can_frame_t  s_cache[DBC_MESSAGE_COUNT];
static bool         s_cache_valid[DBC_MESSAGE_COUNT];
static portMUX_TYPE s_cache_mux = portMUX_INITIALIZER_UNLOCKED;

void dbc_init(void)
{
    memset(s_counter, 0, sizeof(s_counter));
    memset(s_cache_valid, 0, sizeof(s_cache_valid));
    ESP_LOGI(TAG, "DBC engine ready (%d messages, %d signals)",
             DBC_MESSAGE_COUNT, DBC_SIGNAL_COUNT);
}

// Resolve the message a received frame belongs to (matched by id on its bus),
// or DBC_MSG_INVALID if the DBC has no definition for it.
static dbc_msg_t msg_by_frame(uint8_t bus, uint32_t id)
{
    for (int i = 0; i < DBC_MESSAGE_COUNT; i++) {
        if (g_dbc_messages[i].id == id && g_dbc_messages[i].default_bus == bus) {
            return (dbc_msg_t)i;
        }
    }
    return DBC_MSG_INVALID;
}

void dbc_observe_frame(const can_tagged_frame_t *f)
{
    dbc_msg_t mi = msg_by_frame(f->bus_id, f->frame.id);
    if (mi < 0) {
        return;  // no DBC definition for this frame: nothing to cache
    }
    portENTER_CRITICAL(&s_cache_mux);
    s_cache[mi] = f->frame;
    s_cache_valid[mi] = true;
    portEXIT_CRITICAL(&s_cache_mux);
}

// Copy the cached frame for message `mi`, or ESP_ERR_NOT_FOUND if none yet.
static esp_err_t cache_get(dbc_msg_t mi, can_frame_t *out)
{
    esp_err_t err = ESP_ERR_NOT_FOUND;
    portENTER_CRITICAL(&s_cache_mux);
    if (s_cache_valid[mi]) {
        *out = s_cache[mi];
        err = ESP_OK;
    }
    portEXIT_CRITICAL(&s_cache_mux);
    return err;
}

dbc_msg_t dbc_msg(const char *name)
{
    for (int i = 0; i < DBC_MESSAGE_COUNT; i++) {
        if (strcmp(g_dbc_messages[i].name, name) == 0) {
            return (dbc_msg_t)i;
        }
    }
    return DBC_MSG_INVALID;
}

dbc_sig_t dbc_sig(dbc_msg_t msg, const char *name)
{
    if (msg < 0 || msg >= DBC_MESSAGE_COUNT) {
        return DBC_SIG_INVALID;
    }
    const dbc_message_t *m = &g_dbc_messages[msg];
    for (int i = 0; i < m->sig_count; i++) {
        int gi = m->sig_first + i;
        if (strcmp(g_dbc_signals[gi].name, name) == 0) {
            return (dbc_sig_t)gi;
        }
    }
    return DBC_SIG_INVALID;
}

// Little-endian scatter: bit `start_bit` receives the LSB of value. Overwrites
// only the signal's bits, so it is safe on a live read-modify-write frame.
void dbc_pack(uint8_t *data, dbc_sig_t sig, int64_t value)
{
    const dbc_signal_t *s = &g_dbc_signals[sig];
    uint64_t v = (uint64_t)value;
    for (int i = 0; i < s->length; i++) {
        int bit = s->start_bit + i;
        if ((v >> i) & 1u) {
            data[bit / 8] |= (uint8_t)(1u << (bit % 8));
        } else {
            data[bit / 8] &= (uint8_t)~(1u << (bit % 8));
        }
    }
}

// Little-endian gather, sign-extended for signed signals.
int64_t dbc_unpack(const uint8_t *data, dbc_sig_t sig)
{
    const dbc_signal_t *s = &g_dbc_signals[sig];
    uint64_t v = 0;
    for (int i = 0; i < s->length; i++) {
        int bit = s->start_bit + i;
        if ((data[bit / 8] >> (bit % 8)) & 1u) {
            v |= ((uint64_t)1 << i);
        }
    }
    if (s->is_signed && s->length < 64 && (v & ((uint64_t)1 << (s->length - 1)))) {
        v |= ~(((uint64_t)1 << s->length) - 1);  // sign-extend
    }
    return (int64_t)v;
}

double dbc_raw_to_phys(dbc_sig_t sig, int64_t raw)
{
    const dbc_signal_t *s = &g_dbc_signals[sig];
    return (double)raw * s->scale + s->offset;
}

int64_t dbc_phys_to_raw(dbc_sig_t sig, double phys)
{
    const dbc_signal_t *s = &g_dbc_signals[sig];
    double scale = (s->scale != 0.0) ? s->scale : 1.0;
    return (int64_t)llround((phys - s->offset) / scale);
}

// Pack a signal into `data` plus, if it is multiplexed, its selector value.
static void pack_signal(uint8_t *data, dbc_sig_t sig, int64_t raw)
{
    const dbc_signal_t *s = &g_dbc_signals[sig];
    dbc_pack(data, sig, raw);
    if (s->mux_sel_sig >= 0) {
        dbc_pack(data, (dbc_sig_t)s->mux_sel_sig, s->mux_val);
    }
}

// Interpret a caller-supplied value as the raw on-wire integer. With scaling it
// is a physical value (phys = raw*scale+offset); without, it is the raw bits.
static int64_t value_to_raw(dbc_sig_t sig, double value, bool scaling)
{
    return scaling ? dbc_phys_to_raw(sig, value) : (int64_t)llround(value);
}

// Fill the rolling counter (+1, masked to its width) then the checksum. Counter
// first so the checksum, which covers every byte but its own, includes it.
static void frame_finalize(dbc_msg_t mi, can_frame_t *f)
{
    const dbc_message_t *m = &g_dbc_messages[mi];

    if (m->counter_sig >= 0) {
        const dbc_signal_t *cs = &g_dbc_signals[m->counter_sig];
        uint32_t mask = (cs->length >= 32) ? ~0u : ((1u << cs->length) - 1u);
        s_counter[mi] = (s_counter[mi] + 1) & mask;
        dbc_pack(f->data, m->counter_sig, (int64_t)s_counter[mi]);
    }

    if (m->checksum_algo == DBC_CKSUM_TESLA_BYTESUM && m->checksum_sig >= 0) {
        // Tesla checksum: (id_lo + id_hi + sum of all data bytes except the
        // checksum byte) & 0xFF. The checksum signal is always a whole byte.
        int cidx = g_dbc_signals[m->checksum_sig].start_bit / 8;
        uint8_t sum = (uint8_t)((m->id & 0xFF) + ((m->id >> 8) & 0xFF));
        for (int i = 0; i < f->dlc; i++) {
            if (i != cidx) {
                sum += f->data[i];
            }
        }
        f->data[cidx] = sum;
    }
}

// Resolve msg+sig, pack `value` (raw or physical) into `f`, finalize, transmit.
static esp_err_t send_frame(uint8_t bus, const char *msg, const char *sig,
                            double value, bool scaling, can_frame_t *f)
{
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        ESP_LOGW(TAG, "can_send: unknown message '%s'", msg);
        return ESP_ERR_NOT_FOUND;
    }
    dbc_sig_t si = dbc_sig(mi, sig);
    if (si < 0) {
        ESP_LOGW(TAG, "can_send: '%s.%s' not found", msg, sig);
        return ESP_ERR_NOT_FOUND;
    }
    pack_signal(f->data, si, value_to_raw(si, value, scaling));
    frame_finalize(mi, f);
    return can_manager_send(bus, f);
}

esp_err_t can_send(uint8_t bus, const char *msg, const char *sig, double value, bool scaling)
{
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        ESP_LOGW(TAG, "can_send: unknown message '%s'", msg);
        return ESP_ERR_NOT_FOUND;
    }
    const dbc_message_t *m = &g_dbc_messages[mi];
    can_frame_t f = { .id = m->id, .dlc = m->dlc };  // zeroed payload
    return send_frame(bus, msg, sig, value, scaling, &f);
}

esp_err_t can_send_live(uint8_t bus, const char *msg, const char *sig, double value, bool scaling)
{
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        ESP_LOGW(TAG, "can_send_live: unknown message '%s'", msg);
        return ESP_ERR_NOT_FOUND;
    }
    can_frame_t f;
    if (cache_get(mi, &f) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;  // no live frame to read-modify-write yet
    }
    return send_frame(bus, msg, sig, value, scaling, &f);
}

esp_err_t can_get(uint8_t bus, const char *msg, const char *sig, double *out, bool scaling)
{
    (void)bus;  // cache slot is keyed by message (which carries its own bus)
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    dbc_sig_t si = dbc_sig(mi, sig);
    if (si < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    can_frame_t f;
    if (cache_get(mi, &f) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    const dbc_signal_t *s = &g_dbc_signals[si];
    if (s->mux_sel_sig >= 0) {
        int64_t sel = dbc_unpack(f.data, (dbc_sig_t)s->mux_sel_sig);
        if (sel != s->mux_val) {
            return ESP_ERR_INVALID_STATE;  // signal not present in this mux frame
        }
    }
    int64_t raw = dbc_unpack(f.data, si);
    *out = scaling ? dbc_raw_to_phys(si, raw) : (double)raw;
    return ESP_OK;
}

esp_err_t can_frame_init(const char *msg, can_frame_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    const dbc_message_t *m = &g_dbc_messages[mi];
    memset(out, 0, sizeof(*out));
    out->id = m->id;
    out->dlc = m->dlc;
    return ESP_OK;
}

esp_err_t can_frame_live(uint8_t bus, const char *msg, can_frame_t *out)
{
    (void)bus;
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    return cache_get(mi, out);
}

esp_err_t can_frame_send(uint8_t bus, const char *msg, can_frame_t *f)
{
    if (!f) {
        return ESP_ERR_INVALID_ARG;
    }
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    frame_finalize(mi, f);
    return can_manager_send(bus, f);
}

esp_err_t dbc_counter_seed_from_bus(uint8_t bus, const char *msg)
{
    (void)bus;
    dbc_msg_t mi = dbc_msg(msg);
    if (mi < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    const dbc_message_t *m = &g_dbc_messages[mi];
    if (m->counter_sig < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    can_frame_t f;
    if (cache_get(mi, &f) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    s_counter[mi] = (uint32_t)dbc_unpack(f.data, (dbc_sig_t)m->counter_sig);
    return ESP_OK;
}
