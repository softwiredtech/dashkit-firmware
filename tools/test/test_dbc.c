// Host round-trip unit test for the DBC engine (main/can/dbc/dbc.c).
// Compiled with plain gcc (no ESP-IDF) using the shim headers in shim/.
//
//   gcc -I tools/test/shim -I main/can -I main/can/dbc \
//       tools/test/test_dbc.c main/can/dbc/dbc.c main/can/dbc/dbc_generated.c \
//       -o /tmp/test_dbc && /tmp/test_dbc
//
// Verifies pack/unpack, mux selector, signed extension, Tesla checksum+counter,
// and read-modify-write preservation against the values the old hand-rolled
// per-feature code produced.

#include "dbc.h"
#include "can_manager.h"

#include <stdio.h>
#include <string.h>

// ---- can_manager stub (records the last sent frame; serves a settable live) ----
static can_frame_t s_last_sent;
static int         s_sent_count;
static can_frame_t s_live;
static bool        s_live_valid;

esp_err_t can_manager_send(uint8_t bus, const can_frame_t *frame)
{
    (void)bus;
    s_last_sent = *frame;
    s_sent_count++;
    return ESP_OK;
}

esp_err_t can_manager_get_frame(uint8_t bus, uint32_t id, can_frame_t *out)
{
    (void)bus;
    if (!s_live_valid || s_live.id != id) return ESP_ERR_NOT_FOUND;
    *out = s_live;
    return ESP_OK;
}

esp_err_t can_manager_watch_frame(uint8_t bus, uint32_t id)
{
    (void)bus; (void)id;
    return ESP_OK;
}

static void set_live(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    memset(&s_live, 0, sizeof(s_live));
    s_live.id = id;
    s_live.dlc = dlc;
    memcpy(s_live.data, data, dlc);
    s_live_valid = true;
}

// ---- reference implementations copied verbatim from the OLD code ----
static uint8_t ref_tesla_checksum(uint32_t addr, const uint8_t *dat, uint8_t len, uint8_t cb)
{
    uint8_t sum = (addr & 0xFF) + ((addr >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; i++) if (i != cb) sum += dat[i];
    return sum;
}
static void ref_build_left_stalk(uint8_t out[3], uint8_t counter, uint8_t wash_wipe)
{
    out[1] = (counter & 0x0F) | ((wash_wipe & 0x03) << 6);
    out[2] = 0;
    out[0] = ref_tesla_checksum(0x249, out, 3, 0);
}

// ---- tiny test harness ----
static int g_fail;
#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_fail++; } \
    else { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

static dbc_sig_t sig(const char *m, const char *s)
{
    return dbc_sig(dbc_msg(m), s);
}

int main(void)
{
    dbc_init();

    // 1. Charge port open = bit0 of byte0 on a zeroed UI_chargeRequest.
    {
        uint8_t d[8] = {0};
        dbc_pack(d, sig("UI_chargeRequest", "UI_openChargePortDoorRequest"), 1);
        CHECK(d[0] == 0x01, "charge open -> data[0]=0x%02X (want 0x01)", d[0]);
    }

    // 2. Charge port close = bit1.
    {
        uint8_t d[8] = {0};
        dbc_pack(d, sig("UI_chargeRequest", "UI_closeChargePortDoorRequest"), 1);
        CHECK(d[0] == 0x02, "charge close -> data[0]=0x%02X (want 0x02)", d[0]);
    }

    // 3. RMW preserves UI_chargeEnableRequest (bit2) while setting open (bit0).
    {
        uint8_t live[4] = { 0x04, 0x00, 0x00, 0x00 };  // bit2 set
        set_live(0x333, 4, live);
        esp_err_t e = can_send_live(1, "UI_chargeRequest", "UI_openChargePortDoorRequest", 1, false);
        CHECK(e == ESP_OK && s_last_sent.data[0] == 0x05,
              "RMW open keeps enable bit2 -> data[0]=0x%02X (want 0x05)",
              s_last_sent.data[0]);
    }

    // 4. DAS_wiperSpeed nibble (byte0 high nibble).
    {
        uint8_t d[8] = {0}; d[0] = 0xF0;
        int64_t v = dbc_unpack(d, sig("DAS_bodyControls", "DAS_wiperSpeed"));
        CHECK(v == 15, "wiperSpeed 0xF0 -> %d (want 15)", (int)v);
        d[0] = 0x00;
        v = dbc_unpack(d, sig("DAS_bodyControls", "DAS_wiperSpeed"));
        CHECK(v == 0, "wiperSpeed 0x00 -> %d (want 0)", (int)v);
    }

    // 5. UI_status2 active touch points (byte3).
    {
        uint8_t d[8] = {0}; d[3] = 4;
        int64_t v = dbc_unpack(d, sig("UI_status2", "UI_activeTouchPoints"));
        CHECK(v == 4, "touchPoints byte3=4 -> %d (want 4)", (int)v);
    }

    // 6. Signed scroll -1 (6-bit) with auto mux selector, via one-shot send.
    {
        s_sent_count = 0;
        can_send(1, "VCLEFT_switchStatus", "VCLEFT_swcLeftScrollTicks", -1, false);
        bool sel_ok  = (s_last_sent.data[0] & 0x03) == 0x01;  // mux index 1
        bool bits_ok = s_last_sent.data[2] == 0x3F;           // -1 in 6 bits
        int64_t back = dbc_unpack(s_last_sent.data,
                                  sig("VCLEFT_switchStatus", "VCLEFT_swcLeftScrollTicks"));
        CHECK(sel_ok && bits_ok && back == -1,
              "scroll -1: sel=0x%02X data[2]=0x%02X unpack=%d (want sel.bit=1, 0x3F, -1)",
              s_last_sent.data[0], s_last_sent.data[2], (int)back);
    }

    // 7. Scaled signal raw round-trip (UI_chargeTerminationPct, scale 0.1, 10-bit).
    {
        uint8_t d[8] = {0};
        dbc_sig_t s = sig("UI_chargeRequest", "UI_chargeTerminationPct");
        dbc_pack(d, s, 800);  // raw -> 80.0%
        int64_t v = dbc_unpack(d, s);
        CHECK(v == 800, "chargeTerminationPct raw 800 round-trip -> %d", (int)v);
    }

    // 7b. PHYSICAL round-trip through scale 0.1: set 80.0% -> raw 800 on the
    // wire; can_get reads it back as 80.0 (physical). Exercises the double path.
    {
        dbc_sig_t s = sig("UI_chargeRequest", "UI_chargeTerminationPct");
        CHECK(dbc_phys_to_raw(s, 80.0) == 800,
              "phys 80.0%% -> raw %d (want 800)", (int)dbc_phys_to_raw(s, 80.0));
        uint8_t d[8] = {0};
        dbc_pack(d, s, dbc_phys_to_raw(s, 80.0));
        set_live(0x333, 4, d);
        double phys = -1.0;
        esp_err_t e = can_get(1, "UI_chargeRequest", "UI_chargeTerminationPct", &phys, true);
        CHECK(e == ESP_OK && phys > 79.99 && phys < 80.01,
              "can_get physical -> %.2f (want 80.00)", phys);
    }

    // 8. SCCM checksum + counter match the old build_left_stalk reference.
    {
        // No live frame: counter starts at 0, first send emits 1.
        s_live_valid = false;
        can_send(1, "SCCM_leftStalk", "SCCM_washWipeButtonStatus", 1, false);
        uint8_t ref[3];
        ref_build_left_stalk(ref, 1, 1);  // counter=1, wash_wipe=1
        bool match = s_last_sent.dlc == 3 &&
                     s_last_sent.data[0] == ref[0] &&
                     s_last_sent.data[1] == ref[1] &&
                     s_last_sent.data[2] == ref[2];
        CHECK(match, "SCCM frame %02X %02X %02X vs ref %02X %02X %02X",
              s_last_sent.data[0], s_last_sent.data[1], s_last_sent.data[2],
              ref[0], ref[1], ref[2]);
    }

    // 9. Counter seed-from-bus: cached counter=9 -> next send emits 10.
    {
        uint8_t live[3] = {0};
        dbc_pack(live, sig("SCCM_leftStalk", "SCCM_leftStalkCounter"), 9);
        set_live(0x249, 3, live);
        dbc_counter_seed_from_bus(1, "SCCM_leftStalk");
        can_send(1, "SCCM_leftStalk", "SCCM_washWipeButtonStatus", 1, false);
        int64_t c = dbc_unpack(s_last_sent.data, sig("SCCM_leftStalk", "SCCM_leftStalkCounter"));
        CHECK(c == 10, "seed 9 then send -> counter %d (want 10)", (int)c);
    }

    printf("\n%s\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED");
    return g_fail ? 1 : 0;
}
