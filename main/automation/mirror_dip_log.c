// Logs UI_mirrorDipOnReverse (UI_vehicleControl 0x273) on first sight and on
// every change.

#include "automation.h"
#include "dbc.h"

#include "esp_log.h"

static const char *TAG = "mirror_dip";

#define BUS      1
#define MSG_ID   0x273

static uint8_t s_last = 0xFF;

static void mirror_dip_on_frame(automation_t *self, const can_tagged_frame_t *frame)
{
    (void)self;
    if (frame->bus_id != BUS || frame->frame.id != MSG_ID) return;

    double v;
    if (can_get(BUS, "UI_vehicleControl", "UI_mirrorDipOnReverse", &v, false) != ESP_OK) return;

    uint8_t dip = (uint8_t)v;
    if (dip == s_last) return;
    ESP_LOGI(TAG, "UI_mirrorDipOnReverse %u->%u", s_last, dip);
    s_last = dip;
}

automation_t mirror_dip_log_automation = {
    .name     = "mirror_dip_log",
    .on_frame = mirror_dip_on_frame,
};
