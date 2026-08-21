#include "automation_manager.h"
#include "automation.h"
#include "can_manager.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

static const char *TAG = "auto_mgr";

// The central registry: the one place edited to add or remove an automation.
// Each automation lives in its own .c file and defines a single automation_t;
// list it here (and in main/CMakeLists.txt SRCS) to activate it.
extern automation_t wiper_off_automation;
extern automation_t multi_finger_automation;
extern automation_t battery_preheat_automation;

static automation_t *const g_automations[] = {
    &wiper_off_automation,
    &multi_finger_automation,
    &battery_preheat_automation,
};
static const int g_automation_count =
    (int)(sizeof(g_automations) / sizeof(g_automations[0]));

#define AM_MAX_AUTOMATIONS  12

static void tick_cb(void *arg)
{
    automation_t *au = (automation_t *)arg;
    if (au->on_tick) {
        au->on_tick(au);
    }
}

esp_err_t automation_manager_init(void)
{
    if (g_automation_count > AM_MAX_AUTOMATIONS) {
        ESP_LOGE(TAG, "too many automations (%d > %d)",
                 g_automation_count, AM_MAX_AUTOMATIONS);
        return ESP_ERR_NO_MEM;
    }

    for (int a = 0; a < g_automation_count; a++) {
        automation_t *au = g_automations[a];

        if (au->init) {
            au->init(au);
        }

        if (au->tick_period_ms > 0 && au->on_tick) {
            const esp_timer_create_args_t args = {
                .callback = tick_cb,
                .arg = au,
                .name = au->name,
                .dispatch_method = ESP_TIMER_TASK,
            };
            esp_timer_handle_t timer;
            esp_err_t err = esp_timer_create(&args, &timer);
            if (err == ESP_OK) {
                esp_timer_start_periodic(timer, (uint64_t)au->tick_period_ms * 1000);
            } else {
                ESP_LOGE(TAG, "%s: esp_timer_create failed: %s",
                         au->name, esp_err_to_name(err));
            }
        }

        ESP_LOGI(TAG, "automation '%s' (tick %" PRIu32 "ms)",
                 au->name, au->tick_period_ms);
    }

    ESP_LOGI(TAG, "initialized %d automations", g_automation_count);
    return ESP_OK;
}

void automation_manager_on_frame(const can_tagged_frame_t *frame)
{
    for (int a = 0; a < g_automation_count; a++) {
        automation_t *au = g_automations[a];
        if (au->on_frame) {
            au->on_frame(au, frame);
        }
    }
}

void automation_manager_config(uint8_t opcode, uint16_t value)
{
    for (int a = 0; a < g_automation_count; a++) {
        automation_t *au = g_automations[a];
        if (au->on_config) {
            au->on_config(au, opcode, value);
        }
    }
}
