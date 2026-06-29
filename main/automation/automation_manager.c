#include "automation_manager.h"
#include "automation.h"
#include "can_manager.h"
#include "dbc.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

static const char *TAG = "auto_mgr";

// The central registry (the single edit point for adding an automation).
extern automation_t *const g_automations[];
extern const int g_automation_count;

#define AM_MAX_AUTOMATIONS  12
#define AM_MAX_SUBS          6

// Resolved subscribed message ids per automation, for fast on_frame gating.
typedef struct {
    uint32_t ids[AM_MAX_SUBS];
    int      count;
} am_subs_t;
static am_subs_t s_subs[AM_MAX_AUTOMATIONS];

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

    int total_watched = 0;
    for (int a = 0; a < g_automation_count; a++) {
        automation_t *au = g_automations[a];
        s_subs[a].count = 0;

        if (au->init) {
            au->init(au);
        }

        if (au->subscribe) {
            for (const char *const *p = au->subscribe; *p != NULL; p++) {
                dbc_msg_t mi = dbc_msg(*p);
                if (mi < 0) {
                    ESP_LOGE(TAG, "%s subscribes to unknown message '%s'",
                             au->name, *p);
                    continue;
                }
                if (s_subs[a].count >= AM_MAX_SUBS) {
                    ESP_LOGE(TAG, "%s has too many subscriptions", au->name);
                    break;
                }
                uint8_t bus = g_dbc_messages[mi].default_bus;
                dbc_watch(bus, *p);
                s_subs[a].ids[s_subs[a].count++] = g_dbc_messages[mi].id;
                total_watched++;
            }
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

        ESP_LOGI(TAG, "automation '%s' (%d subs, tick %" PRIu32 "ms)",
                 au->name, s_subs[a].count, au->tick_period_ms);
    }

    ESP_LOGI(TAG, "initialized %d automations (%d watched frames)",
             g_automation_count, total_watched);
    return ESP_OK;
}

void automation_manager_on_frame(const can_tagged_frame_t *frame)
{
    for (int a = 0; a < g_automation_count; a++) {
        automation_t *au = g_automations[a];
        if (!au->on_frame) {
            continue;
        }
        for (int i = 0; i < s_subs[a].count; i++) {
            if (s_subs[a].ids[i] == frame->frame.id) {
                au->on_frame(au, frame);
                break;
            }
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
