/*
 * Fake Tesla BLE beacon (Phase 1 verification helper).
 *
 * A standalone ESP-IDF project that advertises a Tesla-format local name from
 * a spare ESP32-S3 board, so the firmware's observer can be verified without a
 * real car (a Tesla need not be awake to advertise, but one must be nearby —
 * this project substitutes for that beacon).
 *
 * Set the advertised name with menuconfig (FAKE_TESLA_NAME). Generate the
 * correct strings for your VIN via tools/test/run_tesla_advert_name_test.sh.
 */

#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "beacon";

static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* BLE_HS_FOREVER should keep advertising, so a completion here means
         * the controller stopped us (e.g. after a reset). Restart to stay
         * self-healing. */
        ESP_LOGW(TAG, "advertising completed unexpectedly; restarting");
        start_advertising();
        break;
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 0x20,   /* 20 ms */
        .itvl_max = 0x40,   /* 40 ms */
    };
    struct ble_hs_adv_fields fields = {0};
    uint8_t own_addr_type;

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)CONFIG_FAKE_TESLA_NAME;
    fields.name_len = strlen(CONFIG_FAKE_TESLA_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields failed: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);   /* best available address */
    if (rc != 0) {
        ESP_LOGE(TAG, "infer own address failed: rc=%d", rc);
        return;
    }
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start failed: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising as \"%s\"", CONFIG_FAKE_TESLA_NAME);
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();               /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(CONFIG_FAKE_TESLA_NAME);

    nimble_port_freertos_init(nimble_host_task);
}