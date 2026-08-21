#include "tesla_ble_storage.h"

#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>
#include <stdio.h>

static const char *TAG        = "tesla_storage";
static const char *NVS_NS     = "tesla";
static const char *KEY_PRIV   = "priv";
static const char *KEY_PUB    = "pub";
static const char *KEY_VIN    = "vin";
static const char *KEY_ADDR   = "addr";
static const char *KEY_BEACON = "beacon_log";
static const char *KEY_BOOTCNT = "bootcnt";

bool tesla_storage_has_key(void)
{
    nvs_handle_t h;
    size_t len = 0;

    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    esp_err_t e = nvs_get_blob(h, KEY_PRIV, NULL, &len);
    nvs_close(h);
    return e == ESP_OK && len == 32;
}

esp_err_t tesla_storage_load_key(tesla_keypair_t *key)
{
    nvs_handle_t h;
    esp_err_t err;

    if (key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }

    size_t n = sizeof(key->priv);
    err = nvs_get_blob(h, KEY_PRIV, key->priv, &n);
    if (err == ESP_OK && n != sizeof(key->priv)) {
        err = ESP_ERR_NVS_INVALID_LENGTH;
    }
    if (err == ESP_OK) {
        n = sizeof(key->pub);
        err = nvs_get_blob(h, KEY_PUB, key->pub, &n);
        if (err == ESP_OK && n != sizeof(key->pub)) {
            err = ESP_ERR_NVS_INVALID_LENGTH;
        }
    }
    nvs_close(h);
    return err;
}

esp_err_t tesla_storage_load_vin(char *vin, size_t cap)
{
    nvs_handle_t h;
    esp_err_t err;

    if (vin == NULL || cap < 18) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = cap;
    err = nvs_get_str(h, KEY_VIN, vin, &len);
    nvs_close(h);
    return err;
}

esp_err_t tesla_storage_save_key(const tesla_keypair_t *key)
{
    nvs_handle_t h;
    esp_err_t err;

    if (key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, KEY_PRIV, key->priv, sizeof(key->priv));
    if (err == ESP_OK) {
        err = nvs_set_blob(h, KEY_PUB, key->pub, sizeof(key->pub));
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t tesla_storage_save_vin(const char *vin)
{
    nvs_handle_t h;
    esp_err_t err;

    if (vin == NULL || strlen(vin) != 17) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, KEY_VIN, vin);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t tesla_storage_load_car_addr(tesla_car_addr_t *addr)
{
    nvs_handle_t h;
    esp_err_t err;

    if (addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t n = sizeof(*addr);
    err = nvs_get_blob(h, KEY_ADDR, addr, &n);
    if (err == ESP_OK && n != sizeof(*addr)) {
        err = ESP_ERR_NVS_INVALID_LENGTH;
    }
    nvs_close(h);
    return err;
}

esp_err_t tesla_storage_save_car_addr(const tesla_car_addr_t *addr)
{
    nvs_handle_t h;
    esp_err_t err;

    if (addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, KEY_ADDR, addr, sizeof(*addr));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

// Remove all Tesla state (factory reset / re-pair flow) is intentionally not
// wired yet: it will be exposed behind a Phase 4 console command / app
// `pair`-reset write rather than shipping dead today.

// ---- Onboard advertisement-name log ----
//
// Persisted as a single NVS blob (a fixed array, oldest dropped when full).
// Rewritten only when a NEW distinct name is first seen (dedup by name bytes),
// so routine re-advertising does not churn NVS.

#define ADVERT_LOG_MAGIC  0xAD5E0BEEu
#define ADVERT_LOG_NSZ    (sizeof(tesla_advert_log_entry_t) * TESLA_ADVERT_LOG_MAX + 8)

typedef struct {
    uint32_t magic;
    uint16_t count;
    uint16_t max;
    tesla_advert_log_entry_t e[TESLA_ADVERT_LOG_MAX];
} __attribute__((packed)) tesla_advert_log_t;

static const char *fmt_name(uint8_t format)
{
    return format == 2 ? "modern" : (format == 1 ? "legacy" : "-");
}

static void advert_log_load(nvs_handle_t h, tesla_advert_log_t *log)
{
    size_t n = ADVERT_LOG_NSZ;
    memset(log, 0, ADVERT_LOG_NSZ);
    if (nvs_get_blob(h, KEY_BEACON, log, &n) != ESP_OK ||
        log->magic != ADVERT_LOG_MAGIC || log->max != TESLA_ADVERT_LOG_MAX ||
        log->count > TESLA_ADVERT_LOG_MAX) {
        log->magic = ADVERT_LOG_MAGIC;
        log->max   = TESLA_ADVERT_LOG_MAX;
        log->count = 0;
        memset(log->e, 0, sizeof(log->e));
    }
}

void tesla_advert_log_add(const uint8_t *name, size_t name_len, uint8_t matched,
                          uint8_t format, const uint8_t mac[6], int8_t rssi)
{
    // The ~3.1 KB log buffer was previously a stack local; on the default
    // `main` task stack that overran during tesla_advert_log_dump() at boot
    // ("stack overflow in task main" -> reboot loop). Allocate it from the heap
    // per call and free before returning, so no task stack is stressed and
    // there is no shared static state between the boot-path dump and this
    // per-advert add() (which runs on the NimBLE host task).
    tesla_advert_log_t *log = malloc(sizeof(*log));
    if (log == NULL) {
        return;
    }
    nvs_handle_t h;

    if (name == NULL || mac == NULL || name_len == 0) {
        free(log);
        return;
    }
    if (name_len > sizeof(((tesla_advert_log_entry_t *)0)->name)) {
        name_len = sizeof(((tesla_advert_log_entry_t *)0)->name);
    }
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        free(log);
        return;
    }
    advert_log_load(h, log);

    // Dedup by name bytes: if already logged, just refresh rssi/sightings.
    for (uint16_t i = 0; i < log->count; i++) {
        tesla_advert_log_entry_t *en = &log->e[i];
        if (en->name_len == (uint8_t)name_len &&
            memcmp(en->name, name, name_len) == 0) {
            en->rssi  = rssi;
            if (en->count != UINT16_MAX) {
                en->count++;
            }
            // A later sighting that turns out to be a Tesla match upgrades a
            // previously-logged "other" entry.
            if (matched != 0) {
                en->matched = 1;
                en->format  = format;
            }
            nvs_close(h);
            free(log);
            return;
        }
    }

    // New distinct name: drop oldest if full, then append.
    if (log->count >= TESLA_ADVERT_LOG_MAX) {
        memmove(&log->e[0], &log->e[1],
                sizeof(tesla_advert_log_entry_t) * (TESLA_ADVERT_LOG_MAX - 1));
    } else {
        log->count++;
    }
    tesla_advert_log_entry_t *en = &log->e[log->count - 1];
    memset(en, 0, sizeof(*en));
    memcpy(en->name, name, name_len);
    en->name_len = (uint8_t)name_len;
    en->matched  = matched;
    en->format   = format;
    memcpy(en->mac, mac, 6);
    en->rssi     = rssi;
    en->count    = 1;
    en->time_s   = (uint32_t)(esp_timer_get_time() / 1000000);

    esp_err_t e = nvs_set_blob(h, KEY_BEACON, log, ADVERT_LOG_NSZ);
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "advert log persist failed: %s", esp_err_to_name(e));
    }
    nvs_close(h);
    free(log);
}

void tesla_advert_log_dump(void)
{
    // The ~3.1 KB log buffer was previously a stack local; on the default
    // `main` task stack that overran during this function at boot
    // ("stack overflow in task main" -> reboot loop). Allocate from the heap
    // and free before returning.
    tesla_advert_log_t *log = malloc(sizeof(*log));
    if (log == NULL) {
        ESP_LOGW(TAG, "advert log: malloc failed");
        return;
    }
    nvs_handle_t h;

    // READWRITE so the "tesla" namespace is created on first boot (it doesn't
    // exist until Phase 3 writes a key); advert_log_load treats absence as an
    // empty log. This also pre-creates the namespace for later writes.
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGI(TAG, "advert log: (NVS unavailable)");
        free(log);
        return;
    }
    advert_log_load(h, log);
    nvs_close(h);

    ESP_LOGI(TAG, "advert log: %u distinct name(s) seen (oldest->newest)", log->count);
    for (uint16_t i = 0; i < log->count; i++) {
        const tesla_advert_log_entry_t *en = &log->e[i];
        ESP_LOGI(TAG, "  #%u name=\"%.*s\" (format=%s)%s MAC=%02X:%02X:%02X:%02X:%02X:%02X "
                      "rssi=%d x%u t=%us",
                 (unsigned)i + 1, (int)en->name_len, (const char *)en->name,
                 fmt_name(en->format), en->matched ? " [TESLA]" : "",
                 en->mac[5], en->mac[4], en->mac[3], en->mac[2], en->mac[1], en->mac[0],
                 (int)en->rssi, (unsigned)en->count, (unsigned)en->time_s);
    }
    free(log);
}

uint32_t tesla_storage_boot_count(void)
{
    nvs_handle_t h;
    uint32_t n = 0;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_get_u32(h, KEY_BOOTCNT, &n);
        n++;
        esp_err_t e = nvs_set_u32(h, KEY_BOOTCNT, n);
        if (e == ESP_OK) {
            e = nvs_commit(h);
        }
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "boot count persist failed: %s", esp_err_to_name(e));
        }
        nvs_close(h);
    }
    return n;
}
