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

// Remove all Tesla state. Phase 4 app-channel "reset Tesla key" command.
esp_err_t tesla_storage_erase_all(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    // Erasing the namespace removes priv/pub/vin/addr.
    err = nvs_erase_all(h);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "erased all Tesla state (re-stage + app re-trigger next)");
    }
    return err;
}


