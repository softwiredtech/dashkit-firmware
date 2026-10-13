#include "serial_cmds.h"
#include "dbc.h"
#include "vehicle_control.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VC2_MSG       "UI_vehicleControl2"
#define GLOVEBOX_SIG  "UI_gloveboxRequest"
#define RELEASE_REPS  5
#define BOTH_BUSES    -1

static void print_frame(const char *label, const can_frame_t *f)
{
    printf("%s 0x%03lX [%u]", label, (unsigned long)f->id, f->dlc);
    for (int i = 0; i < f->dlc; i++) {
        printf(" %02X", f->data[i]);
    }
    printf("\n");
}

static esp_err_t send_on(int bus, const char *msg, const char *sig, int value)
{
    esp_err_t err = ESP_OK;
    if (bus != 1) {
        err = can_send_live(0, msg, sig, value, false);
    }
    if (bus != 0 && err == ESP_OK) {
        err = can_send_live(1, msg, sig, value, false);
    }
    return err;
}

static void cmd_rmw(int bus, const char *msg, const char *sig, int value, unsigned hold_ms, unsigned period_ms)
{
    dbc_msg_t mi = dbc_msg(msg);
    dbc_sig_t si = mi < 0 ? DBC_SIG_INVALID : dbc_sig(mi, sig);
    if (si == DBC_SIG_INVALID) {
        printf("unknown message/signal %s.%s\n", msg, sig);
        return;
    }
    can_frame_t f;
    if (can_frame_live(0, msg, &f) != ESP_OK) {
        printf("no live %s frame yet (car asleep?), nothing sent\n", msg);
        return;
    }
    print_frame("live", &f);
    dbc_pack(f.data, si, value);
    print_frame("send", &f);

    TickType_t period = pdMS_TO_TICKS(period_ms) ? pdMS_TO_TICKS(period_ms) : 1;
    int64_t until = esp_timer_get_time() + (int64_t)hold_ms * 1000;
    unsigned frames = 0;
    while (esp_timer_get_time() < until) {
        esp_err_t err = send_on(bus, msg, sig, value);
        if (err != ESP_OK) {
            printf("send failed after %u frames: %s\n", frames, esp_err_to_name(err));
            break;
        }
        frames++;
        vTaskDelay(period);
    }
    for (int i = 0; i < RELEASE_REPS; i++) {
        send_on(bus, msg, sig, 0);
        vTaskDelay(period);
    }
    printf("done: %s.%s=%d bus=%s hold=%ums period=%ums, %u frames per bus, then %d release\n",
           msg, sig, value, bus == BOTH_BUSES ? "0+1" : (bus ? "1" : "0"), hold_ms, period_ms, frames, RELEASE_REPS);
}

static void cmd_live(const char *msg)
{
    can_frame_t f;
    if (can_frame_live(0, msg, &f) != ESP_OK) {
        printf("no live frame for '%s'\n", msg);
        return;
    }
    print_frame(msg, &f);
}

static void help(void)
{
    printf("commands:\n"
           "  gb [bus] [hold_ms] [period_ms]  glovebox pulse, bus 0|1|b (default b 3000 100)\n"
           "  rmw <bus> <msg> <sig> <value> [hold_ms] [period_ms]  set a signal on the live frame, then release to 0\n"
           "  live <message>                  print the cached live frame, e.g. live UI_vehicleControl2\n"
           "  vc <opcode> <value>             submit an app command, opcode in hex, e.g. vc 0f 1\n");
}

static bool parse_bus(const char *b, int *bus)
{
    if (!b || b[0] == 'b') {
        *bus = BOTH_BUSES;
        return true;
    }
    *bus = atoi(b);
    return (*bus == 0 || *bus == 1) && (b[0] == '0' || b[0] == '1');
}

static void handle(char *line)
{
    printf("> %s\n", line);
    char *cmd = strtok(line, " ");
    if (!cmd) {
        return;
    }
    if (strcmp(cmd, "gb") == 0) {
        char *b = strtok(NULL, " ");
        char *h = strtok(NULL, " ");
        char *p = strtok(NULL, " ");
        int bus;
        unsigned hold = h ? strtoul(h, NULL, 10) : 3000;
        unsigned period = p ? strtoul(p, NULL, 10) : 100;
        if (!parse_bus(b, &bus) || hold > 10000 || period == 0) {
            printf("usage: gb [0|1|b] [hold_ms <= 10000] [period_ms > 0]\n");
            return;
        }
        cmd_rmw(bus, VC2_MSG, GLOVEBOX_SIG, 1, hold, period);
    } else if (strcmp(cmd, "rmw") == 0) {
        char *b = strtok(NULL, " ");
        char *msg = strtok(NULL, " ");
        char *sig = strtok(NULL, " ");
        char *v = strtok(NULL, " ");
        char *h = strtok(NULL, " ");
        char *p = strtok(NULL, " ");
        int bus;
        unsigned hold = h ? strtoul(h, NULL, 10) : 1000;
        unsigned period = p ? strtoul(p, NULL, 10) : 100;
        if (!b || !parse_bus(b, &bus) || !msg || !sig || !v || hold > 10000 || period == 0) {
            printf("usage: rmw <0|1|b> <msg> <sig> <value> [hold_ms <= 10000] [period_ms > 0]\n");
            return;
        }
        cmd_rmw(bus, msg, sig, atoi(v), hold, period);
    } else if (strcmp(cmd, "mute") == 0) {
        char *on = strtok(NULL, " ");
        char *b = strtok(NULL, " ");
        vehicle_control_force_chime_mute(on && atoi(on) == 1, b ? atoi(b) : -1);
    } else if (strcmp(cmd, "live") == 0) {
        char *msg = strtok(NULL, " ");
        cmd_live(msg ? msg : VC2_MSG);
    } else if (strcmp(cmd, "vc") == 0) {
        char *op = strtok(NULL, " ");
        char *val = strtok(NULL, " ");
        if (!op || !val) {
            printf("usage: vc <opcode hex> <value>\n");
            return;
        }
        esp_err_t err = vehicle_control_submit((uint8_t)strtoul(op, NULL, 16), (uint16_t)strtoul(val, NULL, 10));
        printf("vc submit: %s\n", esp_err_to_name(err));
    } else {
        help();
    }
}

static void serial_cmds_task(void *arg)
{
    (void)arg;
    char line[96];
    size_t len = 0;
    help();
    while (true) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len) {
                line[len] = '\0';
                handle(line);
                len = 0;
            }
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        }
    }
}

void serial_cmds_start(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);
    xTaskCreate(serial_cmds_task, "serial_cmds", 4096, NULL, 4, NULL);
}
