# DashKit Firmware

CAN-to-BLE bridge firmware for ESP32-S3. Reads CAN bus via MCP2518FD and streams frames to the [DashPilot](https://github.com/softwiredtech/dashpilot) mobile app over Bluetooth Low Energy.

## Two connected devices

Two paired app devices can receive CAN data simultaneously, alongside the
independent Tesla BLE connection. Both encrypted app devices can send vehicle
commands, configure the bridge, and initiate firmware updates independently.
Writing does not require a CAN notification subscription.

A device opts into read-only mode by writing **`4F 01`** (two bytes) to the
encrypted control characteristic **CADA0004-CA00-B1E0-B0D6-C000AA0100A1**.
Send this as soon as the link is encrypted. It immediately disables the sender's
vehicle commands, CAN filter changes, Tesla enrollment commands, and OTA writes.
Read-only devices continue receiving data, even if the other device leaves.

Write **`4F 00`** to enable writes again. The flag applies only to the sender's current connection and
must be sent again after every reconnect. Devices that omit the flag are
read/write, preserving compatibility with existing apps. If both opt into
read-only mode, neither can send commands.

Both devices receive the same CAN stream using a shared CAN filter. Either
writer can replace the filter; the latest accepted filter write wins.
Each link uses its own negotiated MTU. Tesla status notifications go to every
subscriber of the status characteristic, including devices without a CAN subscription.
Keepalive pings and changes to a device's own read-only flag are accepted from
both devices. Vehicle commands from either writer use the existing command queue.
Firmware updates pause CAN streaming on both links. Only one OTA transfer can
run at a time; its initiating device owns data writes, aborts, and status
notifications. Other devices cannot replace or modify that transfer, and their
disconnects do not abort it. The transfer aborts if its owner disconnects or
marks itself read-only.

Advertising continues until both app slots are occupied. To pair the second
device, open the existing pairing window from either writer. If both slots
are occupied, pairing mode disconnects the other device to make room unless
that device owns an OTA transfer.

Run the host regression tests with `bash tools/test/run_ble_clients_test.sh`.

## Prerequisites

- macOS with [Homebrew](https://brew.sh)
- USB cable for flashing

## Setup

### 1. Install dependencies

```bash
brew install cmake ninja dfu-util
```

### 2. Clone and set up ESP-IDF

ESP-IDF v5.4.4 is included as a git submodule:

```bash
git submodule update --init --recursive
./esp-idf/install.sh esp32s3
```

### 3. Activate ESP-IDF environment

Run this in every new terminal session (or add it to your `~/.zshrc`):

```bash
. ./esp-idf/export.sh
```

## Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Press `Ctrl+]` to exit the serial monitor.

## Debug Logging

To enable verbose CAN frame and status logging, add to `sdkconfig.defaults`:

```
CONFIG_DASHKIT_DEBUG_LOG=y
```

Then rebuild:

```bash
idf.py fullclean && idf.py build
```

## Hardware

| Function | GPIO |
|----------|------|
| SPI MOSI | 11 |
| SPI SCLK | 12 |
| SPI MISO | 13 |
| SPI CS | 10 |
| CAN INT | 9 |
| LED Red | 39 |
| LED Green | 40 |
| LED Blue | 41 |
