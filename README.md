# DashKit Firmware

Firmware for **DashKit**, an ESP32-S3 CAN-to-BLE bridge for Tesla vehicles. DashKit sits on the car's chassis and vehicle CAN buses, streams live data to the [DashPilot](https://github.com/softwiredtech/dashpilot) app over Bluetooth Low Energy, and sends CAN commands for vehicle controls and on-device automations.

## Features

### Live data for DashPilot
- Two CAN buses (chassis + vehicle), each on its own MCP2518FD controller.
- Hardware acceptance filters on each bus pass only the messages DashPilot uses, so no frames get dropped under load.
- An on-device DBC signal layer decodes and caches the latest value of every message it tracks. Automations read signals by name rather than by raw bytes.
- Frames are streamed to the phone over an encrypted, bonded BLE link.

### Vehicle controls
DashPilot sends these commands to DashKit, and DashKit turns them into CAN frames on the vehicle bus:

| Control | Notes |
|---------|-------|
| Frunk / trunk | Remote closure request |
| Mirror fold / unfold | |
| Mirror dip on reverse | Toggles the live setting while in R |
| Charge port open / close | |
| Glovebox | Latch release |
| Rear AC fan | Override toggle |
| AC swing | Sweeps the vents. Choose driver, passenger or both, plus low, medium or full intensity |
| Battery preheat | Makes the car think it's navigating to a Supercharger so it warms the battery |

### Automations
Automations run on DashKit itself, so they keep working when the phone isn't connected. Their settings are saved on the device.

- **Multi-finger gestures**: bind a 3-, 4- or 5-finger tap on the center screen to any control (glovebox, preheat, mirrors, frunk, trunk, charge port, rear fan, mirror dip, AC swing).
- **Auto wiper-off**: turns the wipers fully off when they go from OFF to AUTO. Only acts while lane assist is engaged.
- **Sport kick-down**: switches the pedal map to Sport while the accelerator is past a set threshold in Drive.
- **Keep climate on**: keeps the cabin conditioned for a set time (1–60 min) after the driver leaves the car.

### Tesla BLE key
DashKit can enroll as a Tesla BLE key, with the car provisioned and paired from DashPilot. It keeps a vehicle-command session open and reports presence, lock, sleep, charging and climate state to the app. Disable it with `CONFIG_DASHKIT_TESLA_BLE`.

### Device management
- **OTA updates** from DashPilot. If a new image crash-loops, the device rolls back to the previous one.
- **Secure pairing**: only an already-paired phone can open a pairing window. Up to 3 phones can be bonded, with one active connection at a time.
- **Factory reset**: hold 5 fingers on the center screen to clear all bonds and reboot.
- **BTHome buttons** (experimental, `CONFIG_DASHKIT_BTHOME`): use a Shelly BLU button as a physical trigger, e.g. to open the glovebox.

## Using DashKit with DashPilot

1. Plug in DashKit. The LED turns blue while it boots and green when it's ready (red means a CAN or BLE init failure).
2. In DashPilot, scan for and pair with **DashKit**.
3. DashPilot displays live vehicle data. From its settings you can trigger controls, bind gestures, turn automations on or off, and install firmware updates.
4. Optional: add DashKit as a Tesla BLE key from DashPilot. Tap your key card on the console when prompted, then confirm on the touchscreen (the LED turns cyan while it waits).

## Community Projects

Projects people have built on top of DashKit:

- **[Tesla LilyGO Head-Up Display](https://github.com/philoh-dev/tesla-lilygo-head-up-display)**: a head-up display on a LILYGO T-Display-S3 AMOLED that reads live data from DashKit over BLE. It has a compact HUD layout (speed and essentials) and a "road scene" layout with lane markings, turn signals and blind-spot warnings.

Built something with DashKit? Open a PR to add it here.

## Building

### Prerequisites

- macOS with [Homebrew](https://brew.sh)
- USB cable for flashing

### Setup

```bash
brew install cmake ninja dfu-util
git submodule update --init --recursive
./esp-idf/install.sh esp32s3
```

Activate ESP-IDF in every new terminal session (or add it to your `~/.zshrc`):

```bash
. ./esp-idf/export.sh
```

### Build & flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Press `Ctrl+]` to exit the serial monitor.

### Configuration

Options under `idf.py menuconfig` → **DashKit**:

| Option | Default | |
|--------|---------|---|
| `CONFIG_DASHKIT_TESLA_BLE` | y | Tesla BLE vehicle-command client |
| `CONFIG_DASHKIT_BTHOME` | n | BTHome / Shelly BLU button listener |

For verbose CAN frame and status logging, uncomment `CONFIG_DASHKIT_DEBUG_LOG=y` in `sdkconfig.defaults`.

Run `idf.py fullclean && idf.py build` after changing options.

## Project Layout

| Path | Contents |
|------|----------|
| `main/can/` | MCP251xFD driver, CAN manager, HW filters, DBC signal layer |
| `main/ble/` | NimBLE GATT server, OTA service, app channel |
| `main/controls/` | Vehicle control commands, battery preheat |
| `main/automation/` | One file per automation, plus the registry |
| `main/tesla/` | Tesla BLE key enrollment and vehicle-command client |
| `main/bthome/` | BTHome button listener |
| `components/tesla-protocol/` | Tesla vehicle-command crypto, session, protobufs |
| `dbc/` | Vehicle DBC and the `used_messages.txt` manifest |
| `tools/` | DBC/proto codegen, host tests, OTA publishing |

The DBC tables (`main/can/dbc/dbc_generated.{c,h}`) are generated from `dbc/used_messages.txt` and committed. If `tools/.venv` exists, `idf.py build` regenerates them automatically:

```bash
python3 -m venv tools/.venv && tools/.venv/bin/pip install -r tools/requirements.txt
```

## Hardware

| Function | GPIO |
|----------|------|
| SPI MOSI | 11 |
| SPI SCLK | 12 |
| SPI MISO | 13 |
| CAN0 CS / INT | 10 / 9 |
| CAN1 CS / INT | 7 / 8 |
| LED Red / Green / Blue | 39 / 40 / 41 |

Both CAN controllers are MCP2518FD with 40 MHz crystals and share the SPI2 bus at 10 MHz.

## License

MIT. See [LICENSE](LICENSE).
