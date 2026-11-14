# DashKit Firmware

[![Discord](https://img.shields.io/badge/Discord-Join%20the%20community-5865F2?logo=discord&logoColor=white)](https://discord.gg/naTZgwVqvA)
![Version](https://img.shields.io/badge/version-0.3.1-brightgreen)
![ESP32-S3](https://img.shields.io/badge/ESP32--S3-ESP--IDF%205.4-E7352C?logo=espressif&logoColor=white)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue)](LICENSE)

Firmware for **DashKit**, an ESP32-S3 controller for Tesla vehicles. DashKit plugs into the car's CAN buses so you can **control the car**, run **automations** on the device, and feed **live vehicle data** to the [DashPilot](https://github.com/softwiredtech/dashpilot) app.

- 🎛️ **[Vehicle controls](#vehicle-controls)**: open the frunk or glovebox, fold the mirrors, preheat the battery and more, from your phone or a gesture on the center screen
- ⚡ **[Automations](#automations)**: behaviours that run on DashKit itself, even with no phone connected
- 📊 **[Live data for DashPilot](#live-data-for-dashpilot)**: real-time CAN data streamed to the app over Bluetooth

## Vehicle controls

DashPilot sends these commands to DashKit, and DashKit turns them into CAN frames on the vehicle bus. You can also bind any of them to a [multi-finger gesture](#automations).

- 🚗 Frunk / trunk
- 🪞 Mirror fold / unfold
- ↩️ Mirror dip on reverse
- 🔌 Charge port open / close
- 🧤 Glovebox
- 🌬️ Rear AC fan
- 🔄 AC swing
- 🔋 Battery preheat

## Automations

> [!TIP]
> Automations run on DashKit itself, so they keep working when the phone isn't connected. Their settings are saved on the device.

- ✋ **Multi-finger gestures**: bind a 3-, 4- or 5-finger tap on the center screen to any [vehicle control](#vehicle-controls).
- 🌧️ **Auto wiper-off**: turns the wipers fully off when they go from OFF to AUTO. Only acts while lane assist is engaged.
- 🏁 **Sport kick-down**: switches the pedal map to Sport while the accelerator is past a set threshold in Drive.
- ❄️ **Keep climate on**: keeps the cabin conditioned for a set time (1–60 min) after the driver leaves the car.

## Live data for DashPilot

- Two CAN buses (chassis + vehicle), each on its own MCP2518FD controller.
- Hardware acceptance filters on each bus pass only the messages DashPilot uses, so no frames get dropped under load.
- An on-device DBC signal layer decodes and caches the latest value of every message it tracks. Automations read signals by name rather than by raw bytes.
- Frames are streamed to the phone over an encrypted, bonded BLE link.

## Interior ambient lighting

Vehicle CAN (bus 1) ID **0x679**, `UI_ambientLightingCtrls`, is forwarded
unchanged over the CAN BLE stream. This uses one additional hardware filter
slot (28 of 32 on the vehicle controller). The message is filter-only; the app
decodes it. Apps installing a nonempty CAN filter must include bus 1 / 0x679
alongside their other entries.

The supplied newer Tesla database layout uses zero-based byte offsets:

| Field | Location |
| --- | --- |
| Enable/effect, raw | Byte 0 |
| Red, green, blue | Bytes 1, 2, 3 |
| Brightness, 0-100% | Byte 4, lower seven bits |
| Target sections, raw | Bits 43-48 |

The section mask is `((data[5] >> 3) | (data[6] << 5)) & 0x3f`.
The DBC assumes a nominal eight-byte payload; forwarding preserves the actual
received length and all bytes. RGB forwarding to a Rive dashboard has been
confirmed working on a user's vehicle. Effect subdivisions, section labels,
and compatibility with other vehicle/software variants remain unverified.

## More features

- 🔑 **Tesla BLE key**: DashKit can enroll as a Tesla BLE key, with the car provisioned and paired from DashPilot. It reports presence, lock, sleep, charging and climate state to the app.
- ⬆️ **OTA updates** from DashPilot. If a new image crash-loops, the device rolls back to the previous one.
- 🔒 **Secure pairing**: only an already-paired phone can open a pairing window. Up to 3 phones can be bonded, with one active connection at a time.
- 🧹 **Factory reset**: hold 5 fingers on the center screen to clear all bonds and reboot.
- 🔘 **BTHome buttons** (experimental): use a Shelly BLU button as a physical trigger, e.g. to open the glovebox.

## Using DashKit with DashPilot

1. Plug in DashKit. The LED turns blue while it boots and green when it's ready (red means a CAN or BLE init failure).
2. In DashPilot, scan for and pair with **DashKit**.
3. DashPilot displays live vehicle data. From its settings you can trigger controls, bind gestures, turn automations on or off, and install firmware updates.
4. Optional: add DashKit as a Tesla BLE key from DashPilot. Tap your key card on the console when prompted, then confirm on the touchscreen (the LED turns cyan while it waits).

## Community Projects

Projects people have built on top of DashKit:

- 🕶️ **[Tesla LilyGO Head-Up Display](https://github.com/philoh-dev/tesla-lilygo-head-up-display)**: a head-up display on a LILYGO T-Display-S3 AMOLED that reads live data from DashKit over BLE. It has a compact HUD layout (speed and essentials) and a "road scene" layout with lane markings, turn signals and blind-spot warnings.

> [!NOTE]
> Built something with DashKit? Open a PR to add it here, or share it on [Discord](https://discord.gg/naTZgwVqvA).

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
| `CONFIG_DASHKIT_BTHOME_ALLOWED` | "" | Comma-separated button addresses allowed to trigger controls (from the "BTHome device … seen" log line) |

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
