# Fake Tesla beacon — Phase 1 scan verification helper

A standalone ESP-IDF project that broadcasts a Tesla-format advertisement name
from a spare ESP32-S3 board. The Phase 1 firmware observes BLE advertisements
and logs the name + MAC of anything matching Tesla's two documented formats:

- Legacy: `S` + first-8-hex of `SHA1(VIN)` + `C`/`R`/`D`/`P`
- Modern: `Tesla ` + last 6 characters of the VIN

A real Tesla is the production beacon (VCSEC is always powered, so the car
need not be awake), but you verify the observer loop with this board if no car
is within radio range.

## Pick a name for your "VIN"

The advertisement name is derived from the VIN you want to emulate:

```bash
bash tools/test/run_tesla_advert_name_test.sh 5YJ30123456789ABC
```

The last lines print the two exact names a fake beacon must broadcast (and the
host test verifies the matcher accepts them):

```
== fake-beacon names for this VIN ==
  legacy: S<8-hex>C
  modern: Tesla 789ABC
```

## Set the advertised name

```bash
idf.py menuconfig        # -> Fake Tesla Beacon -> Advertised local name
```

or just edit `main/Kconfig.projbuild`. Paste either the legacy or the modern
name (modern reads more obviously in a BLE scanner).

## Build + run (from this directory)

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <COMx> flash monitor
```

The beacon logs `advertising as "Tesla 789ABC"` and stays on forever.

## Verify the firmware sees it

Flash the Phase 1 firmware on the DashKit board, then bring the beacon within
range. The DashKit serial log shows, once per discovered device:

```
I (...) tesla_ble: Tesla vehicle found: name="Tesla 789ABC" (format=modern), MAC=XX:XX:XX:XX:XX:XX
```

## Phone-phone alternative (no second board)

nRF Connect for Mobile (Nordic) has an **Advertiser** tool:

1. Open **Advertiser** and create a new advertisement.
2. Add an AD type **Complete Local Name** and paste the name from above.
3. Also set the flags AD if prompted. Start advertising; the DashKit observer
   will log the same line. Broadcast time is limited (~3 minutes), fine for
   verifying the observer.

On macOS you can also advertise a custom name with a short Swift/`CoreBluetooth`
snippet or an app that exposes the local-name AD field.

> Note: this beacon advertises the modern format by default — the legacy format
> (starting with `S`) is what a pre-2023 car broadcasts. To verify the legacy
> matcher too, set `FAKE_TESLA_NAME` to the `legacy:` string and repeat.