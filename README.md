# esphome-vl53l1x

An [ESPHome](https://esphome.io) external component for the **ST VL53L1X** time-of-flight
distance sensor — the longer-range successor to the VL53L0X, measuring up to about 4 m.

ESPHome core ships a `vl53l0x` platform but has no VL53L1X support, so this fills the gap.

```yaml
external_components:
  - source: github://damienmorrissey/esphome-vl53l1x
    components: [vl53l1x]

i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: vl53l1x
    name: "Distance"
    update_interval: 1s
```

## Status and relationship to ESPHome core

There is an active pull request to add VL53L1X support to ESPHome core
([esphome/esphome#13605](https://github.com/esphome/esphome/pull/13605)). **If you want the
most feature-complete option, use that** — it supports region-of-interest selection,
calibration offsets and interrupt-driven measurement, and it can be consumed directly today:

```yaml
external_components:
  - source: github://pr#13605
    components: [vl53l1x]
```

This component is deliberately smaller in scope. It exists because it adds one thing that
PR is missing: **automatic recovery when the sensor stops responding** (see below). If the
core PR lands, this repo's value narrows to that recovery behaviour, and the recovery logic
is simple enough to contribute upstream instead.

## Features

- Distance modes: `short` (~1.3 m), `medium` (~3 m), `long` (~4 m)
- Configurable timing budget (20–1000 ms)
- Optional XSHUT/enable pin control, required for multiple sensors on one bus
- Invalid measurements (no target, excessive ambient light, hardware fault) publish as
  unknown rather than as a plausible-looking wrong distance
- **Automatic re-initialisation** if the sensor stops responding

### Automatic recovery

Most ToF drivers initialise once at boot and never retry. If the sensor is not powered yet
at boot, or is disconnected, swapped or power-cycled while running, they stay dead until the
whole device reboots.

This component instead treats initialisation as something it retries whenever needed. After
three consecutive failed reads it marks the sensor as gone, publishes unknown, and attempts a
full reset-and-reinitialise on each subsequent update until the sensor comes back. Hot-swapping
the physical sensor recovers within a couple of update intervals, with no reboot.

## Configuration

| Option | Type | Default | Description |
| --- | --- | --- | --- |
| `address` | i2c address | `0x29` | I²C address. Any value other than `0x29` requires `enable_pin`. |
| `distance_mode` | `short` / `medium` / `long` | `long` | Longer modes see further but are more sensitive to ambient light. |
| `timing_budget` | time (20 ms–1 s) | `50ms` | Time allowed per measurement. Longer is more accurate; must be ≤ `update_interval`. |
| `enable_pin` | pin | — | The sensor's XSHUT pin. Used to reset the sensor and to re-address it. |
| `update_interval` | time | `60s` | Standard polling component option. |

Plus all the usual [sensor](https://esphome.io/components/sensor/) options. The published
value is in **metres**, matching ESPHome's `vl53l0x`.

## Wiring

| Sensor pin | Connect to |
| --- | --- |
| `VCC` / `VIN` | 3.3 V |
| `GND` | GND |
| `SDA` | your configured `i2c` SDA pin |
| `SCL` | your configured `i2c` SCL pin |
| `XSHUT` | a GPIO, set as `enable_pin` (see below) |
| `GPIO1` / `INT` | not used, leave unconnected |

> **Check whether your breakout needs XSHUT wired.** Bare breakouts such as the CJMCU-531
> bring XSHUT out with no pull-up on the board. On those the chip sits in hardware shutdown
> and never appears on an I²C scan at all until something drives XSHUT high — so `enable_pin`
> is effectively mandatory, not optional. Breakouts with an onboard pull-up (Pololu, Adafruit,
> SparkFun) work without it.

> **Voltage.** The VL53L1X is a 3.3 V part with an absolute maximum around 3.6 V, and many
> cheap breakouts have no onboard regulator, so VIN goes more or less straight to the chip.
> Do not feed one 5 V.

### Multiple sensors on one bus

Every VL53L1X powers up at `0x29`, so more than one on a bus needs each to be held in reset
via `XSHUT` and brought up individually. This component does not implement automatic
re-addressing yet; if you need multiple sensors on a single bus today, use
[PR #13605](https://github.com/esphome/esphome/pull/13605), which does.

## Example

See [`example.yaml`](example.yaml).

## Notes on the implementation

The register-level protocol is ported from
[Pololu's VL53L1X Arduino library](https://github.com/pololu/vl53l1x-arduino), which is itself
derived from ST's VL53L1X API (STSW-IMG007).

All of the timing and fixed-point arithmetic — macro period calculation, timeout encode/decode,
the DSS update, the range-status mapping — is transliterated **verbatim** from that driver. Only
the I²C transport was replaced, so the component uses ESPHome's `i2c::I2CDevice` and works under
ESP-IDF without the Arduino `Wire` library.

That was a deliberate choice. This arithmetic is full of easy-to-get-subtly-wrong fixed-point
conversions, and a rewrite risks bugs that produce plausible-looking but incorrect distances.
Keeping it byte-for-byte identical to a driver with years of field use avoids that entire class
of error.

> **If you are writing your own ESPHome component that wraps an Arduino library using `Wire`
> directly:** on ESP32/ESP-IDF, ESPHome's `i2c:` component and Arduino's global `Wire` object are
> two independent drivers that cannot both claim the same I²C peripheral. If both are configured
> on the same pins, `Wire.begin()` silently returns `false` and every subsequent transaction fails
> — while ESPHome's own bus scan keeps working, which makes it look like a dead sensor rather than
> a software conflict. Use `i2c::I2CDevice`.

## Licence

BSD-3-Clause, inherited from the ST API and Pololu library this is derived from. See
[LICENSE](LICENSE).
