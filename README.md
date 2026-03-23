# Sonoff MS01 Soil Moisture Sensor — ESPHome External Component

Reads the **Sonoff MS01** capacitive soil moisture sensor using the ESP32's
hardware **RMT peripheral** for the sensor's DHT22-style one-wire protocol.

No Arduino library required; no busy-wait blocking; fully interrupt driven; plays
nicely with ESPHome; no interference with the FreeRTOS scheduler.

---

## Hardware requirements

| Item | Detail |
|---|---|
| **MCU** | Any ESP32 variant: original, S2, S3, C3, C6 … |
| **Framework** | **ESP-IDF** (Arduino is not supported — RMT v5 API required) |
| **GPIO** | One free internal GPIO per sensor |
| **RMT channel** | One RMT RX channel, borrowed for ~5 ms per reading then released |

---

## Wiring

The MS01 has a 3-pin JST-XH connector:

```
MS01  red   →  ESP32 3.3 V
MS01  black →  ESP32 GND
MS01  white →  ESP32 GPIO (any free internal pin, e.g. GPIO14)
```

A **4.7kΩ to 10kΩ pull-up resistor** from DATA to 3.3 V is recommended for cable runs
longer than ~20 cm.  The sensor has an internal weak pull-up but it can be overwhelmed
by cable capacitance.

I usually use an 'adaptor/converter' type module normally used for DS18b20 sensors.
These have the pullup resistor as well as a little power decoupling capacitor
(one example at: https://www.aliexpress.com/item/1005002989076212.html).

---

## Directory structure

```
sonoff_ms01_esphome/
├── components/
│   └── sonoff_ms01/
│       ├── __init__.py        # ESPHome component namespace declaration
│       ├── sensor.py          # YAML schema + Python code generation
│       ├── sonoff_ms01.h      # C++ class header
│       └── sonoff_ms01.cpp    # C++ implementation
└── example.yaml               # Ready-to-flash sample configuration
```

---

## Quick start

1. **Copy** the contents of example.yaml from the GitHub repo to your local ESPHome config directory

2. **Edit** the file.  Read all the comments and then at a minimum:
   - Set `pin:` to the GPIO you wired the DATA line to.
   - Fill in your Wi-Fi credentials (or use a `secrets.yaml`).

3. **Install** as per usual.  ESPHome will retrieve what it needs from the repo before building the firmware.

4. The device will appear in Home Assistant automatically via the native API.

---

## Using with multiple sensors

The component supports `MULTI_CONF`, so you can attach several MS01s to
different GPIO pins:

```yaml
sensor:
  - platform: sonoff_ms01
    pin: GPIO14
    voltage:
      name: "Pot 1"
      ...

  - platform: sonoff_ms01
    pin: GPIO27
    voltage:
      name: "Pot 2"
      ...
```

Each sensor instance borrows one RMT RX channel during its ~5 ms read window.
On the original ESP32 there are 8 RMT channels total; on C3/C6 there are 4.
Concurrent reads are not attempted (each sensor polls independently via its own
PollingComponent timer), so channel contention is unlikely at typical 60-second
(or more) intervals.

---

## Calibration & Moisture Percentage Reading

The **MS01** simply gives a "voltage" reading.  Calibration and conversion from
the **MS01** reading to a moisture percentage is demonstrated in `example.yaml`.

_READ THE COMMENTS IN THE EXAMPLE FILE!_
---

## How it works (design summary)

The MS01 uses DHT22-compatible one-wire signalling:

1. **`update()`** — allocates one RMT RX channel, arms it to capture incoming
   pulses, then bit-bangs a 450 µs LOW start pulse via `delayMicroseconds()`
   (timing tolerance is generous so software is sufficient), then returns
   immediately.

2. **ISR** (`rmt_rx_done_cb_`) — fires from RMT interrupt context when the
   full 40-bit frame has been received (~4 ms after the start pulse).  Writes
   only two values (`num_symbols_` + `state = DONE`) and returns — no
   allocations, no FreeRTOS API, no `publish_state()`.

3. **`loop()`** — on the very next main-loop tick after the ISR fires, detects
   `state == DONE`, releases the RMT channel back to the pool, decodes the
   symbol buffer, converts raw ADC counts to voltage, and calls `publish_state()`.

The RMT channel is held for approximately **5 ms** per 60-second poll cycle.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Could not allocate RMT RX channel` | All RMT channels taken by other components (LED strip, IR, …) — check channel count for your chip variant |
| `Timeout — no response` | Wiring fault; missing or wrong pull-up; wrong GPIO number |
| `Too few RMT symbols` | Noisy bus; very long cable; weak pull-up |
| `Checksum failed` | Noisy bus; add a 100 nF decoupling cap near the sensor VCC pin |
| Moisture stuck at 0 % | Sensor not reaching soil; re-check wiring |
| Moisture > 100 % | Not physically possible with the formula; open an issue |

---

## Licence

MIT — do whatever you like with it.
