"""
sensor.py — Sonoff MS01 sensor platform
========================================
Defines the YAML schema and the Python-side code-generation function that
wires the C++ component together.

The component outputs only the raw ADC voltage from the sensor.  Moisture
percentage conversion is intentionally left to the YAML layer (template
sensors with lambda) so it can be easily tuned.
See example.yaml for ready-to-use Tasmota formula and linear calibration
lambda options.

Usage in YAML:
    sensor:
      - platform: sonoff_ms01
        pin: GPIO14
        update_interval: 60s
        voltage:
          name: "Soil ADC Voltage"
          id: soil_voltage
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    UNIT_VOLT,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
)
from esphome.core import CORE
from . import sonoff_ms01_ns, SonoffMS01Component

# ── Rather important ─────────────────────────────────────────────────────────
DEPENDENCIES = ["esp32"]

# ── Config key constants ────────────────────────────────────────────────────
CONF_VOLTAGE = "voltage"

# ── Custom validator ─────────────────────────────────────────────────────────
def _validate_platform_requirements(config):
    if not CORE.is_esp32:
        raise cv.Invalid(
            "sonoff_ms01 requires an ESP32 because it uses the RMT peripheral.\n"
            "Add an `esp32:` section to your YAML, for example:\n"
            "  esp32:\n"
            "    board: esp32dev\n"
            "    framework:\n"
            "      type: esp-idf"
        )

    if CORE.using_arduino:
        raise cv.Invalid(
            "sonoff_ms01 requires the ESP-IDF framework on ESP32 because it uses the RMT driver API.\n"
            "Update your `esp32:` section to:\n"
            "  esp32:\n"
            "    framework:\n"
            "      type: esp-idf\n"
            "      version: recommended"
        )

    return config


# ── Schema ──────────────────────────────────────────────────────────────────
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SonoffMS01Component),

            # Must be an internal GPIO (no I2C expanders — bit-bang + RMT need
            # direct pad access).
            cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,

            # Raw ADC voltage from the sensor (volts, 4 decimal places).
            # Typically ranges from ~1.5 V (saturated) to ~2.9 V (dry air).
            # Note: voltage DECREASES as moisture increases.
            cv.Required(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=4,
                icon="mdi:sine-wave",
            ),
        }
    )
    .extend(cv.polling_component_schema("60s")),  # default poll = 60 s
    # cv.only_on_esp32,
    # cv.only_with_framework("esp-idf"),
    _validate_platform_requirements,
)


# ── Code generation ─────────────────────────────────────────────────────────
async def to_code(config):
    # Instantiate the C++ object and register it as a PollingComponent
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Resolve the GPIO pin and pass it to the component
    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))

    # Wire up the voltage sensor
    sens = await sensor.new_sensor(config[CONF_VOLTAGE])
    cg.add(var.set_voltage_sensor(sens))

    # ── IDF component dependency ────────────────────────────────────────────
    # ESPHome 2026.2.0 introduced a breaking change: unused built-in IDF
    # components are now excluded from the build by default to reduce compile
    # time.  "esp_driver_rmt" owns driver/rmt_rx.h and is excluded unless
    # re-enabled here.
    #
    # include_builtin_idf_component() was added in ESPHome 2026.2.0.
    # The try/except makes the component also work on older installs where
    # all IDF components were included by default and no call was needed.
    if CORE.is_esp32:
        try:
            from esphome.components.esp32 import include_builtin_idf_component
            include_builtin_idf_component("esp_driver_rmt")
        except ImportError:
            # ESPHome < 2026.2.0 — all IDF components included by default,
            # no action needed.
            pass