from esphome import pins
import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENABLE_PIN,
    CONF_UPDATE_INTERVAL,
    ICON_ARROW_EXPAND_VERTICAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)
from esphome.types import ConfigType

CODEOWNERS = ["@damienmorrissey"]
DEPENDENCIES = ["i2c"]

CONF_DISTANCE_MODE = "distance_mode"
CONF_TIMING_BUDGET = "timing_budget"

vl53l1x_ns = cg.esphome_ns.namespace("vl53l1x")
VL53L1XSensor = vl53l1x_ns.class_(
    "VL53L1XSensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

DistanceMode = vl53l1x_ns.enum("DistanceMode")
DISTANCE_MODES = {
    "short": DistanceMode.DISTANCE_MODE_SHORT,
    "medium": DistanceMode.DISTANCE_MODE_MEDIUM,
    "long": DistanceMode.DISTANCE_MODE_LONG,
}


def validate_timing_budget(config: ConfigType) -> ConfigType:
    # A measurement cannot complete if the sensor is polled faster than the time
    # it is allowed to spend on one reading.
    if config[CONF_TIMING_BUDGET] > config[CONF_UPDATE_INTERVAL]:
        raise cv.Invalid(
            f"timing_budget ({config[CONF_TIMING_BUDGET]}) must not be longer than "
            f"update_interval ({config[CONF_UPDATE_INTERVAL]})"
        )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        VL53L1XSensor,
        unit_of_measurement=UNIT_METER,
        icon=ICON_ARROW_EXPAND_VERTICAL,
        accuracy_decimals=3,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_DISTANCE_MODE, default="long"): cv.enum(
                DISTANCE_MODES, lower=True
            ),
            cv.Optional(CONF_TIMING_BUDGET, default="50ms"): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=20),
                    max=cv.TimePeriod(milliseconds=1000),
                ),
            ),
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x29)),
    validate_timing_budget,
)


async def to_code(config: ConfigType) -> None:
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_distance_mode(config[CONF_DISTANCE_MODE]))
    cg.add(var.set_timing_budget_us(config[CONF_TIMING_BUDGET]))

    if enable_pin_config := config.get(CONF_ENABLE_PIN):
        enable_pin = await cg.gpio_pin_expression(enable_pin_config)
        cg.add(var.set_enable_pin(enable_pin))
