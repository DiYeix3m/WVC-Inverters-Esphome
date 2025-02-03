import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ICON_CURRENT_AC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)
from . import CONF_ID, WVCComponent

DEPENDENCIES = ["wvc"]

CONF_TEMPERATURE = "temperature"
CONF_VAC = "vac"
CONF_AAC = "aac"
CONF_VDC = "vdc"
CONF_ADC = "adc"
CONF_EFF = "eff"
CONF_ACW = "acw"
CONF_DCW = "dcw"
ICON_CURRENT_DC = "mdi:current-dc"
ICON_DC_WATTS = "mdi:solar-power"
ICON_AC = "mdi:sine-wave"
ICON_EFF = "mdi:cloud-cog-outline"
ICON_AC_WATTS = "mdi:transmission-tower-import"
ICON_TEMPERATURE_C ="mdi:thermometer-lines"

SENSORS = [
    CONF_TEMPERATURE,
    CONF_VAC,
    CONF_AAC,
    CONF_VDC,
    CONF_ADC,
    CONF_EFF,
    CONF_ACW,
    CONF_DCW,
]

# Helper function to avoid repetition in sensor schema creation
def create_sensor_schema(unit, icon, accuracy, device_class):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        icon=icon,
        accuracy_decimals=accuracy,
        device_class=device_class,
    )

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.use_id(WVCComponent),
        cv.Optional(CONF_TEMPERATURE): create_sensor_schema(
            unit=UNIT_CELSIUS, icon=ICON_TEMPERATURE_C, accuracy=0, device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(CONF_VAC): create_sensor_schema(
            unit=UNIT_VOLT, icon=ICON_AC, accuracy=2, device_class=DEVICE_CLASS_VOLTAGE, 
        ),
        cv.Optional(CONF_VDC): create_sensor_schema(
            unit=UNIT_VOLT, icon=ICON_CURRENT_DC, accuracy=2, device_class=DEVICE_CLASS_VOLTAGE, 
        ),
        cv.Optional(CONF_AAC): create_sensor_schema(
            unit=UNIT_AMPERE, icon=ICON_CURRENT_AC, accuracy=2, device_class=DEVICE_CLASS_CURRENT,
        ),
        cv.Optional(CONF_ADC): create_sensor_schema(
            unit=UNIT_AMPERE, icon=ICON_CURRENT_DC, accuracy=2, device_class=DEVICE_CLASS_CURRENT,
        ),
        cv.Optional(CONF_EFF): create_sensor_schema(
            unit=UNIT_PERCENT, icon=ICON_EFF, accuracy=0, device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_ACW): create_sensor_schema(
            unit=UNIT_WATT, icon=ICON_AC_WATTS, accuracy=0, device_class=DEVICE_CLASS_ENERGY,
        ),
        cv.Optional(CONF_DCW): create_sensor_schema(
            unit=UNIT_WATT, icon=ICON_DC_WATTS, accuracy=0, device_class=DEVICE_CLASS_ENERGY,
        ),
    }
)

def to_code(config):
    hub = yield cg.get_variable(config[CONF_ID])
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
