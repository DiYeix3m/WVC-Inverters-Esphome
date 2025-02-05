import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_THROTTLE

AUTO_LOAD = ["sensor","text_sensor"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

wvc_ns = cg.esphome_ns.namespace("wvc")
WVCComponent = wvc_ns.class_("WVCComponent", uart.UARTDevice, cg.Component)

CONF_WVC_ID = "wvc_id"
CONF_INVERTER_MODEL = "inverter_model"
CONF_INVERTER_TYPE = "inverter_type"
CONF_INVERTER_SN = "inverter_sn"
CONF_THROTTLE = "throttle"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WVCComponent),
        cv.Required(CONF_INVERTER_MODEL): cv.string,
        cv.Required(CONF_INVERTER_SN): cv.string,
        cv.Optional(CONF_THROTTLE, default="5s"): cv.time_period,
    }
).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_inverter_model(config[CONF_INVERTER_MODEL]))
    cg.add(var.set_inverter_sn(config[CONF_INVERTER_SN]))
    throttle_ms = config[CONF_THROTTLE].total_milliseconds
    cg.add(var.set_throttle(throttle_ms))
