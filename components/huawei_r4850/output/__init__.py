import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import (
    CONF_ID,
)

from .. import HuaweiR4850Component, huawei_r4850_ns, CONF_HUAWEI_R4850_ID


CONF_FAN_DUTY_CYCLE = "fan_duty_cycle"

HuaweiR4850FloatOutput = huawei_r4850_ns.class_(
    "HuaweiR4850FloatOutput", output.FloatOutput, cg.Component
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_HUAWEI_R4850_ID): cv.use_id(HuaweiR4850Component),
            cv.Optional(CONF_FAN_DUTY_CYCLE): output.FLOAT_OUTPUT_SCHEMA.extend(
                {
                    cv.GenerateID(): cv.declare_id(HuaweiR4850FloatOutput),
                }
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUAWEI_R4850_ID])
    if CONF_FAN_DUTY_CYCLE in config:
        conf = config[CONF_FAN_DUTY_CYCLE]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await output.register_output(
            var,
            conf,
        )
        cg.add(var.set_parent(hub, 0x114))
