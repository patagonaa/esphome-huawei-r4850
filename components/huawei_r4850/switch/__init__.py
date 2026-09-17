import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_RESTORE_MODE,
    ENTITY_CATEGORY_CONFIG,
    ICON_FAN,
    ICON_POWER,
)

from .. import CONF_HUAWEI_R4850_ID, HUAWEI_R4850_COMPONENT_SCHEMA, huawei_r4850_ns

CONF_FAN_SPEED_MAX = "fan_speed_max"
CONF_STANDBY = "standby"

HuaweiR4850Switch = huawei_r4850_ns.class_(
    "HuaweiR4850Switch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = HUAWEI_R4850_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FAN_SPEED_MAX): switch.switch_schema(
            HuaweiR4850Switch, icon=ICON_FAN, entity_category=ENTITY_CATEGORY_CONFIG
        ).extend(
            {
                cv.Optional(CONF_RESTORE_MODE, default="RESTORE_DEFAULT_OFF"): cv.enum(
                    switch.RESTORE_MODES, upper=True, space="_"
                ),
            }
        ),
        cv.Optional(CONF_STANDBY): switch.switch_schema(
            HuaweiR4850Switch, icon=ICON_POWER, entity_category=ENTITY_CATEGORY_CONFIG
        ).extend(
            {
                cv.Optional(CONF_RESTORE_MODE, default="RESTORE_DEFAULT_OFF"): cv.enum(
                    switch.RESTORE_MODES, upper=True, space="_"
                ),
            }
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUAWEI_R4850_ID])
    if CONF_FAN_SPEED_MAX in config:
        conf = config[CONF_FAN_SPEED_MAX]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await switch.register_switch(var, conf)
        cg.add(getattr(hub, "register_input")(var))
        cg.add(var.set_parent(hub, 0x134))
        cg.add(var.set_restore_mode(conf[CONF_RESTORE_MODE]))

    if CONF_STANDBY in config:
        conf = config[CONF_STANDBY]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await switch.register_switch(var, conf)
        cg.add(getattr(hub, "register_input")(var))
        cg.add(var.set_parent(hub, 0x132))
        cg.add(var.set_restore_mode(conf[CONF_RESTORE_MODE]))
