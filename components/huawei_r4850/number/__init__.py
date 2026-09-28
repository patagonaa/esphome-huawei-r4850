import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_MODE,
    CONF_RESTORE_VALUE,
    CONF_STEP,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_CONFIG,
    ICON_CURRENT_AC,
    ICON_FAN,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_VOLT,
)
from esphome.types import ConfigType
from .. import CONF_HUAWEI_R4850_ID, HUAWEI_R4850_COMPONENT_SCHEMA, huawei_r4850_ns

ICON_CURRENT_DC = "mdi:current-dc"

CONF_RESEND = "resend"

CONF_OUTPUT_VOLTAGE = "output_voltage"
CONF_OUTPUT_VOLTAGE_DEFAULT = "output_voltage_default"
CONF_MAX_OUTPUT_CURRENT = "max_output_current"
CONF_MAX_OUTPUT_CURRENT_DEFAULT = "max_output_current_default"
CONF_MAX_AC_CURRENT = "max_ac_current"
CONF_FAN_DUTY_CYCLE = "fan_duty_cycle"

REGISTER_IDS = {
    CONF_OUTPUT_VOLTAGE: 0x100,
    CONF_OUTPUT_VOLTAGE_DEFAULT: 0x101,
    CONF_MAX_OUTPUT_CURRENT: 0x103,
    CONF_MAX_OUTPUT_CURRENT_DEFAULT: 0x104,
    CONF_MAX_AC_CURRENT: 0x109,
    CONF_FAN_DUTY_CYCLE: 0x114,
}

HuaweiR4850Number = huawei_r4850_ns.class_(
    "HuaweiR4850Number", number.Number, cg.Component
)

CONFIG_SCHEMA = HUAWEI_R4850_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_OUTPUT_VOLTAGE): number.number_schema(
            HuaweiR4850Number,
            device_class=DEVICE_CLASS_VOLTAGE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_VOLT,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=41.0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=58.6): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
                cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=True): cv.boolean,
            }
        ),
        cv.Optional(CONF_OUTPUT_VOLTAGE_DEFAULT): number.number_schema(
            HuaweiR4850Number,
            device_class=DEVICE_CLASS_VOLTAGE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_VOLT,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=48.0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=58.4): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
                cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=False): cv.boolean,
            }
        ),
        cv.Optional(CONF_MAX_OUTPUT_CURRENT): number.number_schema(
            HuaweiR4850Number,
            icon=ICON_CURRENT_DC,
            device_class=DEVICE_CLASS_CURRENT,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_AMPERE,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=0.0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=63.3): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
                cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=True): cv.boolean,
            }
        ),
        cv.Optional(CONF_MAX_OUTPUT_CURRENT_DEFAULT): number.number_schema(
            HuaweiR4850Number,
            icon=ICON_CURRENT_DC,
            device_class=DEVICE_CLASS_CURRENT,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_AMPERE,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=0.0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=63.3): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
                cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=False): cv.boolean,
            }
        ),
        cv.Optional(CONF_MAX_AC_CURRENT): number.number_schema(
            HuaweiR4850Number,
            icon=ICON_CURRENT_AC,
            device_class=DEVICE_CLASS_CURRENT,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_AMPERE,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=20): cv.float_,
                cv.Optional(CONF_STEP, default=0.1): cv.float_,
                cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=False): cv.boolean,
            }
        ),
        cv.Optional(CONF_FAN_DUTY_CYCLE): number.number_schema(
            HuaweiR4850Number,
            icon=ICON_FAN,
            entity_category=ENTITY_CATEGORY_CONFIG,
            unit_of_measurement=UNIT_PERCENT,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_range(min=0, max=100),
                cv.Optional(CONF_MAX_VALUE, default=100): cv.float_range(min=0, max=100),
                cv.Optional(CONF_STEP, default=1): cv.float_,
                cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(number.NUMBER_MODES, upper=True),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
                cv.Optional(CONF_RESEND, default=True): cv.boolean,
            }
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_HUAWEI_R4850_ID])

    for (number_name, register_id) in REGISTER_IDS.items():
        if number_name in config:
            conf = config[number_name]
            var = cg.new_Pvariable(conf[CONF_ID])
            await cg.register_component(var, conf)
            await number.register_number(
                var,
                conf,
                min_value=conf[CONF_MIN_VALUE],
                max_value=conf[CONF_MAX_VALUE],
                step=conf[CONF_STEP],
            )
            cg.add(getattr(hub, "register_input")(var))
            cg.add(var.set_parent(hub, register_id))
            cg.add(var.set_restore_value(conf[CONF_RESTORE_VALUE]))
            cg.add(var.set_resend(conf[CONF_RESEND]))
