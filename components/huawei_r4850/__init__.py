import esphome.codegen as cg
from esphome.components.canbus import CanbusComponent
import esphome.config_validation as cv
from esphome.const import CONF_ID
import esphome.final_validate as fv
from esphome.types import ConfigType

MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
CONF_HUAWEI_R4850_ID = "huawei_r4850_id"
CONF_PSU_ADDRESS = "psu_address"
CONF_PSU_SLOT_ID = "psu_slot_id"
CONF_RESEND_INTERVAL = "resend_interval"

huawei_r4850_ns = cg.esphome_ns.namespace("huawei_r4850")
HuaweiR4850Component = huawei_r4850_ns.class_(
    "HuaweiR4850Component", cg.PollingComponent
)

HUAWEI_R4850_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_HUAWEI_R4850_ID): cv.use_id(HuaweiR4850Component),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HuaweiR4850Component),
            cv.Required(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
            cv.Optional(CONF_PSU_ADDRESS): cv.int_range(min=1, max=127),
            cv.Optional(CONF_PSU_SLOT_ID): cv.uint16_t,
            cv.Optional(CONF_RESEND_INTERVAL, default="5s"): cv.update_interval,
        }
    ).extend(cv.polling_component_schema("5s")),
    cv.has_exactly_one_key(CONF_PSU_ADDRESS, CONF_PSU_SLOT_ID)
)


async def to_code(config):
    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    canbus_var = cg.new_Pvariable(config[CONF_ID], canbus)
    await cg.register_component(canbus_var, config)

    hub = await cg.get_variable(config[CONF_ID])
    if CONF_PSU_ADDRESS in config:
        cg.add(hub.set_psu_address(config[CONF_PSU_ADDRESS]))
    if CONF_PSU_SLOT_ID in config:
        cg.add(hub.set_psu_slot_id(config[CONF_PSU_SLOT_ID]))
    cg.add(hub.set_resend_interval(config[CONF_RESEND_INTERVAL]))


def final_validate(config: ConfigType) -> None:
    full_config = fv.full_config.get()

    other_configs_on_bus = (
        other_config for
        other_config in full_config.get("huawei_r4850", [])
        if other_config[CONF_ID] != config[CONF_ID] and other_config[CONF_CANBUS_ID] == config[CONF_CANBUS_ID]
        )

    for other_config in other_configs_on_bus:
        if CONF_PSU_SLOT_ID in config and CONF_PSU_SLOT_ID not in other_config:
            raise cv.Invalid(
                "Addressing via slot id and address must not be mixed on a single CAN bus.",
                path=[CONF_PSU_SLOT_ID, config[CONF_PSU_SLOT_ID]],
            )

        if CONF_PSU_ADDRESS in config and CONF_PSU_ADDRESS not in other_config:
            raise cv.Invalid(
                "Addressing via slot id and address must not be mixed on a single CAN bus.",
                path=[CONF_PSU_ADDRESS, config[CONF_PSU_ADDRESS]],
            )

        if CONF_PSU_SLOT_ID in config and other_config[CONF_PSU_SLOT_ID] == config[CONF_PSU_SLOT_ID]:
            raise cv.Invalid(
                f"Duplicate slot id on bus '{config[CONF_CANBUS_ID]}'.",
                path=[CONF_PSU_SLOT_ID, config[CONF_PSU_SLOT_ID]],
            )

        if CONF_PSU_ADDRESS in config and other_config[CONF_PSU_ADDRESS] == config[CONF_PSU_ADDRESS]:
            raise cv.Invalid(
                f"Duplicate address on bus '{config[CONF_CANBUS_ID]}'.",
                path=[CONF_PSU_ADDRESS, config[CONF_PSU_ADDRESS]],
            )


FINAL_VALIDATE_SCHEMA = final_validate
