import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_HUAWEI_R4850_ID, HUAWEI_R4850_COMPONENT_SCHEMA

CONF_BOARD_TYPE = "board_type"
CONF_SERIAL_NUMBER = "serial_number"
CONF_ITEM = "item"
CONF_MODEL = "model"

TEXT_SENSORS = {
    CONF_BOARD_TYPE: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_SERIAL_NUMBER: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_ITEM: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_MODEL: text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

CONFIG_SCHEMA = HUAWEI_R4850_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(sensor_name): sensor_config
        for sensor_name, sensor_config in TEXT_SENSORS.items()
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUAWEI_R4850_ID])
    for sensor_name in TEXT_SENSORS:
        if sensor_name in config:
            conf = config[sensor_name]
            sens = await text_sensor.new_text_sensor(conf)
            cg.add(getattr(hub, f"set_{sensor_name}_text_sensor")(sens))
