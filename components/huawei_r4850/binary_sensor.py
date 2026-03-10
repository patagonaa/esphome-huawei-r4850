import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_HUAWEI_R4850_ID, HUAWEI_R4850_COMPONENT_SCHEMA

CONF_CANBUS_CONNECTIVITY = "canbus_connectivity"

BINARY_SENSORS = {
    CONF_CANBUS_CONNECTIVITY: binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

CONFIG_SCHEMA = HUAWEI_R4850_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(sensor_name): sensor_config
        for sensor_name, sensor_config in BINARY_SENSORS.items()
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_HUAWEI_R4850_ID])
    for sensor_name in BINARY_SENSORS:
        if sensor_name in config:
            conf = config[sensor_name]
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(hub, f"set_{sensor_name}_binary_sensor")(sens))
