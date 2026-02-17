import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

motion_detector_ns = cg.esphome_ns.namespace("motion_detector")
MotionDetector = motion_detector_ns.class_("MotionDetector", cg.Component, binary_sensor.BinarySensor)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(MotionDetector)

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)
