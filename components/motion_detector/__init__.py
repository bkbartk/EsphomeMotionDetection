import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

motion_detector_ns = cg.esphome_ns.namespace("motion_detector")
MotionDetector = motion_detector_ns.class_("MotionDetector", binary_sensor.BinarySensor, cg.Component)

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(MotionDetector),
    cv.Optional("threshold", default=25): cv.int_,
    cv.Optional("motion_pixels", default=2000): cv.int_,
    cv.Optional("frame_skip", default=5): cv.int_,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await binary_sensor.register_binary_sensor(var, config)

    cg.add(var.threshold.set(config["threshold"]))
    cg.add(var.motion_pixels.set(config["motion_pixels"]))
    cg.add(var.frame_skip.set(config["frame_skip"]))
