import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

motion_detector_ns = cg.esphome_ns.namespace("motion_detector")
MotionDetector = motion_detector_ns.class_("MotionDetector", binary_sensor.BinarySensor, cg.Component)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(MotionDetector).extend({
    cv.Optional("threshold", default=25): cv.int_,
    cv.Optional("motion_pixels", default=2000): cv.int_,
    cv.Optional("frame_skip", default=5): cv.int_,
})

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_threshold(config["threshold"]))
    cg.add(var.set_motion_pixels(config["motion_pixels"]))
    cg.add(var.set_frame_skip(config["frame_skip"]))

