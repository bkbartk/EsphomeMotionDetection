import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

motion_detector_ns = cg.esphome_ns.namespace("motion_detector")
MotionDetector = motion_detector_ns.class_("MotionDetector", cg.Component, binary_sensor.BinarySensor)

CONF_PIXEL_DIFF_THRESHOLD = "pixel_diff_threshold"
CONF_MOTION_BLOCKS_THRESHOLD = "motion_blocks_threshold"
CONF_FRAME_SKIP = "frame_skip"
CONF_BLOCK_WIDTH = "block_width"
CONF_BLOCK_HEIGHT = "block_height"
CONF_BACKGROUND_ALPHA = "background_alpha"
CONF_OUTPUT_WIDTH = "output_width"
CONF_OUTPUT_HEIGHT = "output_height"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(MotionDetector).extend({
    cv.Optional(CONF_PIXEL_DIFF_THRESHOLD, default=25): cv.int_,
    cv.Optional(CONF_MOTION_BLOCKS_THRESHOLD, default=5): cv.int_,
    cv.Optional(CONF_FRAME_SKIP, default=5): cv.int_,
    cv.Optional(CONF_BLOCK_WIDTH, default=8): cv.int_,
    cv.Optional(CONF_BLOCK_HEIGHT, default=8): cv.int_,
    cv.Optional(CONF_BACKGROUND_ALPHA, default=0.05): cv.float_range(min=0.0, max=1.0),
    cv.Optional(CONF_OUTPUT_WIDTH, default=128): cv.int_,
    cv.Optional(CONF_OUTPUT_HEIGHT, default=96): cv.int_,
})

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_pixel_diff_threshold(config[CONF_PIXEL_DIFF_THRESHOLD]))
    cg.add(var.set_motion_blocks_threshold(config[CONF_MOTION_BLOCKS_THRESHOLD]))
    cg.add(var.set_frame_skip(config[CONF_FRAME_SKIP]))
    cg.add(var.set_block_size(config[CONF_BLOCK_WIDTH], config[CONF_BLOCK_HEIGHT]))
    cg.add(var.set_background_alpha(config[CONF_BACKGROUND_ALPHA]))
    cg.add(var.set_output_size(config[CONF_OUTPUT_WIDTH], config[CONF_OUTPUT_HEIGHT]))
