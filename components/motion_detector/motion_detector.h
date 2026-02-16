#pragma once
#include "esphome.h"
#include "esp_camera.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  int threshold = 25;
  int motion_pixels = 2000;
  int frame_skip = 5;

  camera_fb_t *last_frame = nullptr;
  int counter = 0;

  void setup() override {
    ESP_LOGI("motion", "Motion detector initialized");
  }

  void loop() override {
    counter++;
    if (counter % frame_skip != 0) return;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    if (last_frame) {
      int changes = 0;
      for (int i = 0; i < fb->len; i++) {
        int diff = abs(fb->buf[i] - last_frame->buf[i]);
        if (diff > threshold) changes++;
      }

      publish_state(changes > motion_pixels);

      esp_camera_fb_return(last_frame);
    }

    last_frame = fb;
  }
};

}  // namespace motion_detector
}  // namespace esphome
