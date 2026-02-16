#pragma once
#include "esphome.h"
#include "esp_camera.h"

class MotionDetector : public esphome::Component,
                       public esphome::camera::CameraImageReader,
                       public esphome::binary_sensor::BinarySensor {
 public:
  int threshold = 25;        // pixel difference threshold
  int motion_pixels = 2000;  // number of changed pixels required
  int frame_skip = 5;        // analyze every N frames

  camera_fb_t *last_frame = nullptr;
  int counter = 0;

  void setup() override {
    ESP_LOGI("motion", "Motion detector initialized");
  }

  void on_frame(camera_fb_t *fb) override {
    counter++;
    if (counter % frame_skip != 0) return;

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
