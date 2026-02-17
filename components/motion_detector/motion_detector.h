#pragma once
#include "esphome.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  void setup() override {
    ESP_LOGI("motion", "MotionDetector setup()");
  }

  void loop() override {
    // Just to prove it's alive
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last > 5000) {
      last = now;
      ESP_LOGI("motion", "MotionDetector loop() still running");
    }
  }
};

}  // namespace motion_detector
}  // namespace esphome
