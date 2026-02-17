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
   // 1. Ensure camera is initialized
   sensor_t *s = esp_camera_sensor_get();
   if (s == nullptr) {
     // Camera not ready yet
     return;
   }
 
   // 2. Try to get a frame
   camera_fb_t *fb = esp_camera_fb_get();
   if (!fb) {
     ESP_LOGW("motion", "Failed to get frame");
     return;
   }
 
   // 3. Log once to confirm this part works
   ESP_LOGI("motion", "Got frame %dx%d", fb->width, fb->height);
 
   // 4. Return the frame immediately (no processing yet)
   esp_camera_fb_return(fb);
 }

};

}  // namespace motion_detector
}  // namespace esphome
