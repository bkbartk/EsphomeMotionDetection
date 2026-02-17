#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  // Assume camera is set to 160x120 in YAML
  static const int SRC_W = 160;
  static const int SRC_H = 120;

  // Downscaled size
  static const int OUT_W = 64;
  static const int OUT_H = 48;

  // Much smaller buffers now
  uint8_t rgb_[SRC_W * SRC_H * 3];          // 160*120*3 = 57,600 bytes
  uint8_t gray_[OUT_W * OUT_H];            // 3,072 bytes

  int frame_skip_ = 5;
  int counter_ = 0;

  void setup() override {
    ESP_LOGI("motion", "MotionDetector setup()");
  }

  void loop() override {
    // Throttle by frame_skip
    counter_++;
    if (counter_ % frame_skip_ != 0) {
      return;
    }

    // Ensure camera is ready
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
      return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    const int src_w = fb->width;
    const int src_h = fb->height;

    if (src_w != SRC_W || src_h != SRC_H) {
      ESP_LOGW("motion",
               "Unexpected frame size %dx%d, expected %dx%d. "
               "Set camera resolution to 160x120.",
               src_w, src_h, SRC_W, SRC_H);
      esp_camera_fb_return(fb);
      return;
    }

    const int needed_rgb = src_w * src_h * 3;
    if (needed_rgb > sizeof(rgb_)) {
      ESP_LOGW("motion", "RGB buffer too small for %dx%d", src_w, src_h);
      esp_camera_fb_return(fb);
      return;
    }

    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_);
    esp_camera_fb_return(fb);

    if (!ok) {
      ESP_LOGW("motion", "JPEG decode failed");
      return;
    }

    // Downscale + grayscale to 64x48
    for (int y = 0; y < OUT_H; y++) {
      for (int x = 0; x < OUT_W; x++) {
        int sx = x * src_w / OUT_W;
        int sy = y * src_h / OUT_H;
        int idx = (sy * src_w + sx) * 3;

        uint8_t r = rgb_[idx];
        uint8_t g = rgb_[idx + 1];
        uint8_t b = rgb_[idx + 2];

        gray_[y * OUT_W + x] = (r * 30 + g * 59 + b * 11) / 100;
      }
    }

    ESP_LOGI("motion", "Decoded + downscaled to %dx%d", OUT_W, OUT_H);

    // For now, no real motion detection — just publish false
    publish_state(false);
  }
};

}  // namespace motion_detector
}  // namespace esphome
