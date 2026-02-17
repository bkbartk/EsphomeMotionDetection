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
  counter_++;
  if (counter_ % frame_skip_ != 0) {
    return;
  }

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

  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_);
  esp_camera_fb_return(fb);

  if (!ok) {
    ESP_LOGW("motion", "JPEG decode failed");
    return;
  }

  // Downscale + grayscale
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

  // Initialize background on first run
  if (!has_bg_) {
    for (int i = 0; i < OUT_W * OUT_H; i++) {
      bg_frame_[i] = gray_[i];
    }
    has_bg_ = true;
    publish_state(false);
    return;
  }

  // Background update + diff
  int active_pixels = 0;
  const int total_pixels = OUT_W * OUT_H;

  for (int i = 0; i < total_pixels; i++) {
    float bg = bg_frame_[i];
    float cur = gray_[i];

    // Update background slowly
    bg = bg + 0.05f * (cur - bg);
    bg_frame_[i] = bg;

    // Pixel difference
    float diff = fabsf(cur - bg);

    if (diff > 20) {  // threshold
      active_pixels++;
    }
  }

  // Motion decision
  bool motion = (active_pixels > (total_pixels * 0.02));  // 2% pixels changed

  publish_state(motion);
}

};

}  // namespace motion_detector
}  // namespace esphome
