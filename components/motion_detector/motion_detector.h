#pragma once
#include "esphome.h"
#include "esp_camera.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  // Camera must be set to 160x120 in YAML
  static const int SRC_W = 160;
  static const int SRC_H = 120;

  // Downscaled resolution
  static const int OUT_W = 64;
  static const int OUT_H = 48;

  // Buffers
  uint8_t gray_[OUT_W * OUT_H];
  float   bg_[OUT_W * OUT_H] = {0.0f};

  bool has_bg_ = false;

  int frame_skip_ = 5;
  int counter_ = 0;

  void setup() override {
    ESP_LOGI("motion", "MotionDetector setup()");

    // Force YUV422 (no JPEG decode needed)
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
      s->set_pixformat(s, PIXFORMAT_YUV422);
      ESP_LOGI("motion", "Pixel format set to YUV422");
    }
  }

  void loop() override {
    counter_++;
    if (counter_ % frame_skip_ != 0) {
      return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    // fb->buf is YUV422: Y0 U Y1 V
    uint8_t *yuv = fb->buf;

    // Convert YUV422 → grayscale (use Y only)
    int gi = 0;
    for (int i = 0; i < SRC_W * SRC_H * 2; i += 4) {
      uint8_t y0 = yuv[i];

      int px = gi % OUT_W;
      int py = gi / OUT_W;

      if (px < OUT_W && py < OUT_H) {
        gray_[py * OUT_W + px] = y0;
      }

      gi++;
      if (gi >= OUT_W * OUT_H) break;
    }

    esp_camera_fb_return(fb);

    // Initialize background
    if (!has_bg_) {
      for (int i = 0; i < OUT_W * OUT_H; i++) {
        bg_[i] = gray_[i];
      }
      has_bg_ = true;
      publish_state(false);
      return;
    }

    // Background update + diff
    int active = 0;
    const int total = OUT_W * OUT_H;

    for (int i = 0; i < total; i++) {
      float bg = bg_[i];
      float cur = gray_[i];

      bg = bg + 0.05f * (cur - bg);
      bg_[i] = bg;

      float diff = fabsf(cur - bg);
      if (diff > 20) {
        active++;
      }
    }

    bool motion = (active > total * 0.02f);
    publish_state(motion);
  }
};

}  // namespace motion_detector
}  // namespace esphome
