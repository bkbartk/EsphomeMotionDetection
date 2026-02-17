#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  // Downscaled resolution (fixed, small, fast)
  static const int OUT_W = 48;
  static const int OUT_H = 36;

  // Buffers
  uint8_t gray_[OUT_W * OUT_H];
  float   bg_[OUT_W * OUT_H] = {0.0f};

  bool has_bg_ = false;

  void setup() override {
    ESP_LOGI("motion", "MotionDetector setup()");
  }

  void loop() override {
    // Run at 1 FPS
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 1000) return;
    last = now;

    // Grab frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    int src_w = fb->width;
    int src_h = fb->height;

    // Temporary RGB buffer (dynamic, small)
    int rgb_size = src_w * src_h * 3;
    std::vector<uint8_t> rgb(rgb_size);

    // Decode JPEG → RGB888
    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb.data());
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

        uint8_t r = rgb[idx];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];

        gray_[y * OUT_W + x] = (r * 30 + g * 59 + b * 11) / 100;
      }
    }

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

      bg = bg + 0.01f * (cur - bg);  // slow update
      bg_[i] = bg;

      float diff = fabsf(cur - bg);
      if (diff > 35) {  // pixel threshold
        active++;
      }
    }

    bool motion = (active > total * 0.10f);  // 10% pixels changed
    publish_state(motion);
  }
};

}  // namespace motion_detector
}  // namespace esphome
