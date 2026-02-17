#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  // Supported max input resolution
  static const int MAX_SRC_W = 320;
  static const int MAX_SRC_H = 240;

  // Default downscaled resolution
  static const int MAX_OUT_W = 128;
  static const int MAX_OUT_H = 96;

  // Internal working buffers (static, safe)
  uint8_t rgb_[MAX_SRC_W * MAX_SRC_H * 3];                 // RGB888 buffer
  uint8_t gray_[MAX_OUT_W * MAX_OUT_H];                    // grayscale downscaled
  float   bg_frame_[MAX_OUT_W * MAX_OUT_H] = {0.0f};       // background model
  uint8_t gray_prev_[MAX_OUT_W * MAX_OUT_H] = {0};         // previous frame

  bool has_bg_ = false;
  bool has_prev_ = false;

  int out_w_ = 128;
  int out_h_ = 96;

  int frame_skip_ = 5;
  int counter_ = 0;

  void setup() override {
    ESP_LOGI("motion", "MotionDetector setup()");
  }

  void loop() override {
    // Run only every Nth loop
    counter_++;
    if (counter_ % frame_skip_ != 0) {
      return;
    }

    // Ensure camera is initialized
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
      return;
    }

    // Try to get a frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    const int src_w = fb->width;
    const int src_h = fb->height;

    // Guard: only support up to 320x240
    if (src_w > MAX_SRC_W || src_h > MAX_SRC_H) {
      ESP_LOGW("motion",
               "Source frame too large (%dx%d). "
               "Set camera resolution to <= %dx%d.",
               src_w, src_h, MAX_SRC_W, MAX_SRC_H);
      esp_camera_fb_return(fb);
      return;
    }

    const int needed_rgb = src_w * src_h * 3;
    if (needed_rgb > sizeof(rgb_)) {
      ESP_LOGW("motion", "RGB buffer too small for %dx%d", src_w, src_h);
      esp_camera_fb_return(fb);
      return;
    }

    // Decode JPEG → RGB888
    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_);
    esp_camera_fb_return(fb);

    if (!ok) {
      ESP_LOGW("motion", "JPEG decode failed");
      return;
    }

    // Downscale + grayscale
    const int W = out_w_;
    const int H = out_h_;

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int sx = x * src_w / W;
        int sy = y * src_h / H;
        int idx = (sy * src_w + sx) * 3;

        uint8_t r = rgb_[idx];
        uint8_t g = rgb_[idx + 1];
        uint8_t b = rgb_[idx + 2];

        gray_[y * W + x] = (r * 30 + g * 59 + b * 11) / 100;
      }
    }

    ESP_LOGI("motion", "Decoded + downscaled to %dx%d", W, H);

    // No motion detection yet — just publish "false"
    publish_state(false);
  }
};

}  // namespace motion_detector
}  // namespace esphome
