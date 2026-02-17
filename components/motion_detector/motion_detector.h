#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  int threshold = 25;          // grayscale diff threshold
  int motion_pixels = 200;     // number of changed pixels needed
  int frame_skip = 5;          // skip frames to reduce load

  void set_threshold(int v) { threshold = v; }
  void set_motion_pixels(int v) { motion_pixels = v; }
  void set_frame_skip(int v) { frame_skip = v; }

  // Downscaled grayscale buffer size
  static const int W = 80;
  static const int H = 60;

  uint8_t gray_prev[W * H] = {0};
  bool has_prev = false;

  int counter = 0;

  void setup() override {
    ESP_LOGI("motion", "Motion detector initialized");
  }

  void loop() override {
    counter++;
    if (counter % frame_skip != 0)
      return;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    // Allocate buffer for decoded RGB888 frame
    // (only temporarily, then freed)
    const int RGB_SIZE = 640 * 480 * 3;
    uint8_t *rgb = (uint8_t *) malloc(RGB_SIZE);
    if (!rgb) {
      ESP_LOGW("motion", "Failed to allocate RGB buffer");
      esp_camera_fb_return(fb);
      return;
    }

    // Decode JPEG → RGB888
    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
    esp_camera_fb_return(fb);

    if (!ok) {
      ESP_LOGW("motion", "JPEG decode failed");
      free(rgb);
      return;
    }

    // Downscale + grayscale
    uint8_t gray[W * H];

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int sx = x * 640 / W;
        int sy = y * 480 / H;
        int idx = (sy * 640 + sx) * 3;

        uint8_t r = rgb[idx];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];

        // Convert to grayscale
        gray[y * W + x] = (r * 30 + g * 59 + b * 11) / 100;
      }
    }

    free(rgb);

    // Compare with previous grayscale frame
    if (has_prev) {
      int changes = 0;

      for (int i = 0; i < W * H; i++) {
        int diff = abs(gray[i] - gray_prev[i]);
        if (diff > threshold)
          changes++;
      }

      publish_state(changes > motion_pixels);
    }

    // Save current frame
    memcpy(gray_prev, gray, W * H);
    has_prev = true;
  }
};

}  // namespace motion_detector
}  // namespace esphome
