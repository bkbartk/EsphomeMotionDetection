#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  // Downscaled working resolution (for motion detection only)
  static const int W = 80;
  static const int H = 60;

  // Thresholds and tuning
  int pixel_diff_threshold = 25;   // per-pixel grayscale diff threshold
  int motion_blocks_threshold = 5; // how many blocks must be "active" to trigger motion

  // Frame skipping
  int frame_skip = 5;

  // Block configuration (overridable)
  int block_w = 8;
  int block_h = 8;

  // Background averaging speed (alpha, 0..1), overridable
  // 0.05 = slow (good for outdoor)
  float bg_alpha = 0.05f;

  // Setters for configuration
  void set_pixel_diff_threshold(int v) { pixel_diff_threshold = v; }
  void set_motion_blocks_threshold(int v) { motion_blocks_threshold = v; }
  void set_frame_skip(int v) { frame_skip = v; }
  void set_block_size(int bw, int bh) { block_w = bw; block_h = bh; }
  void set_background_alpha(float a) { bg_alpha = a; }

  // Internal buffers
  uint8_t gray_prev[W * H] = {0};
  float   bg_frame[W * H]  = {0.0f};  // rolling background (grayscale)
  bool    has_prev = false;
  bool    has_bg   = false;

  int counter = 0;

  void setup() override {
    ESP_LOGI("motion", "Motion detector (block-based + background) initialized");
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

    // Use actual frame size from camera
    const int src_w = fb->width;
    const int src_h = fb->height;

    // Allocate buffer for decoded RGB888 frame
    const int RGB_SIZE = src_w * src_h * 3;
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

    // Downscale + grayscale into 80x60
    uint8_t gray[W * H];

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int sx = x * src_w / W;
        int sy = y * src_h / H;
        int idx = (sy * src_w + sx) * 3;

        uint8_t r = rgb[idx];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];

        gray[y * W + x] = (r * 30 + g * 59 + b * 11) / 100;
      }
    }

    free(rgb);

    // Initialize background on first frame
    if (!has_bg) {
      for (int i = 0; i < W * H; i++) {
        bg_frame[i] = gray[i];
      }
      has_bg = true;
    }

    // Block grid dimensions
    const int blocks_x = W / block_w;
    const int blocks_y = H / block_h;
    const int total_blocks = blocks_x * blocks_y;

    // Accumulated diff per block
    // Using float for convenience; could be int for tighter memory
    float block_diff[total_blocks];
    for (int i = 0; i < total_blocks; i++) {
      block_diff[i] = 0.0f;
    }

    // Update background and accumulate block differences
    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int idx = y * W + x;

        // Background exponential moving average
        float bg_val = bg_frame[idx];
        float cur    = gray[idx];
        bg_val = bg_val + bg_alpha * (cur - bg_val);
        bg_frame[idx] = bg_val;

        // Difference from background
        float diff = fabsf(cur - bg_val);

        int bx = x / block_w;
        int by = y / block_h;
        int b  = by * blocks_x + bx;

        block_diff[b] += diff;
      }
    }

    // Count active blocks
    int active_blocks = 0;
    const float pixels_per_block = (float)(block_w * block_h);
    for (int b = 0; b < total_blocks; b++) {
      float avg_diff = block_diff[b] / pixels_per_block;
      if (avg_diff > pixel_diff_threshold) {
        active_blocks++;
      }
    }

    // Compare with previous frame (optional extra stability)
    if (has_prev) {
      // You can optionally blend in previous-frame comparison here if desired.
      // For now, we rely on background-based detection only.
    }

    // Motion decision
    bool motion = (active_blocks >= motion_blocks_threshold);
    publish_state(motion);

    // Save current grayscale as previous frame
    memcpy(gray_prev, gray, W * H);
    has_prev = true;
  }
};

}  // namespace motion_detector
}  // namespace esphome
