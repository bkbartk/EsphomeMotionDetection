#pragma once
#include "esphome.h"
#include "esp_camera.h"
#include "img_converters.h"

namespace esphome {
namespace motion_detector {

class MotionDetector : public Component, public binary_sensor::BinarySensor {
 public:
  static const int MAX_SRC_W = 320;
  static const int MAX_SRC_H = 240;

  int out_w_ = 128;
  int out_h_ = 96;

  int pixel_diff_threshold_    = 25;
  int motion_blocks_threshold_ = 5;
  int frame_skip_              = 5;

  int block_w_ = 8;
  int block_h_ = 8;

  float bg_alpha_ = 0.05f;

  void set_output_size(int w, int h) { out_w_ = w; out_h_ = h; }
  void set_pixel_diff_threshold(int v) { pixel_diff_threshold_ = v; }
  void set_motion_blocks_threshold(int v) { motion_blocks_threshold_ = v; }
  void set_frame_skip(int v) { frame_skip_ = v; }
  void set_block_size(int bw, int bh) { block_w_ = bw; block_h_ = bh; }
  void set_background_alpha(float a) { bg_alpha_ = a; }

  static const int MAX_OUT_W = 128;
  static const int MAX_OUT_H = 96;

  // moved from stack to members
  uint8_t gray_prev_[MAX_OUT_W * MAX_OUT_H] = {0};
  float   bg_frame_[MAX_OUT_W * MAX_OUT_H]  = {0.0f};
  uint8_t gray_[MAX_OUT_W * MAX_OUT_H]      = {0};
  // worst case: (128/1) * (96/1) blocks = 12288, but we clamp via block size
  static const int MAX_BLOCKS = 64 * 64;
  float block_diff_[MAX_BLOCKS] = {0.0f};

  bool has_prev_ = false;
  bool has_bg_   = false;

  static const int RGB_SIZE = MAX_SRC_W * MAX_SRC_H * 3;
  uint8_t rgb_[RGB_SIZE];

  int counter_ = 0;

  void setup() override {
    ESP_LOGI("motion", "Motion detector (block-based + background) initialized");
    if (out_w_ > MAX_OUT_W || out_h_ > MAX_OUT_H) {
      ESP_LOGW("motion", "Configured output size too large, clamping to %dx%d",
               MAX_OUT_W, MAX_OUT_H);
      out_w_ = MAX_OUT_W;
      out_h_ = MAX_OUT_H;
    }
  }

  void loop() override {
    counter_++;
    if (counter_ % frame_skip_ != 0)
      return;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW("motion", "Failed to get frame");
      return;
    }

    const int src_w = fb->width;
    const int src_h = fb->height;

    if (src_w > MAX_SRC_W || src_h > MAX_SRC_H) {
      ESP_LOGW("motion",
               "Source frame too large (%dx%d). "
               "Set camera resolution to <= %dx%d for motion detection.",
               src_w, src_h, MAX_SRC_W, MAX_SRC_H);
      esp_camera_fb_return(fb);
      publish_state(false);
      return;
    }

    const int needed_rgb = src_w * src_h * 3;
    if (needed_rgb > RGB_SIZE) {
      ESP_LOGW("motion", "Internal RGB buffer too small for %dx%d", src_w, src_h);
      esp_camera_fb_return(fb);
      publish_state(false);
      return;
    }

    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_);
    esp_camera_fb_return(fb);

    if (!ok) {
      ESP_LOGW("motion", "JPEG decode failed");
      return;
    }

    const int W = out_w_;
    const int H = out_h_;

    // downscale + grayscale into member buffer gray_
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

    if (!has_bg_) {
      for (int i = 0; i < W * H; i++) {
        bg_frame_[i] = gray_[i];
      }
      has_bg_ = true;
    }

    const int blocks_x = W / block_w_;
    const int blocks_y = H / block_h_;
    const int total_blocks = blocks_x * blocks_y;

    if (total_blocks > MAX_BLOCKS) {
      ESP_LOGW("motion", "Too many blocks (%d), adjust block size", total_blocks);
      publish_state(false);
      return;
    }

    for (int i = 0; i < total_blocks; i++) {
      block_diff_[i] = 0.0f;
    }

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int idx = y * W + x;

        float bg_val = bg_frame_[idx];
        float cur    = gray_[idx];
        bg_val = bg_val + bg_alpha_ * (cur - bg_val);
        bg_frame_[idx] = bg_val;

        float diff = fabsf(cur - bg_val);

        int bx = x / block_w_;
        int by = y / block_h_;
        int b  = by * blocks_x + bx;

        block_diff_[b] += diff;
      }
    }

    int active_blocks = 0;
    const float pixels_per_block = (float)(block_w_ * block_h_);
    for (int b = 0; b < total_blocks; b++) {
      float avg_diff = block_diff_[b] / pixels_per_block;
      if (avg_diff > pixel_diff_threshold_) {
        active_blocks++;
      }
    }

    bool motion = (active_blocks >= motion_blocks_threshold_);
    publish_state(motion);

    memcpy(gray_prev_, gray_, W * H);
    has_prev_ = true;
  }
};

}  // namespace motion_detector
}  // namespace esphome
