#pragma once

#ifdef USE_ESP32_VARIANT_ESP32S3
#include "esphome/components/st7701s/st7701s.h"
#include "esphome/components/lvgl/lvgl_esphome.h"
#include "esphome/core/helpers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>

namespace esphome::crowpanel_rgb {

// Tear-free variant of the ST7701S RGB driver for the Elecrow CrowPanel 2.1" knob.
//
// ESPHome's st7701s has one framebuffer, so LVGL writes into the buffer the panel is scanning
// (tearing), and it restarts the panel stream every loop to mask drift. Here:
//  - LVGL renders into a single working CANVAS (PSRAM) that is never displayed (LVGL direct mode,
//    one buffer, switched on once ESPHome has created the LVGL display). The canvas is always the
//    complete, current picture.
//  - Two display framebuffers mirror the canvas. After each LVGL frame, this frame's and last
//    frame's changed areas are copied canvas -> hidden display buffer, which is then handed to
//    the driver; we wait for on_frame_buf_complete (the driver has finished READING the old
//    buffer). Not on_vsync: bounce buffers read ahead of vsync.
//    (An earlier version let LVGL alternate between the two display buffers itself; they drifted
//    apart in practice — stale regions flashed during animations. A single canvas makes that
//    impossible by construction.)
//  - The panel sets the pace: LVGL's refresh timer runs every 2ms and the main loop at full speed,
//    and present blocks until the panel is ready.
//  - slide(): animate page changes by moving pixels between a snapshot of the old page and the
//    canvas (new page) at the framebuffer level — LVGL can only redraw the full screen at ~10 fps.
//  - bounce-buffer ISR stays on the LVGL core (core 0 caused PSRAM contention), 40 lines of slack.
class CrowpanelRGB : public st7701s::ST7701S {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr, display::ColorOrder order,
                      display::ColorBitness bitness, bool big_endian, int x_offset, int y_offset, int x_pad) override;
  void set_log_fps(bool log_fps) { this->log_fps_ = log_fps; }

  // Slide to the next/previous LVGL page. From YAML: lambda: 'id(my_display).slide(id(lvgl_main), true);'
  void slide(lvgl::LvglComponent *lv, bool forward, uint32_t duration_ms = 300);

 protected:
  struct Rect {
    int16_t x, y, w, h;
  };
  // past this many areas in a frame, mirror the whole canvas instead of tracking areas
  static constexpr size_t MAX_RECTS = 32;

  void enable_canvas_mode_();
  void add_rect_(int x, int y, int w, int h);
  void copy_rect_(uint16_t *dst, const uint16_t *src, const Rect &r);
  void mirror_and_present_();          // canvas mode: copy dirty areas into hidden buffer, swap
  void swap_to_(uint16_t *fb);         // hand fb to the driver and wait until the old one is free
  void present_copy_mode_();           // before canvas mode: fb-local double buffering
  uint16_t *hidden_() { return this->shown_ == this->fb_[0] ? this->fb_[1] : this->fb_[0]; }

  uint16_t *fb_[2]{};
  uint16_t *shown_{nullptr};           // display buffer currently on screen
  uint16_t *canvas_{nullptr};          // LVGL's only draw buffer (canvas mode)
  uint16_t *old_img_{nullptr};         // slide(): snapshot of the outgoing page
  std::vector<uint16_t> span_lo_;      // first visible column per row (round panel)
  bool canvas_mode_{false};
  bool capture_{false};                // slide(): LVGL is rendering the new page; don't present it
  HighFrequencyLoopRequester high_freq_;
  SemaphoreHandle_t frame_done_{nullptr};

  Rect prev_[MAX_RECTS]{};
  size_t prev_count_{0};
  bool prev_full_{false};
  Rect cur_[MAX_RECTS]{};
  size_t cur_count_{0};
  bool cur_full_{false};
  bool frame_open_{false};

  bool log_fps_{false};
  uint32_t frames_{0}, render_us_{0}, wait_us_{0}, worst_render_us_{0}, last_log_ms_{0};

 public:
  uint32_t frame_start_us_{0};         // set from LVGL's REFR_START event
};

}  // namespace esphome::crowpanel_rgb
#endif  // USE_ESP32_VARIANT_ESP32S3
