#ifdef USE_ESP32_VARIANT_ESP32S3
#include "crowpanel_rgb.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "lvgl.h"
#include <driver/gpio.h>
#include <algorithm>
#include <cmath>
#include <cstring>

// LVGL's RGB565 swap runs on the flush pointer for the area's pixel count; in direct mode that pointer is
// the canvas origin, so it scrambles the top of the canvas every frame.
#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
#error "crowpanel_rgb needs `byte_order: little_endian` in the lvgl: config"
#endif

namespace esphome::crowpanel_rgb {

static const char *const TAG = "display.crowpanel_rgb";

static bool IRAM_ATTR on_frame_buf_complete(esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t *,
                                            void *ctx) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(ctx), &woken);
  return woken == pdTRUE;
}

void CrowpanelRGB::setup() {
  this->spi_setup();
  this->write_init_sequence_();

  esp_lcd_rgb_panel_config_t config{};
  config.flags.fb_in_psram = 1;
  config.num_fbs = 2;
  config.bounce_buffer_size_px = this->width_ * 40;
  config.timings.h_res = this->width_;
  config.timings.v_res = this->height_;
  config.timings.hsync_pulse_width = this->hsync_pulse_width_;
  config.timings.hsync_back_porch = this->hsync_back_porch_;
  config.timings.hsync_front_porch = this->hsync_front_porch_;
  config.timings.vsync_pulse_width = this->vsync_pulse_width_;
  config.timings.vsync_back_porch = this->vsync_back_porch_;
  config.timings.vsync_front_porch = this->vsync_front_porch_;
  config.timings.flags.pclk_active_neg = this->pclk_inverted_;
  config.timings.pclk_hz = this->pclk_frequency_;
  config.clk_src = LCD_CLK_SRC_PLL160M;
  size_t data_pin_count = sizeof(this->data_pins_) / sizeof(this->data_pins_[0]);
  // st7701s's codegen swaps the pin bytes for big-endian pixels; LVGL renders native (little-endian)
  // here, so undo it (assumes red/green/blue data_pins, which is what codegen swaps).
  for (size_t i = 0; i != data_pin_count; i++)
    config.data_gpio_nums[i] = static_cast<gpio_num_t>(this->data_pins_[(i + 8) % data_pin_count]->get_pin());
  config.data_width = data_pin_count;
  config.disp_gpio_num = GPIO_NUM_NC;
  config.hsync_gpio_num = static_cast<gpio_num_t>(this->hsync_pin_->get_pin());
  config.vsync_gpio_num = static_cast<gpio_num_t>(this->vsync_pin_->get_pin());
  config.de_gpio_num = static_cast<gpio_num_t>(this->de_pin_->get_pin());
  config.pclk_gpio_num = static_cast<gpio_num_t>(this->pclk_pin_->get_pin());

  esp_err_t err = esp_lcd_new_rgb_panel(&config, &this->handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  this->frame_done_ = xSemaphoreCreateBinary();
  esp_lcd_rgb_panel_event_callbacks_t cbs{};
  cbs.on_frame_buf_complete = on_frame_buf_complete;
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(this->handle_, &cbs, this->frame_done_));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(this->handle_));
  ESP_ERROR_CHECK(esp_lcd_panel_init(this->handle_));
  void *fb0 = nullptr, *fb1 = nullptr;
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(this->handle_, 2, &fb0, &fb1));
  this->fb_[0] = static_cast<uint16_t *>(fb0);
  this->fb_[1] = static_cast<uint16_t *>(fb1);
  this->shown_ = this->fb_[0];  // the driver scans fb 0 first

  this->span_lo_.resize(this->height_);
  const float r = this->width_ / 2.0f;
  for (size_t y = 0; y < this->height_; y++) {
    float dy = y + 0.5f - this->height_ / 2.0f;
    float half = dy * dy < r * r ? sqrtf(r * r - dy * dy) : 0.0f;
    this->span_lo_[y] = (uint16_t) std::max(0.0f, floorf(r - half - 1));  // 1px margin: AA edge
  }
}

// No esp_lcd_rgb_panel_restart() every loop (the st7701s band-aid for drift).
void CrowpanelRGB::loop() {
  if (!this->canvas_mode_ && this->fb_[0] != nullptr && lv_display_get_default() != nullptr)
    this->enable_canvas_mode_();
  if (!this->log_fps_)
    return;
  uint32_t now = millis();
  if (now - this->last_log_ms_ < 5000)
    return;
  float secs = (now - this->last_log_ms_) / 1000.0f;
  uint32_t f = this->frames_ ? this->frames_ : 1;
  ESP_LOGI(TAG, "fps=%.1f render=%ums (worst %ums) present=%ums", this->frames_ / secs,
           (unsigned) (this->render_us_ / f / 1000), (unsigned) (this->worst_render_us_ / 1000),
           (unsigned) (this->wait_us_ / f / 1000));
  this->frames_ = this->render_us_ = this->wait_us_ = this->worst_render_us_ = 0;
  this->last_log_ms_ = now;
}

// Give LVGL a single canvas (never displayed). Runs from loop(), between LVGL refreshes.
void CrowpanelRGB::enable_canvas_mode_() {
  this->canvas_mode_ = true;  // one attempt; on failure draw_pixels_at keeps using copy mode
  const size_t bytes = this->width_ * this->height_ * sizeof(uint16_t);
  uint16_t *canvas = static_cast<uint16_t *>(heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_SPIRAM));
  uint16_t *old_img = static_cast<uint16_t *>(heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_SPIRAM));
  if (canvas == nullptr || old_img == nullptr) {
    ESP_LOGE(TAG, "no PSRAM for the canvas; staying in copy mode");
    heap_caps_free(canvas);
    heap_caps_free(old_img);
    return;
  }
  memcpy(canvas, this->shown_, bytes);
  this->canvas_ = canvas;
  this->old_img_ = old_img;
  lv_display_t *disp = lv_display_get_default();
  lv_display_set_buffers(disp, this->canvas_, nullptr, bytes, LV_DISPLAY_RENDER_MODE_DIRECT);
  lv_display_add_event_cb(
      disp, [](lv_event_t *e) { static_cast<CrowpanelRGB *>(lv_event_get_user_data(e))->frame_start_us_ = micros(); },
      LV_EVENT_REFR_START, this);
  // Let the panel set the pace (ESPHome: LVGL refresh every 16ms, main loop ~16ms).
  lv_timer_set_period(lv_display_get_refr_timer(disp), 2);
  this->high_freq_.start();
  this->cur_count_ = this->prev_count_ = 0;
  this->cur_full_ = false;
  this->prev_full_ = true;  // mirror everything once
  lv_obj_invalidate(lv_screen_active());
  ESP_LOGI(TAG, "LVGL renders into a canvas mirrored to two display buffers");
}

void CrowpanelRGB::add_rect_(int x, int y, int w, int h) {
  if (this->cur_count_ < MAX_RECTS) {
    this->cur_[this->cur_count_++] = {(int16_t) x, (int16_t) y, (int16_t) w, (int16_t) h};
  } else {
    this->cur_full_ = true;
  }
}

void CrowpanelRGB::copy_rect_(uint16_t *dst, const uint16_t *src, const Rect &r) {
  for (int y = r.y; y < r.y + r.h; y++)
    memcpy(dst + y * this->width_ + r.x, src + y * this->width_ + r.x, r.w * sizeof(uint16_t));
}

void CrowpanelRGB::swap_to_(uint16_t *fb) {
  uint32_t t = micros();
  // Passing one of the driver's own framebuffers just switches which one it scans out next frame.
  esp_lcd_panel_draw_bitmap(this->handle_, 0, 0, this->width_, this->height_, fb);
  // Clear after requesting the swap, then wait for a whole frame to finish reading: the next frame
  // comes from fb, so the old buffer is free. A completion in between costs one extra frame of
  // waiting, never a tear.
  xSemaphoreTake(this->frame_done_, 0);
  xSemaphoreTake(this->frame_done_, pdMS_TO_TICKS(60));
  this->wait_us_ += micros() - t;
  this->shown_ = fb;
  this->frames_++;
}

// The hidden buffer last matched the canvas two frames ago: bring over last frame's and this
// frame's changed areas, then show it.
void CrowpanelRGB::mirror_and_present_() {
  uint32_t render = micros() - this->frame_start_us_;
  this->render_us_ += render;
  if (render > this->worst_render_us_)
    this->worst_render_us_ = render;
  uint16_t *dst = this->hidden_();
  if (this->prev_full_ || this->cur_full_) {
    memcpy(dst, this->canvas_, this->width_ * this->height_ * sizeof(uint16_t));
  } else {
    for (size_t i = 0; i != this->prev_count_; i++)
      this->copy_rect_(dst, this->canvas_, this->prev_[i]);
    for (size_t i = 0; i != this->cur_count_; i++)
      this->copy_rect_(dst, this->canvas_, this->cur_[i]);
  }
  this->swap_to_(dst);
  memcpy(this->prev_, this->cur_, sizeof(Rect) * this->cur_count_);
  this->prev_count_ = this->cur_count_;
  this->prev_full_ = this->cur_full_;
  this->cur_count_ = 0;
  this->cur_full_ = false;
}

void CrowpanelRGB::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                                  display::ColorOrder order, display::ColorBitness bitness, bool big_endian,
                                  int x_offset, int y_offset, int x_pad) {
  if (w <= 0 || h <= 0)
    return;
  if (bitness != display::COLOR_BITNESS_565 || this->fb_[0] == nullptr) {
    display::Display::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset, y_offset,
                                     x_pad);
    return;
  }
  lv_display_t *disp = lv_display_get_default();
  bool last = disp == nullptr || lv_display_flush_is_last(disp);

  // Canvas mode: LVGL already drew into the canvas; just note the area.
  if (this->canvas_ != nullptr && ptr == reinterpret_cast<const uint8_t *>(this->canvas_)) {
    this->add_rect_(x_start, y_start, w, h);
    if (last && !this->capture_)
      this->mirror_and_present_();
    return;
  }

  // Copy mode (boot only, until LVGL's display exists): start the hidden buffer from the shown
  // one, write the areas into it, swap on the last flush.
  x_start += this->offset_x_;
  y_start += this->offset_y_;
  int x0 = std::max(x_start, 0), y0 = std::max(y_start, 0);
  int x1 = std::min<int>(x_start + w, this->width_), y1 = std::min<int>(y_start + h, this->height_);
  if (x0 >= x1 || y0 >= y1)
    return;
  uint16_t *dst = this->hidden_();
  if (!this->frame_open_) {
    this->frame_open_ = true;
    memcpy(dst, this->shown_, this->width_ * this->height_ * sizeof(uint16_t));
  }
  const int stride = x_offset + w + x_pad;
  const uint16_t *src = reinterpret_cast<const uint16_t *>(ptr);
  for (int y = y0; y < y1; y++)
    memcpy(dst + y * this->width_ + x0, src + (y - y_start + y_offset) * stride + x_offset + (x0 - x_start),
           (x1 - x0) * sizeof(uint16_t));
  if (last) {
    this->swap_to_(dst);
    this->frame_open_ = false;
  }
}

void CrowpanelRGB::slide(lvgl::LvglComponent *lv, bool forward, uint32_t duration_ms) {
  auto change_page = [&]() {
    forward ? lv->show_next_page(LV_SCREEN_LOAD_ANIM_NONE, 0) : lv->show_prev_page(LV_SCREEN_LOAD_ANIM_NONE, 0);
  };
  if (this->canvas_ == nullptr) {
    change_page();  // not ready yet: plain page change
    return;
  }
  const size_t w = this->width_, h = this->height_, bytes = w * h * sizeof(uint16_t);
  uint32_t t_start = millis();
  memcpy(this->old_img_, this->canvas_, bytes);  // the canvas is the current picture

  // Render the new page into the canvas without presenting it.
  this->capture_ = true;
  change_page();
  lv_refr_now(nullptr);
  this->capture_ = false;
  uint32_t t_prep = millis();

  // Animate: each frame is two row copies (visible span only) into the hidden buffer, then a swap.
  const uint16_t *a = this->old_img_, *b = this->canvas_;
  uint32_t t0 = millis(), n = 0;
  for (;;) {
    float p = std::min(1.0f, (millis() - t0) / (float) duration_ms);
    float e = 1.0f - (1.0f - p) * (1.0f - p) * (1.0f - p);  // ease-out cubic
    size_t x = std::min<size_t>(w, (size_t) (e * w + 0.5f));
    uint16_t *dst = this->hidden_();
    for (size_t y = 0; y < h; y++) {
      size_t lo = this->span_lo_[y], hi = w - lo;
      if (lo >= hi)
        continue;
      uint16_t *row = dst + y * w;
      const uint16_t *ar = a + y * w, *br = b + y * w;
      // forward: column c shows old[c + x] for c < w - x, else new[c - (w - x)]; mirrored backward
      size_t split = forward ? w - x : x;
      const uint16_t *left = forward ? ar + x : br + (w - x);   // source for columns [0, split)
      const uint16_t *right = forward ? br - (w - x) : ar - x;  // source for columns [split, w)
      size_t l1 = std::min(hi, split);
      if (lo < l1)
        memcpy(row + lo, left + lo, (l1 - lo) * sizeof(uint16_t));
      size_t r0 = std::max(lo, split);
      if (r0 < hi)
        memcpy(row + r0, right + r0, (hi - r0) * sizeof(uint16_t));
    }
    this->swap_to_(dst);
    n++;
    App.feed_wdt();
    if (p >= 1.0f)
      break;
  }
  // The shown buffer now equals the canvas (final frame); mirror it into the other one too.
  memcpy(this->hidden_(), this->canvas_, bytes);
  this->cur_count_ = this->prev_count_ = 0;
  this->cur_full_ = this->prev_full_ = false;
  if (this->log_fps_) {
    uint32_t anim = millis() - t0;
    ESP_LOGI(TAG, "slide: prep %ums, %u frames in %ums = %.1f fps", (unsigned) (t_prep - t_start), (unsigned) n,
             (unsigned) anim, anim ? n * 1000.0f / anim : 0.0f);
  }
}

void CrowpanelRGB::dump_config() {
  st7701s::ST7701S::dump_config();
  ESP_LOGCONFIG(TAG,
                "  Pipeline: LVGL canvas mirrored to 2 framebuffers, swap on frame-complete\n"
                "  PCLK: %u Hz, inverted: %s",
                (unsigned) this->pclk_frequency_, YESNO(this->pclk_inverted_));
}

}  // namespace esphome::crowpanel_rgb
#endif  // USE_ESP32_VARIANT_ESP32S3
