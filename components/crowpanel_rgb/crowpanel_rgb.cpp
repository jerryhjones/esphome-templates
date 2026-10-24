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
  // Effects keep moving when LVGL has nothing to redraw (one present per loop at most).
  if (this->canvas_ != nullptr && !this->presented_ && this->effects_active_()) {
    this->frame_start_us_ = micros();
    this->mirror_and_present_();
  }
  this->presented_ = false;
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
  this->draw_effects_(dst);
  this->swap_to_(dst);
  this->presented_ = true;
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

// Snapshot the current picture into old_img_, apply `change`, and render the result into the canvas
// without presenting it. False (change applied plainly) before canvas mode is up.
bool CrowpanelRGB::begin_transition_(const std::function<void()> &change) {
  if (this->canvas_ == nullptr) {
    change();
    return false;
  }
  memcpy(this->old_img_, this->canvas_, this->width_ * this->height_ * sizeof(uint16_t));
  this->capture_ = true;
  change();
  lv_refr_now(nullptr);
  this->capture_ = false;
  return true;
}

// The shown buffer now equals the canvas (final frame); mirror it into the other one too.
void CrowpanelRGB::end_transition_(const char *name, uint32_t t0, uint32_t frames) {
  memcpy(this->hidden_(), this->canvas_, this->width_ * this->height_ * sizeof(uint16_t));
  this->pbox_count_[0] = this->pbox_count_[1] = 0;
  this->cur_count_ = this->prev_count_ = 0;
  this->cur_full_ = this->prev_full_ = false;
  if (this->log_fps_) {
    uint32_t anim = millis() - t0;
    ESP_LOGI(TAG, "%s: %u frames in %ums = %.1f fps", name, (unsigned) frames, (unsigned) anim,
             anim ? frames * 1000.0f / anim : 0.0f);
  }
}

void CrowpanelRGB::slide(lvgl::LvglComponent *lv, bool forward, uint32_t duration_ms) {
  if (!this->begin_transition_([&]() {
        forward ? lv->show_next_page(LV_SCREEN_LOAD_ANIM_NONE, 0) : lv->show_prev_page(LV_SCREEN_LOAD_ANIM_NONE, 0);
      }))
    return;
  const size_t w = this->width_, h = this->height_;

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
  this->end_transition_("slide", t0, n);
}

void CrowpanelRGB::iris(bool open, const std::function<void()> &change, uint32_t edge_color, uint32_t duration_ms) {
  if (!this->begin_transition_(change))
    return;
  // open: the new picture inside a growing circle; close: the old picture inside a shrinking one
  const uint16_t *in = open ? this->canvas_ : this->old_img_, *out = open ? this->old_img_ : this->canvas_;
  const uint16_t edge = ((edge_color >> 8) & 0xF800) | ((edge_color >> 5) & 0x07E0) | ((edge_color >> 3) & 0x001F);
  const int w = this->width_, h = this->height_, ring = 4;
  const float cx = w / 2.0f, cy = h / 2.0f, rmax = cx + ring + 3;  // final open frame: ring fully past the visible rim
  uint32_t t0 = millis(), n = 0;
  for (;;) {
    float p = std::min(1.0f, (millis() - t0) / (float) duration_ms);
    float e = open ? 1.0f - (1.0f - p) * (1.0f - p) * (1.0f - p) : p * p * p;  // fast near the rim
    float r = (open ? e : 1.0f - e) * rmax, ri = std::max(0.0f, r - ring);
    uint16_t *dst = this->hidden_();
    for (int y = 0; y < h; y++) {
      int lo = this->span_lo_[y], hi = w - lo;
      if (lo >= hi)
        continue;
      uint16_t *row = dst + y * w;
      float dy = y + 0.5f - cy;
      // columns: out [lo,a) edge [a,b) in [b,c) edge [c,d) out [d,hi)
      float ho = r * r > dy * dy ? sqrtf(r * r - dy * dy) : -1.0f;
      float hi_ = ri * ri > dy * dy ? sqrtf(ri * ri - dy * dy) : -1.0f;
      auto clampx = [&](float x) { return std::min(hi, std::max(lo, (int) (x + 0.5f))); };
      int a = ho < 0 ? hi : clampx(cx - ho), d = ho < 0 ? hi : clampx(cx + ho);
      int b = hi_ < 0 ? a : clampx(cx - hi_), cc = hi_ < 0 ? a : clampx(cx + hi_);
      if (hi_ < 0)  // inside the ring band only: all edge
        b = cc = d;
      const uint16_t *orow = out + y * w, *irow = in + y * w;
      memcpy(row + lo, orow + lo, (a - lo) * 2);
      std::fill(row + a, row + b, edge);
      memcpy(row + b, irow + b, (cc - b) * 2);
      std::fill(row + cc, row + d, edge);
      if (d < hi)
        memcpy(row + d, orow + d, (hi - d) * 2);
    }
    this->swap_to_(dst);
    n++;
    App.feed_wdt();
    if (p >= 1.0f)
      break;
  }
  this->end_transition_(open ? "iris open" : "iris close", t0, n);
}

void CrowpanelRGB::dump_config() {
  st7701s::ST7701S::dump_config();
  ESP_LOGCONFIG(TAG,
                "  Pipeline: LVGL canvas mirrored to 2 framebuffers, swap on frame-complete\n"
                "  PCLK: %u Hz, inverted: %s",
                (unsigned) this->pclk_frequency_, YESNO(this->pclk_inverted_));
}

// ---- Framebuffer effects: drawn over the canvas into each frame after LVGL, never into the canvas,
// so every effect pixel is canvas-under-it blended with colour and erases itself next time.

static inline uint16_t blend565(uint16_t bg, uint16_t fg, uint32_t a /* 0..32 */) {
  uint32_t b = (bg | (bg << 16)) & 0x07E0F81F, f = (fg | (fg << 16)) & 0x07E0F81F;
  uint32_t r = (b + (((f - b) * a) >> 5)) & 0x07E0F81F;
  return (uint16_t) (r | (r >> 16));
}
static inline uint16_t rgb565(uint32_t c) {
  return ((c >> 8) & 0xF800) | ((c >> 5) & 0x07E0) | ((c >> 3) & 0x001F);
}

bool CrowpanelRGB::effects_active_() {
  return this->comet_on_ || this->comet_clean_ > 0 || this->particles_alive_ > 0 || this->pbox_count_[0] > 0 ||
         this->pbox_count_[1] > 0;
}

void CrowpanelRGB::comet(bool on, uint32_t color, uint32_t period_ms) {
  if (on && this->ring_px_ == nullptr) {
    // every pixel of the rim band, with its angle and radius (for antialiased edges); ~170KB, PSRAM
    const float cx = this->width_ / 2.0f, cy = this->height_ / 2.0f;
    for (int pass = 0; pass < 2; pass++) {
      size_t n = 0;
      for (int y = 0; y < (int) this->height_; y++)
        for (int x = 0; x < (int) this->width_; x++) {
          float dx = x + 0.5f - cx, dy = y + 0.5f - cy, r = sqrtf(dx * dx + dy * dy);
          if (r < COMET_R - COMET_W - 1 || r > COMET_R + 1)
            continue;
          if (pass == 1) {
            float ang = atan2f(dx, -dy);  // 0 at 12 o'clock, clockwise
            if (ang < 0)
              ang += 2 * (float) M_PI;
            this->ring_px_[n] = {(uint32_t) (y * this->width_ + x), (uint16_t) (ang / (2 * (float) M_PI) * 65536.0f),
                                 (uint16_t) (r * 16)};
          }
          n++;
        }
      if (pass == 0) {
        this->ring_px_ = static_cast<RingPx *>(heap_caps_malloc(n * sizeof(RingPx), MALLOC_CAP_SPIRAM));
        if (this->ring_px_ == nullptr)
          return;
        this->ring_n_ = n;
      }
    }
  }
  if (on) {
    this->comet_color_ = rgb565(color);
    this->comet_period_ms_ = std::max<uint32_t>(period_ms, 100);
    if (!this->comet_on_)
      this->comet_t0_ = millis();
  } else if (this->comet_on_) {
    this->comet_clean_ = 2;  // restore the band in both buffers
  }
  this->comet_on_ = on;
}

void CrowpanelRGB::burst(int x, int y, int count) {
  static const uint32_t PALETTE[] = {0x3DDC84, 0xFFB020, 0xFF4D5E, 0x5BA8FF, 0xFFFFFF, 0xB98CFF};
  count = std::min<int>(count, MAX_PARTICLES);
  for (int i = 0; i < count; i++) {
    Particle &p = this->particles_[i];
    float ang = (i + (random_uint32() % 100) / 100.0f) * 2 * (float) M_PI / count;
    float speed = 260 + random_uint32() % 220;  // px/s
    p = {(float) x, (float) y, cosf(ang) * speed, sinf(ang) * speed - 120, 0, 0.9f + (random_uint32() % 50) / 100.0f,
         4.0f + (random_uint32() % 30) / 10.0f, rgb565(PALETTE[i % 6])};
  }
  this->particles_alive_ = count;
  this->particles_last_ms_ = millis();
}

void CrowpanelRGB::draw_effects_(uint16_t *dst) {
  const int w = this->width_, h = this->height_;
  const int bi = dst == this->fb_[0] ? 0 : 1;
  // Restore what particles left in this buffer last time.
  for (int i = 0; i < this->pbox_count_[bi]; i++)
    this->copy_rect_(dst, this->canvas_, this->pbox_[bi][i]);
  this->pbox_count_[bi] = 0;

  if (this->comet_on_ || this->comet_clean_ > 0) {
    if (!this->comet_on_) {
      for (const RingPx &px : this->ring())
        dst[px.off] = this->canvas_[px.off];
      this->comet_clean_--;
    } else {
      const uint32_t head = (uint32_t) (((millis() - this->comet_t0_) % this->comet_period_ms_) * 65536ULL /
                                        this->comet_period_ms_);
      const float tail = COMET_TAIL_DEG / 360.0f * 65536.0f, outer = COMET_R;
      const float px_per_unit = COMET_R * 2 * (float) M_PI / 65536.0f;
      for (const RingPx &px : this->ring()) {
        uint16_t bg = this->canvas_[px.off];
        uint32_t d = (head - px.ang) & 0xFFFF;  // how far behind the head
        float t = d / tail;
        if (d > 0x8000)  // ahead of the head: only the rounded tip
          t = -(float) (0x10000 - d) / tail;
        float tip = t < 0 ? -t * tail * px_per_unit : 0;  // px ahead of the head
        if (t >= 1.0f || tip > COMET_W / 2) {
          dst[px.off] = bg;
          continue;
        }
        float r = px.r16 / 16.0f;
        float tt = std::max(0.0f, t), thick = COMET_W * (1.0f - 0.65f * tt);
        float half = thick / 2;
        // distance from the band centre line, rounded at the tip
        float dr = fabsf(r - (outer - half)), dist = tip > 0 ? sqrtf(dr * dr + tip * tip) : dr;
        float cov = std::min(1.0f, std::max(0.0f, half - dist + 0.5f));
        float fade = (1.0f - tt) * (1.0f - tt);
        uint32_t a = (uint32_t) (cov * fade * 32.0f + 0.5f);
        dst[px.off] = a ? blend565(bg, this->comet_color_, a) : bg;
      }
    }
  }

  if (this->particles_alive_ > 0) {
    uint32_t now = millis();
    float dt = std::min(0.05f, (now - this->particles_last_ms_) / 1000.0f);
    this->particles_last_ms_ = now;
    int alive = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
      Particle &p = this->particles_[i];
      if (p.age >= p.life)
        continue;
      p.age += dt;
      p.vx *= 1.0f - 2.2f * dt;  // drag
      p.vy = p.vy * (1.0f - 2.2f * dt) + 520 * dt;  // gravity
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      if (p.age >= p.life)
        continue;
      alive++;
      float k = p.age / p.life, rad = p.size * (k < 0.6f ? 1.0f : 1.0f - (k - 0.6f) / 0.4f * 0.6f);
      float opa = k < 0.6f ? 1.0f : 1.0f - (k - 0.6f) / 0.4f;
      int x0 = std::max(0, (int) (p.x - rad - 1)), x1 = std::min(w, (int) (p.x + rad + 2));
      int y0 = std::max(0, (int) (p.y - rad - 1)), y1 = std::min(h, (int) (p.y + rad + 2));
      if (x0 >= x1 || y0 >= y1)
        continue;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) {
          float dx = x + 0.5f - p.x, dy = y + 0.5f - p.y;
          float cov = std::min(1.0f, std::max(0.0f, rad - sqrtf(dx * dx + dy * dy) + 0.5f));
          uint32_t a = (uint32_t) (cov * opa * 32.0f + 0.5f);
          if (a)
            dst[y * w + x] = blend565(this->canvas_[y * w + x], p.color, a);
        }
      if (this->pbox_count_[bi] < MAX_PARTICLES)
        this->pbox_[bi][this->pbox_count_[bi]++] = {(int16_t) x0, (int16_t) y0, (int16_t) (x1 - x0),
                                                    (int16_t) (y1 - y0)};
    }
    this->particles_alive_ = alive;
  }
}

}  // namespace esphome::crowpanel_rgb
#endif  // USE_ESP32_VARIANT_ESP32S3
