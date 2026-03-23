/**
 * sonoff_ms01.cpp — Sonoff MS01 Soil Moisture Sensor driver (implementation)
 *
 * See sonoff_ms01.h for full protocol and design documentation.
 *
 */

#include "sonoff_ms01.h"
#include "esphome/core/log.h"
#include "esp_rom_sys.h"  // esp_rom_delay_us

static const char *const TAG = "sonoff_ms01";

// ════════════════════════════════════════════════════════════════════════════
// ESP-IDF build path
// ════════════════════════════════════════════════════════════════════════════
#ifdef USE_ESP_IDF

namespace esphome {
namespace sonoff_ms01 {

// ── setup() ─────────────────────────────────────────────────────────────────
void SonoffMS01Component::setup() {
  const gpio_num_t gpio = static_cast<gpio_num_t>(pin_->get_pin());

  // Configure the pad as open-drain input+output and leave it there forever.
  //
  // WHY OPEN-DRAIN INPUT+OUTPUT (and why we never call pin_mode() again):
  //
  // When rmt_new_rx_channel() runs later in update(), it calls gpio_matrix_in()
  // internally to route this pad's signal to the RMT RX peripheral input.
  // If we subsequently call gpio_set_direction(GPIO_MODE_OUTPUT), the IDF
  // GPIO driver overwrites the IO_MUX/GPIO matrix configuration and silently
  // disconnects the RMT peripheral's input path — the RMT becomes blind and
  // every read times out.
  //
  // The solution: set GPIO_MODE_INPUT_OUTPUT_OD once here.  This keeps:
  //   • input path ENABLED  → RMT can see the signal at all times
  //   • output driver ENABLED in open-drain mode → we can pull the line LOW
  //     by writing 0, and release it (sensor can pull low) by writing 1
  //
  // For the start pulse we then use gpio_set_level() only — no direction
  // change, no disturbance to the GPIO matrix routing that RMT set up.
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask  = 1ULL << gpio;
  io_conf.mode          = GPIO_MODE_INPUT_OUTPUT_OD;
  io_conf.pull_up_en    = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type     = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

  // Line idles HIGH (open-drain + pullup; sensor also has internal pullup)
  gpio_set_level(gpio, 1);

  ESP_LOGCONFIG(TAG, "Sonoff MS01 on GPIO%d, poll every %.0f s",
                gpio,
                static_cast<double>(get_update_interval()) / 1000.0);
}

// ── update() ────────────────────────────────────────────────────────────────
// Called by ESPHome at the configured update_interval.
// Returns immediately after arming the hardware; loop() does the rest.
void SonoffMS01Component::update() {
  if (state_.load(std::memory_order_relaxed) != State::IDLE) {
    // Shouldn't happen at typical poll rates, but defend against it.
    ESP_LOGW(TAG, "Previous read still in progress — skipping this cycle");
    return;
  }

  // ── 1. Allocate RMT channel ──────────────────────────────────────────────
  if (!allocate_rmt_()) {
    return;  // Warning already logged inside allocate_rmt_()
  }

  // ── 2. Arm the RMT receiver BEFORE sending the start pulse ──────────────
  //
  // RMT receive configuration.
  //
  // signal_range_min_ns: discard glitches shorter than 1 µs.
  //
  // signal_range_max_ns: the RMT treats any signal lasting longer than this
  //   as an end-of-frame marker and stops capturing.  This MUST be greater
  //   than the longest legitimate signal in the entire frame — including our
  //   own 450 µs start pulse which the RMT sees via the GPIO matrix.
  //   Setting it to 200 µs (as originally written) caused the RMT to stop
  //   immediately after the start pulse, capturing exactly 1 symbol.
  //
  //   Longest pulses in the frame:
  //     Start pulse (our bit-bang) : 450 µs LOW
  //     Sensor ACK                 :  80 µs LOW / 80 µs HIGH
  //     Data bit '1'               :  70 µs HIGH
  //   → minimum safe value is ~500 µs; we use 1 ms (1,000,000 ns).
  //
  //   End-of-frame is detected by the post-data line-HIGH: after the last
  //   data bit the sensor releases the bus and the pullup holds it HIGH
  //   indefinitely, which always exceeds 1 ms.
  rmt_receive_config_t rx_conf{};
  rx_conf.signal_range_min_ns = 1'000;       //   1 µs glitch filter
  rx_conf.signal_range_max_ns = 1'000'000;   //   1 ms end-of-frame threshold

  esp_err_t err = rmt_receive(rx_channel_, symbols_, sizeof(symbols_), &rx_conf);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "rmt_receive() failed: %s — aborting read", esp_err_to_name(err));
    release_rmt_();
    return;
  }

  // ── 3. Bit-bang the start pulse ──────────────────────────────────────────
  //
  // Drive the GPIO LOW for 450 µs, then release by writing HIGH.
  // We use gpio_set_level() directly — NOT pin_->pin_mode() / digital_write().
  //
  // The pad is already GPIO_MODE_INPUT_OUTPUT_OD from setup(), so:
  //   gpio_set_level(gpio, 0) → open-drain pulls line LOW  (start pulse)
  //   gpio_set_level(gpio, 1) → open-drain releases line   (pullup brings HIGH)
  //
  // Critically, the GPIO matrix input path to the RMT peripheral remains
  // connected throughout — the RMT sees the entire waveform including our
  // own start pulse as the first captured symbol.
  const gpio_num_t gpio = static_cast<gpio_num_t>(pin_->get_pin());
  gpio_set_level(gpio, 0);
  esp_rom_delay_us(450);
  gpio_set_level(gpio, 1);  // release — sensor detects rising edge ~30 µs later

  // Full frame: 450µs start + ~5ms data + 1ms end-of-frame threshold = ~7ms.
  // 20ms gives comfortable margin.
  state_.store(State::WAITING, std::memory_order_relaxed);
  deadline_ms_ = millis() + 20;
}

// ── loop() ──────────────────────────────────────────────────────────────────
// Runs on every ESPHome main-loop tick.  When the ISR signals completion,
// this method decodes the symbols and publishes the sensor values.
void SonoffMS01Component::loop() {
  switch (state_.load(std::memory_order_relaxed)) {
    case State::IDLE:
      return;  // Nothing to do

    case State::WAITING:
      if (millis() > deadline_ms_) {
        // Log num_symbols_ to distinguish "RMT saw nothing" (0) from
        // "RMT captured symbols but ISR never fired" (>0, shouldn't happen).
        ESP_LOGW(TAG, "Timeout — no response from MS01 (symbols captured before timeout: %u)",
                 (unsigned) num_symbols_);
        release_rmt_();
        state_.store(State::IDLE, std::memory_order_relaxed);
      }
      // Otherwise keep waiting; the ISR will flip state to DONE
      return;

    case State::DONE:
      // ISR has written num_symbols_ and set state = DONE.
      // We are now back in main-loop context: safe to call publish_state(),
      // allocate memory, use FreeRTOS blocking primitives, etc.
      release_rmt_();       // Return channel to pool immediately
      decode_and_publish_();
      state_.store(State::IDLE, std::memory_order_relaxed);
      return;
  }
}

// ── ISR callback ─────────────────────────────────────────────────────────────
// Placed in IRAM so it can execute even if flash cache is disabled
// (e.g., during OTA or SPI flash operations).
bool IRAM_ATTR SonoffMS01Component::rmt_rx_done_cb_(
    rmt_channel_handle_t            /* channel — unused */,
    const rmt_rx_done_event_data_t *edata,
    void                           *user_ctx)
{
  auto *self = static_cast<SonoffMS01Component *>(user_ctx);

  // edata->received_symbols points directly into our symbols_[] buffer —
  // no copy is necessary.  Just record the count and signal loop().
  self->num_symbols_ = edata->num_symbols;
  self->state_.store(State::DONE, std::memory_order_relaxed);

  // Return false: we did not wake a higher-priority FreeRTOS task,
  // so the scheduler need not yield at ISR exit.
  return false;
}

// ── allocate_rmt_() ──────────────────────────────────────────────────────────
bool SonoffMS01Component::allocate_rmt_() {
  rmt_rx_channel_config_t rx_cfg{};
  rx_cfg.gpio_num          = static_cast<gpio_num_t>(pin_->get_pin());
  rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;  // typically APB @ 80 MHz
  rx_cfg.resolution_hz     = 1'000'000;             // 1 tick = 1 µs
  // One RMT memory block (64 symbols on ESP32 original).
  // We expect at most ~43 symbols per frame, so 64 is sufficient.
  rx_cfg.mem_block_symbols = 64;

  esp_err_t err = rmt_new_rx_channel(&rx_cfg, &rx_channel_);
  if (err != ESP_OK) {
    // ESP_ERR_NOT_FOUND means all RMT channels are in use by other components.
    // This is recoverable — the next update() call will retry.
    ESP_LOGW(TAG, "Could not allocate RMT RX channel: %s "
                  "(another component may be holding all channels)",
             esp_err_to_name(err));
    rx_channel_ = nullptr;
    return false;
  }

  rmt_rx_event_callbacks_t cbs{};
  cbs.on_recv_done = &SonoffMS01Component::rmt_rx_done_cb_;
  rmt_rx_register_event_callbacks(rx_channel_, &cbs, this);

  rmt_enable(rx_channel_);
  return true;
}

// ── release_rmt_() ───────────────────────────────────────────────────────────
void SonoffMS01Component::release_rmt_() {
  if (rx_channel_ == nullptr)
    return;
  // IDF requires the channel to be disabled before deletion.
  rmt_disable(rx_channel_);
  rmt_del_channel(rx_channel_);
  rx_channel_ = nullptr;
}

// ── decode_and_publish_() ─────────────────────────────────────────────────────
void SonoffMS01Component::decode_and_publish_() {
  const size_t n = num_symbols_;

  // ── Raw symbol dump (logged at DEBUG level) ───────────────────────────────
  // Each RMT symbol is two pairs of (duration µs, level).
  // Expected layout:
  //   sym[0]        : our start pulse       ~{450 LOW, <30 HIGH}
  //   sym[1]        : sensor ACK            ~{80  LOW, 80  HIGH}
  //   sym[2..41]    : 40 data bits          ~{50  LOW, 26  HIGH=0 / 70 HIGH=1}
  //   sym[42]       : end pulse (optional)  ~{50  LOW, >>1000 HIGH → RMT stop}
  ESP_LOGD(TAG, "RMT captured %u symbols:", n);
  for (size_t i = 0; i < n; i++) {
    const rmt_symbol_word_t &s = symbols_[i];
    ESP_LOGD(TAG, "  sym[%02u]: {dur0=%4u lv0=%u}  {dur1=%4u lv1=%u}",
             i, s.duration0, s.level0, s.duration1, s.level1);
  }

  // ── Symbol count check ────────────────────────────────────────────────────
  static constexpr size_t DATA_OFFSET = 2;   // skip sym[0] (start) + sym[1] (ACK)
  static constexpr size_t BITS        = 40;
  static constexpr size_t MIN_SYMBOLS = DATA_OFFSET + BITS;  // 42

  if (n < MIN_SYMBOLS) {
    ESP_LOGW(TAG, "Too few RMT symbols: got %u, need at least %u — "
                  "check wiring and pullup resistor", n, MIN_SYMBOLS);
    return;
  }

  // ── Decode 40 bits MSB-first into 5 bytes ────────────────────────────────
  uint8_t data[5] = {};

  for (size_t i = 0; i < BITS; i++) {
    const rmt_symbol_word_t &sym = symbols_[DATA_OFFSET + i];

    // Each bit frame: ~50 µs LOW then HIGH whose duration encodes the value.
    // Accept LOW of 20–100 µs to tolerate cable capacitance / sensor variation.
    if (sym.level0 != 0 || sym.duration0 < 20 || sym.duration0 > 100) {
      ESP_LOGW(TAG, "Unexpected framing on bit[%u]: "
                    "level0=%u dur0=%u — noise or protocol mismatch",
               i, sym.level0, sym.duration0);
      return;
    }

    // dur1 >= 40 µs → bit '1',  dur1 < 40 µs → bit '0'
    const bool bit = (sym.duration1 >= 40);
    data[i / 8] |= static_cast<uint8_t>(bit ? 1U : 0U) << (7U - (i % 8U));
  }

  // ── Checksum validation with last-bit recovery ────────────────────────────
  //
  // If the sensor omits its trailing end pulse, the 40th bit's HIGH duration
  // is inflated to ~1000 µs (the RMT end-of-frame timeout) and decoded as '1'
  // regardless of its true value.  Since that bit is the LSB of the checksum
  // byte (data[4]) — not a data byte — we can recover by flipping it and
  // retrying.  False-positive recovery is negligible for an 8-bit sum.
  const uint8_t calc = static_cast<uint8_t>(
      (data[0] + data[1] + data[2] + data[3]) & 0xFFU);

  if (data[4] != calc) {
    const uint8_t recovered = data[4] ^ 0x01U;
    if (recovered != calc) {
      ESP_LOGW(TAG, "Checksum failed: calculated 0x%02X, received 0x%02X "
                    "(recovery also failed) — discarding reading", calc, data[4]);
      return;
    }
    ESP_LOGD(TAG, "Last-bit timeout recovery applied (end pulse absent)");
    data[4] = recovered;
  }

  // ── Voltage conversion ────────────────────────────────────────────────────
  //
  // Bytes 0+1 are a big-endian unsigned 16-bit integer: the sensor's internal
  // ADC count scaled so that dividing by 10000 gives volts.
  // e.g. raw=15870 → 1.587 V  (wet),  raw=28387 → 2.839 V  (dry)
  //
  // Note: voltage DECREASES as moisture increases — the higher the moisture,
  // the higher the capacitance, the lower the oscillator voltage.
  //
  // Bytes 2+3 are always 0x00 (MS01 has no temperature sensor).
  const int32_t raw     = (static_cast<int32_t>(data[0]) << 8) | data[1];
  const float   voltage = static_cast<float>(raw) / 10000.0f;

  ESP_LOGD(TAG, "raw=%d  voltage=%.4f V", raw, voltage);

  voltage_sensor_->publish_state(voltage);
}

}  // namespace sonoff_ms01
}  // namespace esphome

// ════════════════════════════════════════════════════════════════════════════
// Non-IDF fallback (Arduino framework selected by mistake)
// ════════════════════════════════════════════════════════════════════════════
#else  // !USE_ESP_IDF

namespace esphome {
namespace sonoff_ms01 {

void SonoffMS01Component::setup() {
  ESP_LOGE(TAG, "sonoff_ms01 requires the ESP-IDF framework. "
                "Add 'framework: {type: esp-idf}' to your esp32: section.");
  this->mark_failed();
}

void SonoffMS01Component::loop()   {}
void SonoffMS01Component::update() {}

}  // namespace sonoff_ms01
}  // namespace esphome

#endif  // USE_ESP_IDF