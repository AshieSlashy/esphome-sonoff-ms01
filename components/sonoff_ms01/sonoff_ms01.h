/**
 * sonoff_ms01.h — Sonoff MS01 Soil Moisture Sensor driver
 * =========================================================
 *
 * Protocol
 * --------
 * The MS01 uses the same single-wire signalling as the DHT22/AM2301 family:
 *
 *   Host  ──LOW 450µs──► release (HIGH)
 *   Sensor              ──LOW ~80µs──HIGH ~80µs──[40 data bits]──LOW ~50µs──
 *
 * Each data bit is framed as ~50µs LOW followed by a HIGH whose duration
 * encodes the bit value:
 *   HIGH  26-28 µs  →  logic '0'
 *   HIGH  ~70  µs   →  logic '1'
 *
 * 40 bits arrive MSB-first in 5 bytes:
 *   [0] voltage high byte
 *   [1] voltage low byte
 *   [2] always 0x00  (temperature not measured)
 *   [3] always 0x00
 *   [4] checksum = (byte0 + byte1 + byte2 + byte3) & 0xFF
 *
 * Implementation strategy
 * -----------------------
 * The start pulse is bit-banged out of the GPIO (simplicity; timing tolerance
 * is ±tens of µs so software is fine).  The sensor response is captured by a
 * single RMT RX channel that is allocated only for the ~4-5 ms the sensor
 * needs to transmit, then released back to the shared pool.
 *
 * The state machine uses ESPHome's update() / loop() split:
 *   update()  — arm RMT, send start pulse, return immediately
 *   loop()    — on each tick: check for ISR completion or timeout,
 *               decode + publish in main-loop context, free RMT channel
 *
 * The RMT done callback runs in ISR context and does nothing except flip a
 * volatile state flag and record the symbol count — no allocations, no
 * publish_state() calls, no FreeRTOS API beyond what is ISR-safe.
 *
 * Requirements
 * ------------
 *   - ESP32 (any variant with RMT: original, S2, S3, C3, C6 …)
 *   - ESP-IDF framework v5+  (NOT Arduino)
 *   - One free RMT RX channel during each read (~5 ms per poll)
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

// ── ESP-IDF RMT API (IDF v5) ─────────────────────────────────────────────
// USE_ESP_IDF is defined by the ESPHome build system when the ESP-IDF
// framework is selected.  The guard lets the non-IDF fallback in .cpp
// produce a clean mark_failed() rather than a cascade of type errors.
#ifdef USE_ESP_IDF
#  include "esp_idf_version.h"
#  if ESP_IDF_VERSION_MAJOR < 5
     // Should not be reachable — sensor.py's _validate_not_arduino() rejects
     // Arduino builds, and ESPHome 2025.2+ bundles IDF v5 by default.
#    error "sonoff_ms01 requires ESP-IDF v5+. Add 'version: recommended' under framework: in your esp32: section."
#  endif
#  include "driver/rmt_rx.h"   // requires CMake component: esp_driver_rmt
#  include "driver/gpio.h"
#  include <atomic>
#endif

namespace esphome {
namespace sonoff_ms01 {

class SonoffMS01Component : public PollingComponent {
 public:
  // ── Setters called by ESPHome code-generation ──────────────────────────
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_voltage_sensor(sensor::Sensor *s)  { voltage_sensor_  = s; }

  // ── PollingComponent interface ─────────────────────────────────────────
  void setup()  override;
  void loop()   override;
  void update() override;

  // Run after wifi/api init so sensors are registered before first read
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
#ifdef USE_ESP_IDF

  // ── RMT helpers ──────────────────────────────────────────────────────────
  /** Allocate one RMT RX channel, register the done callback, enable it.
   *  Returns false (and logs a warning) if no channel is available. */
  bool allocate_rmt_();

  /** Disable + delete the RMT channel, returning it to the shared pool. */
  void release_rmt_();

  /** Parse symbols_ and publish to ESPHome sensors.
   *  Called from loop() (main-loop context — safe to call publish_state). */
  void decode_and_publish_();

  // ── ISR callback ─────────────────────────────────────────────────────────
  /**
   * Called from RMT interrupt context when the frame is complete.
   *
   * CONTRACT — do not break:
   *   • No heap allocation (no new / malloc / String construction)
   *   • No FreeRTOS blocking calls (no xSemaphoreTake with non-zero timeout)
   *   • No ESPHome publish_state() or any function that may yield/delay
   *   • Only touch ISR-safe, volatile-qualified members
   *
   * The received symbols already live in symbols_[] (the buffer we passed
   * to rmt_receive()); the ISR just records the count and flips the state.
   */
  static bool IRAM_ATTR rmt_rx_done_cb_(rmt_channel_handle_t            channel,
                                         const rmt_rx_done_event_data_t *edata,
                                         void                           *user_ctx);

  // ── RMT state ────────────────────────────────────────────────────────────

  rmt_channel_handle_t rx_channel_{nullptr};

  // Symbol buffer passed directly to rmt_receive().  The RMT peripheral
  // writes into this buffer via its internal FIFO.  Sized to one RMT memory
  // block (64 symbols); we expect at most ~43 symbols per frame.
  rmt_symbol_word_t symbols_[64];

  // State machine ─────────────────────────────────────────────────────────
  enum class State : uint8_t {
    IDLE,     ///< Waiting for next update() call
    WAITING,  ///< RMT armed, start pulse sent, waiting for ISR
    DONE,     ///< ISR fired; data ready in symbols_[] for loop() to consume
  };

  // Written by ISR (Core 0), read by loop() (Core 1).
  // std::atomic with relaxed ordering is sufficient:
  // we only need atomicity of the single-byte write/read, not
  // ordering of surrounding memory operations.
  std::atomic<State> state_{State::IDLE};

  // Written by ISR only — volatile is sufficient since it's a single word
  // and is only read after state_ == DONE is observed (which acts as the
  // synchronisation point via the atomic load).
  volatile size_t num_symbols_{0};

  uint32_t deadline_ms_{0};  ///< Absolute ms timestamp for read timeout

#endif  // USE_ESP_IDF

  // ── Shared members (always present) ──────────────────────────────────────
  InternalGPIOPin  *pin_{nullptr};
  sensor::Sensor   *voltage_sensor_{nullptr};
};

}  // namespace sonoff_ms01
}  // namespace esphome