#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::vl53l1x {

/// Ranging mode. Longer modes see further but are more sensitive to ambient light.
enum DistanceMode : uint8_t {
  DISTANCE_MODE_SHORT,   // up to ~1.3 m, best ambient immunity
  DISTANCE_MODE_MEDIUM,  // up to ~3 m
  DISTANCE_MODE_LONG,    // up to ~4 m, most ambient sensitive
};

/// Decoded measurement status, from VL53L1_GetRangingMeasurementData().
/// Values are prefixed to avoid collisions with platform SDK macros.
enum RangeStatus : uint8_t {
  RANGE_STATUS_VALID = 0,
  RANGE_STATUS_SIGMA_FAIL = 1,
  RANGE_STATUS_SIGNAL_FAIL = 2,
  RANGE_STATUS_VALID_MIN_RANGE_CLIPPED = 3,
  RANGE_STATUS_OUT_OF_BOUNDS_FAIL = 4,
  RANGE_STATUS_HARDWARE_FAIL = 5,
  RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL = 6,
  RANGE_STATUS_WRAP_TARGET_FAIL = 7,
  RANGE_STATUS_XTALK_SIGNAL_FAIL = 9,
  RANGE_STATUS_SYNCHRONIZATION_INT = 10,
  RANGE_STATUS_MIN_RANGE_FAIL = 13,
  RANGE_STATUS_NONE = 255,
};

/// Raw 17-byte result block read from RESULT__RANGE_STATUS.
struct ResultBuffer {
  uint8_t range_status;
  uint8_t stream_count;
  uint16_t dss_actual_effective_spads_sd0;
  uint16_t ambient_count_rate_mcps_sd0;
  uint16_t final_crosstalk_corrected_range_mm_sd0;
  uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
};

/// ST VL53L1X time-of-flight distance sensor (up to ~4 m).
///
/// The register-level protocol is ported from Pololu's VL53L1X Arduino library,
/// which is itself derived from ST's VL53L1X API (STSW-IMG007). All of the
/// timing/fixed-point arithmetic is transliterated verbatim from that driver;
/// only the I2C transport was replaced with ESPHome's i2c::I2CDevice so the
/// component works under ESP-IDF without the Arduino Wire library.
class VL53L1XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_distance_mode(DistanceMode mode) { this->distance_mode_ = mode; }
  void set_timing_budget_us(uint32_t timing_budget_us) { this->timing_budget_us_ = timing_budget_us; }
  void set_enable_pin(GPIOPin *enable_pin) { this->enable_pin_ = enable_pin; }

 protected:
  // --- register access (VL53L1X uses 16-bit register addresses) ---
  bool write_reg_(uint16_t reg, uint8_t value);
  bool write_reg16_(uint16_t reg, uint16_t value);
  bool write_reg32_(uint16_t reg, uint32_t value);
  bool read_reg_(uint16_t reg, uint8_t *value);
  bool read_reg16_(uint16_t reg, uint16_t *value);
  bool read_block_(uint16_t reg, uint8_t *data, size_t len);

  // --- device bring-up ---
  /// Full reset + initialisation sequence. Safe to call repeatedly; used both at
  /// boot and to recover a sensor that stopped responding (e.g. was hot-swapped).
  bool try_init_();
  bool set_distance_mode_(DistanceMode mode);
  bool set_measurement_timing_budget_(uint32_t budget_us);
  uint32_t get_measurement_timing_budget_();
  bool start_continuous_(uint32_t period_ms);

  // --- measurement ---
  bool data_ready_(bool *ready);
  bool read_results_(ResultBuffer *results);
  bool setup_manual_calibration_();
  bool update_dss_(const ResultBuffer &results);
  uint16_t get_range_mm_(const ResultBuffer &results, RangeStatus *status);
  void handle_read_failure_(const char *reason);

  // --- timing helpers (verbatim from the ST/Pololu calculation) ---
  static uint32_t decode_timeout_(uint16_t reg_val);
  static uint16_t encode_timeout_(uint32_t timeout_mclks);
  static uint32_t timeout_mclks_to_microseconds_(uint32_t timeout_mclks, uint32_t macro_period_us);
  static uint32_t timeout_microseconds_to_mclks_(uint32_t timeout_us, uint32_t macro_period_us);
  uint32_t calc_macro_period_(uint8_t vcsel_period);

  DistanceMode distance_mode_{DISTANCE_MODE_LONG};
  uint32_t timing_budget_us_{50000};
  GPIOPin *enable_pin_{nullptr};

  uint16_t fast_osc_frequency_{0};
  uint16_t osc_calibrate_val_{0};
  bool calibrated_{false};
  uint8_t saved_vhv_init_{0};
  uint8_t saved_vhv_timeout_{0};

  bool initialized_{false};
  uint8_t consecutive_failures_{0};
  /// After this many consecutive failed reads, assume the sensor is gone and
  /// fall back to retrying try_init_() on each update.
  static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 3;
};

}  // namespace esphome::vl53l1x
