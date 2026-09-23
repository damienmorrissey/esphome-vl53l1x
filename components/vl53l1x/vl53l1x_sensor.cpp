// Register-level protocol ported from Pololu's VL53L1X Arduino library
// (https://github.com/pololu/vl53l1x-arduino), which is itself based on ST's
// VL53L1X API (STSW-IMG007). Timing and fixed-point arithmetic is transliterated
// verbatim from that driver; only the I2C transport differs. See LICENSE.

#include "vl53l1x_sensor.h"

#include "esphome/core/log.h"

namespace esphome::vl53l1x {

static const char *const TAG = "vl53l1x";

namespace {

// Register addresses (VL53L1X uses 16-bit register addressing).
constexpr uint16_t SOFT_RESET = 0x0000;
constexpr uint16_t OSC_MEASURED_FAST_OSC_FREQUENCY = 0x0006;
constexpr uint16_t VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t VHV_CONFIG_INIT = 0x000B;
constexpr uint16_t ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
constexpr uint16_t MM_CONFIG_OUTER_OFFSET_MM = 0x0022;
constexpr uint16_t DSS_CONFIG_TARGET_TOTAL_RATE_MCPS = 0x0024;
constexpr uint16_t PAD_I2C_HV_EXTSUP_CONFIG = 0x002E;
constexpr uint16_t GPIO_TIO_HV_STATUS = 0x0031;
constexpr uint16_t SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS = 0x0036;
constexpr uint16_t SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037;
constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039;
constexpr uint16_t ALGO_RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E;
constexpr uint16_t ALGO_RANGE_MIN_CLIP = 0x003F;
constexpr uint16_t ALGO_CONSISTENCY_CHECK_TOLERANCE = 0x0040;
constexpr uint16_t CAL_CONFIG_VCSEL_START = 0x0047;
constexpr uint16_t PHASECAL_CONFIG_TIMEOUT_MACROP = 0x004B;
constexpr uint16_t PHASECAL_CONFIG_OVERRIDE = 0x004D;
constexpr uint16_t DSS_CONFIG_ROI_MODE_CONTROL = 0x004F;
constexpr uint16_t SYSTEM_THRESH_RATE_HIGH = 0x0050;
constexpr uint16_t SYSTEM_THRESH_RATE_LOW = 0x0052;
constexpr uint16_t DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054;
constexpr uint16_t DSS_CONFIG_APERTURE_ATTENUATION = 0x0057;
constexpr uint16_t MM_CONFIG_TIMEOUT_MACROP_A = 0x005A;
constexpr uint16_t MM_CONFIG_TIMEOUT_MACROP_B = 0x005C;
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A = 0x005E;
constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060;
constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B = 0x0061;
constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063;
constexpr uint16_t RANGE_CONFIG_SIGMA_THRESH = 0x0064;
constexpr uint16_t RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
constexpr uint16_t RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069;
constexpr uint16_t SYSTEM_INTERMEASUREMENT_PERIOD = 0x006C;
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD_0 = 0x0071;
constexpr uint16_t SYSTEM_SEED_CONFIG = 0x0077;
constexpr uint16_t SD_CONFIG_WOI_SD0 = 0x0078;
constexpr uint16_t SD_CONFIG_WOI_SD1 = 0x0079;
constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A;
constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD1 = 0x007B;
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD_1 = 0x007C;
constexpr uint16_t SD_CONFIG_QUANTIFIER = 0x007E;
constexpr uint16_t SYSTEM_SEQUENCE_CONFIG = 0x0081;
constexpr uint16_t SYSTEM_GROUPED_PARAMETER_HOLD = 0x0082;
constexpr uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t SYSTEM_MODE_START = 0x0087;
constexpr uint16_t RESULT_RANGE_STATUS = 0x0089;
constexpr uint16_t PHASECAL_RESULT_VCSEL_START = 0x00D8;
constexpr uint16_t RESULT_OSC_CALIBRATE_VAL = 0x00DE;
constexpr uint16_t FIRMWARE_SYSTEM_STATUS = 0x00E5;
constexpr uint16_t IDENTIFICATION_MODEL_ID = 0x010F;

/// Model ID + module type, per the datasheet.
constexpr uint16_t EXPECTED_MODEL_ID = 0xEACC;

/// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
/// LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + 1 (see ST API).
constexpr uint32_t TIMING_GUARD = 4528;

/// DSS target rate, 9.7 fixed point (0x0A00 = 20.0 MCPS).
constexpr uint16_t TARGET_RATE = 0x0A00;

/// Max time to wait for the firmware boot poll to complete.
constexpr uint32_t BOOT_TIMEOUT_MS = 500;

}  // namespace

// --- register access -------------------------------------------------------

bool VL53L1XSensor::write_reg_(uint16_t reg, uint8_t value) {
  return this->write_register16(reg, &value, 1) == i2c::ERROR_OK;
}

bool VL53L1XSensor::write_reg16_(uint16_t reg, uint16_t value) {
  const uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return this->write_register16(reg, data, 2) == i2c::ERROR_OK;
}

bool VL53L1XSensor::write_reg32_(uint16_t reg, uint32_t value) {
  const uint8_t data[4] = {static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
                           static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return this->write_register16(reg, data, 4) == i2c::ERROR_OK;
}

bool VL53L1XSensor::read_block_(uint16_t reg, uint8_t *data, size_t len) {
  const uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
  return this->write_read(addr, 2, data, len) == i2c::ERROR_OK;
}

bool VL53L1XSensor::read_reg_(uint16_t reg, uint8_t *value) { return this->read_block_(reg, value, 1); }

bool VL53L1XSensor::read_reg16_(uint16_t reg, uint16_t *value) {
  uint8_t data[2];
  if (!this->read_block_(reg, data, 2))
    return false;
  *value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return true;
}

// --- component lifecycle ---------------------------------------------------

void VL53L1XSensor::setup() {
  if (this->enable_pin_ != nullptr)
    this->enable_pin_->setup();

  if (!this->try_init_()) {
    // Not fatal: the sensor may simply not be powered/connected yet. update()
    // keeps retrying, so it will come up on its own once it does respond.
    ESP_LOGW(TAG, "VL53L1X not detected at 0x%02X - will keep retrying", this->address_);
  }
}

bool VL53L1XSensor::try_init_() {
  this->initialized_ = false;
  this->calibrated_ = false;

  if (this->enable_pin_ != nullptr) {
    // Drive XSHUT low then high to force a clean hardware reset. Matters both
    // at boot and when recovering a sensor that was swapped or power-cycled.
    this->enable_pin_->digital_write(false);
    delay(10);
    this->enable_pin_->digital_write(true);
    // Datasheet: firmware boot takes ~1.2 ms after XSHUT is released.
    delay(10);
  }

  uint16_t model_id;
  if (!this->read_reg16_(IDENTIFICATION_MODEL_ID, &model_id)) {
    ESP_LOGV(TAG, "No response from sensor at 0x%02X", this->address_);
    return false;
  }
  if (model_id != EXPECTED_MODEL_ID) {
    ESP_LOGW(TAG, "Unexpected model ID 0x%04X (expected 0x%04X) - not a VL53L1X?", model_id, EXPECTED_MODEL_ID);
    return false;
  }

  // VL53L1_software_reset()
  if (!this->write_reg_(SOFT_RESET, 0x00))
    return false;
  delayMicroseconds(100);
  if (!this->write_reg_(SOFT_RESET, 0x01))
    return false;
  delay(1);

  // VL53L1_poll_for_boot_completion()
  const uint32_t boot_start = millis();
  while (true) {
    uint8_t status;
    if (this->read_reg_(FIRMWARE_SYSTEM_STATUS, &status) && (status & 0x01) != 0)
      break;
    if (millis() - boot_start > BOOT_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Timed out waiting for firmware boot");
      return false;
    }
    delay(1);
  }

  // VL53L1_DataInit(): switch I/O to 2V8 mode.
  uint8_t extsup;
  if (!this->read_reg_(PAD_I2C_HV_EXTSUP_CONFIG, &extsup))
    return false;
  if (!this->write_reg_(PAD_I2C_HV_EXTSUP_CONFIG, extsup | 0x01))
    return false;

  if (!this->read_reg16_(OSC_MEASURED_FAST_OSC_FREQUENCY, &this->fast_osc_frequency_))
    return false;
  if (!this->read_reg16_(RESULT_OSC_CALIBRATE_VAL, &this->osc_calibrate_val_))
    return false;
  if (this->fast_osc_frequency_ == 0) {
    ESP_LOGW(TAG, "Invalid oscillator frequency read from sensor");
    return false;
  }

  // VL53L1_StaticInit(): static config.
  this->write_reg16_(DSS_CONFIG_TARGET_TOTAL_RATE_MCPS, TARGET_RATE);
  this->write_reg_(GPIO_TIO_HV_STATUS, 0x02);
  this->write_reg_(SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS, 8);
  this->write_reg_(SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS, 16);
  this->write_reg_(ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  this->write_reg_(ALGO_RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  this->write_reg_(ALGO_RANGE_MIN_CLIP, 0);
  this->write_reg_(ALGO_CONSISTENCY_CHECK_TOLERANCE, 2);

  // General config.
  this->write_reg16_(SYSTEM_THRESH_RATE_HIGH, 0x0000);
  this->write_reg16_(SYSTEM_THRESH_RATE_LOW, 0x0000);
  this->write_reg_(DSS_CONFIG_APERTURE_ATTENUATION, 0x38);

  // Timing config.
  this->write_reg16_(RANGE_CONFIG_SIGMA_THRESH, 360);
  this->write_reg16_(RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

  // Dynamic config.
  this->write_reg_(SYSTEM_GROUPED_PARAMETER_HOLD_0, 0x01);
  this->write_reg_(SYSTEM_GROUPED_PARAMETER_HOLD_1, 0x01);
  this->write_reg_(SD_CONFIG_QUANTIFIER, 2);
  this->write_reg_(SYSTEM_GROUPED_PARAMETER_HOLD, 0x00);
  this->write_reg_(SYSTEM_SEED_CONFIG, 1);

  // VL53L1_config_low_power_auto_mode(): VHV, PHASECAL, DSS1, RANGE.
  this->write_reg_(SYSTEM_SEQUENCE_CONFIG, 0x8B);
  this->write_reg16_(DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  this->write_reg_(DSS_CONFIG_ROI_MODE_CONTROL, 2);  // REQUESTED_EFFECTIVE_SPADS

  if (!this->set_distance_mode_(this->distance_mode_)) {
    ESP_LOGW(TAG, "Failed to apply distance mode");
    return false;
  }
  if (!this->set_measurement_timing_budget_(this->timing_budget_us_)) {
    ESP_LOGW(TAG, "Failed to apply timing budget of %" PRIu32 " us", this->timing_budget_us_);
    return false;
  }

  // The ST API makes this change in VL53L1_init_and_start_range(); assumes MM1
  // and MM2 are disabled.
  uint16_t outer_offset;
  if (!this->read_reg16_(MM_CONFIG_OUTER_OFFSET_MM, &outer_offset))
    return false;
  this->write_reg16_(ALGO_PART_TO_PART_RANGE_OFFSET_MM, outer_offset * 4);

  // Run continuously, one measurement per timing budget (plus a small margin so
  // the sensor is never re-triggered before the previous reading completes).
  if (!this->start_continuous_(this->timing_budget_us_ / 1000 + 5)) {
    ESP_LOGW(TAG, "Failed to start continuous ranging");
    return false;
  }

  this->initialized_ = true;
  this->consecutive_failures_ = 0;
  this->status_clear_warning();
  ESP_LOGI(TAG, "VL53L1X initialised at 0x%02X", this->address_);
  return true;
}

void VL53L1XSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_PIN("  Enable (XSHUT) Pin: ", this->enable_pin_);
  static const char *const DISTANCE_MODE_NAMES[] = {"short", "medium", "long"};
  ESP_LOGCONFIG(TAG, "  Distance Mode: %s", DISTANCE_MODE_NAMES[this->distance_mode_]);
  ESP_LOGCONFIG(TAG, "  Timing Budget: %" PRIu32 " us", this->timing_budget_us_);
  if (!this->initialized_)
    ESP_LOGW(TAG, "  Sensor not currently responding - retrying on each update");
  LOG_SENSOR("  ", "Distance", this);
}

// --- configuration ---------------------------------------------------------

bool VL53L1XSensor::set_distance_mode_(DistanceMode mode) {
  // Save existing timing budget so it can be reapplied afterwards.
  const uint32_t budget_us = this->get_measurement_timing_budget_();

  switch (mode) {
    case DISTANCE_MODE_SHORT:
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
      this->write_reg_(RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
      this->write_reg_(SD_CONFIG_WOI_SD0, 0x07);
      this->write_reg_(SD_CONFIG_WOI_SD1, 0x05);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD0, 6);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD1, 6);
      break;

    case DISTANCE_MODE_MEDIUM:
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_A, 0x0B);
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_B, 0x09);
      this->write_reg_(RANGE_CONFIG_VALID_PHASE_HIGH, 0x78);
      this->write_reg_(SD_CONFIG_WOI_SD0, 0x0B);
      this->write_reg_(SD_CONFIG_WOI_SD1, 0x09);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD0, 10);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD1, 10);
      break;

    case DISTANCE_MODE_LONG:
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
      this->write_reg_(RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
      this->write_reg_(RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
      this->write_reg_(SD_CONFIG_WOI_SD0, 0x0F);
      this->write_reg_(SD_CONFIG_WOI_SD1, 0x0D);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD0, 14);
      this->write_reg_(SD_CONFIG_INITIAL_PHASE_SD1, 14);
      break;

    default:
      return false;
  }

  return this->set_measurement_timing_budget_(budget_us);
}

bool VL53L1XSensor::set_measurement_timing_budget_(uint32_t budget_us) {
  // Assumes PresetMode is LOWPOWER_AUTONOMOUS.
  if (budget_us <= TIMING_GUARD)
    return false;

  uint32_t range_config_timeout_us = budget_us - TIMING_GUARD;
  if (range_config_timeout_us > 1100000)  // FDA_MAX_TIMING_BUDGET_US * 2
    return false;

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values()
  uint8_t vcsel_period_a;
  if (!this->read_reg_(RANGE_CONFIG_VCSEL_PERIOD_A, &vcsel_period_a))
    return false;
  uint32_t macro_period_us = this->calc_macro_period_(vcsel_period_a);

  // "Update Phase timeout - uses Timing A". Timeout of 1000 is the tuning
  // parameter default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT).
  uint32_t phasecal_timeout_mclks = timeout_microseconds_to_mclks_(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF)
    phasecal_timeout_mclks = 0xFF;
  this->write_reg_(PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  this->write_reg16_(MM_CONFIG_TIMEOUT_MACROP_A, encode_timeout_(timeout_microseconds_to_mclks_(1, macro_period_us)));

  // "Update Range Timing A timeout"
  this->write_reg16_(RANGE_CONFIG_TIMEOUT_MACROP_A,
                     encode_timeout_(timeout_microseconds_to_mclks_(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  uint8_t vcsel_period_b;
  if (!this->read_reg_(RANGE_CONFIG_VCSEL_PERIOD_B, &vcsel_period_b))
    return false;
  macro_period_us = this->calc_macro_period_(vcsel_period_b);

  // "Update MM Timing B timeout"
  this->write_reg16_(MM_CONFIG_TIMEOUT_MACROP_B, encode_timeout_(timeout_microseconds_to_mclks_(1, macro_period_us)));

  // "Update Range Timing B timeout"
  this->write_reg16_(RANGE_CONFIG_TIMEOUT_MACROP_B,
                     encode_timeout_(timeout_microseconds_to_mclks_(range_config_timeout_us, macro_period_us)));

  return true;
}

uint32_t VL53L1XSensor::get_measurement_timing_budget_() {
  // Assumes PresetMode is LOWPOWER_AUTONOMOUS and that VHV, PHASECAL, DSS1 and
  // RANGE sequence steps are enabled.
  uint8_t vcsel_period_a;
  if (!this->read_reg_(RANGE_CONFIG_VCSEL_PERIOD_A, &vcsel_period_a))
    return 0;
  const uint32_t macro_period_us = this->calc_macro_period_(vcsel_period_a);

  uint16_t timeout_reg;
  if (!this->read_reg16_(RANGE_CONFIG_TIMEOUT_MACROP_A, &timeout_reg))
    return 0;

  const uint32_t range_config_timeout_us =
      timeout_mclks_to_microseconds_(decode_timeout_(timeout_reg), macro_period_us);

  return 2 * range_config_timeout_us + TIMING_GUARD;
}

bool VL53L1XSensor::start_continuous_(uint32_t period_ms) {
  // VL53L1_set_inter_measurement_period_ms()
  if (!this->write_reg32_(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms * this->osc_calibrate_val_))
    return false;
  if (!this->write_reg_(SYSTEM_INTERRUPT_CLEAR, 0x01))
    return false;
  return this->write_reg_(SYSTEM_MODE_START, 0x40);  // mode_range__timed
}

// --- measurement -----------------------------------------------------------

void VL53L1XSensor::update() {
  if (!this->initialized_) {
    // Covers both "never came up at boot" and "stopped responding mid-run"
    // (e.g. the sensor was unplugged or swapped) with the same path, so no
    // reboot is needed to recover.
    this->try_init_();
    return;
  }

  bool ready = false;
  if (!this->data_ready_(&ready)) {
    this->handle_read_failure_("I2C read failed");
    return;
  }
  if (!ready) {
    // With continuous ranging the inter-measurement period is <= the update
    // interval, so this should be rare; treat a persistent stall as a fault.
    this->handle_read_failure_("measurement not ready");
    return;
  }

  ResultBuffer results;
  if (!this->read_results_(&results)) {
    this->handle_read_failure_("failed to read results");
    return;
  }

  if (!this->calibrated_) {
    this->setup_manual_calibration_();
    this->calibrated_ = true;
  }

  this->update_dss_(results);

  RangeStatus status;
  const uint16_t range_mm = this->get_range_mm_(results, &status);

  this->write_reg_(SYSTEM_INTERRUPT_CLEAR, 0x01);

  this->consecutive_failures_ = 0;
  this->status_clear_warning();

  if (status != RANGE_STATUS_VALID && status != RANGE_STATUS_VALID_MIN_RANGE_CLIPPED &&
      status != RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL) {
    // A measurement completed but the sensor flagged it as unreliable (no
    // target in range, too much ambient light, etc). Publishing it would look
    // like a real distance, so publish unknown instead.
    ESP_LOGD(TAG, "Measurement invalid (range status %u), publishing unknown", status);
    this->publish_state(NAN);
    return;
  }

  ESP_LOGD(TAG, "Got distance %u mm (range status %u)", range_mm, status);
  this->publish_state(range_mm / 1000.0f);
}

void VL53L1XSensor::handle_read_failure_(const char *reason) {
  this->consecutive_failures_++;
  this->status_set_warning();
  ESP_LOGW(TAG, "%s (%u/%u consecutive)", reason, this->consecutive_failures_, MAX_CONSECUTIVE_FAILURES);

  if (this->consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
    ESP_LOGW(TAG, "Sensor unresponsive - will attempt to reinitialise");
    this->initialized_ = false;
    // A stale reading is worse than an obviously-unknown one once we have
    // concluded the sensor is no longer there.
    this->publish_state(NAN);
  }
}

bool VL53L1XSensor::data_ready_(bool *ready) {
  uint8_t status;
  if (!this->read_reg_(GPIO_TIO_HV_STATUS, &status))
    return false;
  *ready = (status & 0x01) == 0;
  return true;
}

bool VL53L1XSensor::read_results_(ResultBuffer *results) {
  uint8_t buf[17];
  if (!this->read_block_(RESULT_RANGE_STATUS, buf, sizeof(buf)))
    return false;

  // Field offsets match the ST result block layout; unused fields are skipped.
  results->range_status = buf[0];
  // buf[1]: report_status - not used
  results->stream_count = buf[2];
  results->dss_actual_effective_spads_sd0 = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
  // buf[5..6]: peak_signal_count_rate_mcps_sd0 - not used
  results->ambient_count_rate_mcps_sd0 = (static_cast<uint16_t>(buf[7]) << 8) | buf[8];
  // buf[9..10]: sigma_sd0 - not used
  // buf[11..12]: phase_sd0 - not used
  results->final_crosstalk_corrected_range_mm_sd0 = (static_cast<uint16_t>(buf[13]) << 8) | buf[14];
  results->peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (static_cast<uint16_t>(buf[15]) << 8) | buf[16];
  return true;
}

bool VL53L1XSensor::setup_manual_calibration_() {
  // VL53L1_low_power_auto_setup_manual_calibration()
  if (!this->read_reg_(VHV_CONFIG_INIT, &this->saved_vhv_init_))
    return false;
  if (!this->read_reg_(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, &this->saved_vhv_timeout_))
    return false;

  // "disable VHV init"
  this->write_reg_(VHV_CONFIG_INIT, this->saved_vhv_init_ & 0x7F);
  // "set loop bound to tuning param" (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
  this->write_reg_(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, (this->saved_vhv_timeout_ & 0x03) + (3 << 2));
  // "override phasecal"
  this->write_reg_(PHASECAL_CONFIG_OVERRIDE, 0x01);

  uint8_t vcsel_start;
  if (!this->read_reg_(PHASECAL_RESULT_VCSEL_START, &vcsel_start))
    return false;
  return this->write_reg_(CAL_CONFIG_VCSEL_START, vcsel_start);
}

bool VL53L1XSensor::update_dss_(const ResultBuffer &results) {
  // VL53L1_low_power_auto_update_DSS()
  const uint16_t spad_count = results.dss_actual_effective_spads_sd0;

  if (spad_count != 0) {
    uint32_t total_rate_per_spad = static_cast<uint32_t>(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0) +
                                   results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (total_rate_per_spad > 0xFFFF)
      total_rate_per_spad = 0xFFFF;

    // "shift up to take advantage of 32 bits"
    total_rate_per_spad <<= 16;
    total_rate_per_spad /= spad_count;

    if (total_rate_per_spad != 0) {
      // "get the target rate and shift up by 16"
      uint32_t required_spads = (static_cast<uint32_t>(TARGET_RATE) << 16) / total_rate_per_spad;

      // "clip to 16 bit"
      if (required_spads > 0xFFFF)
        required_spads = 0xFFFF;

      return this->write_reg16_(DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, required_spads);
    }
  }

  // Anything above would have divided by zero; "we want to gracefully set a
  // spad target, not just exit with an error" - set the target to the midpoint.
  return this->write_reg16_(DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

uint16_t VL53L1XSensor::get_range_mm_(const ResultBuffer &results, RangeStatus *status) {
  // VL53L1_copy_sys_and_core_results_to_range_results(): apply the correction
  // gain. 2011 is the tuning parameter default
  // (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT), i.e. scale by
  // 2011/2048 with 1024 added for rounding.
  const uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;
  const uint16_t range_mm = (static_cast<uint32_t>(range) * 2011 + 0x0400) / 0x0800;

  // Map the raw RESULT__RANGE_STATUS value, mostly per ConvertStatusLite().
  switch (results.range_status) {
    case 17:  // MULTCLIPFAIL
    case 2:   // VCSELWATCHDOGTESTFAILURE
    case 1:   // VCSELCONTINUITYTESTFAILURE
    case 3:   // NOVHVVALUEFOUND
      *status = RANGE_STATUS_HARDWARE_FAIL;
      break;
    case 13:  // USERROICLIP
      *status = RANGE_STATUS_MIN_RANGE_FAIL;
      break;
    case 18:  // GPHSTREAMCOUNT0READY
      *status = RANGE_STATUS_SYNCHRONIZATION_INT;
      break;
    case 5:  // RANGEPHASECHECK
      *status = RANGE_STATUS_OUT_OF_BOUNDS_FAIL;
      break;
    case 4:  // MSRCNOTARGET
      *status = RANGE_STATUS_SIGNAL_FAIL;
      break;
    case 6:  // SIGMATHRESHOLDCHECK
      *status = RANGE_STATUS_SIGMA_FAIL;
      break;
    case 7:  // PHASECONSISTENCY
      *status = RANGE_STATUS_WRAP_TARGET_FAIL;
      break;
    case 12:  // RANGEIGNORETHRESHOLD
      *status = RANGE_STATUS_XTALK_SIGNAL_FAIL;
      break;
    case 8:  // MINCLIP
      *status = RANGE_STATUS_VALID_MIN_RANGE_CLIPPED;
      break;
    case 9:  // RANGECOMPLETE
      *status = results.stream_count == 0 ? RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL : RANGE_STATUS_VALID;
      break;
    default:
      *status = RANGE_STATUS_NONE;
      break;
  }

  return range_mm;
}

// --- timing helpers --------------------------------------------------------

uint32_t VL53L1XSensor::decode_timeout_(uint16_t reg_val) {
  // VL53L1_decode_timeout()
  return (static_cast<uint32_t>(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint16_t VL53L1XSensor::encode_timeout_(uint32_t timeout_mclks) {
  // VL53L1_encode_timeout(): encoded format is "(LSByte * 2^MSByte) + 1".
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks == 0)
    return 0;

  ls_byte = timeout_mclks - 1;
  while ((ls_byte & 0xFFFFFF00) > 0) {
    ls_byte >>= 1;
    ms_byte++;
  }
  return (ms_byte << 8) | (ls_byte & 0xFF);
}

uint32_t VL53L1XSensor::timeout_mclks_to_microseconds_(uint32_t timeout_mclks, uint32_t macro_period_us) {
  // VL53L1_calc_timeout_us(); macro_period_us is 12.12 fixed point.
  return (static_cast<uint64_t>(timeout_mclks) * macro_period_us + 0x800) >> 12;
}

uint32_t VL53L1XSensor::timeout_microseconds_to_mclks_(uint32_t timeout_us, uint32_t macro_period_us) {
  // VL53L1_calc_timeout_mclks(); macro_period_us is 12.12 fixed point.
  return ((static_cast<uint32_t>(timeout_us) << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t VL53L1XSensor::calc_macro_period_(uint8_t vcsel_period) {
  // VL53L1_calc_pll_period_us(): fast oscillator frequency is 4.12 format, the
  // resulting PLL period is 0.24 format.
  const uint32_t pll_period_us = (static_cast<uint32_t>(0x01) << 30) / this->fast_osc_frequency_;

  // VL53L1_decode_vcsel_period()
  const uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_calc_macro_period_us(); VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304.
  uint32_t macro_period_us = static_cast<uint32_t>(2304) * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

}  // namespace esphome::vl53l1x
