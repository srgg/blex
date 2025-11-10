#include "device_settings.h"
#include "../../../../src/log.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <cmath>
#include <mutex>    // for std::call_once

// ============================================================================
// PImpl: DeviceSettingsImpl definition (hidden from header)
// ============================================================================

struct DeviceSettingsImpl {
  ImuCalibration imu_cal;
  bool apply_calibration;
};

// Compile-time verification: NVS binary compatibility
static_assert(sizeof(float) == 4, "Requires IEEE 754 32-bit float for NVS compatibility");
static_assert(sizeof(ImuCalibration) == 76, "ImuCalibration size changed - verify NVS compatibility");

// Compile-time verification: builder storage is large enough for DeviceSettingsImpl
static_assert(sizeof(DeviceSettingsImpl) <= DeviceSettingsBuilder::IMPL_STORAGE_SIZE,
              "Builder storage too small for DeviceSettingsImpl");

static DeviceSettingsImpl g_settings_impl;
static std::timed_mutex g_settings_mutex;  // Prevents torn reads/writes
static constexpr auto MUTEX_TIMEOUT = std::chrono::milliseconds(1000);

DeviceSettings::ImplLock::ImplLock() noexcept : owns_lock_(false) {
  if (g_settings_mutex.try_lock_for(MUTEX_TIMEOUT)) {
    owns_lock_ = true;
  } else {
    BLIM_LOG_WARN("Settings mutex timeout (%dms) - blocking until acquired...\n",
             static_cast<int>(MUTEX_TIMEOUT.count()));

    auto block_start = std::chrono::steady_clock::now();
    g_settings_mutex.lock();  // Block until acquired (prevents torn reads)
    owns_lock_ = true;

    auto block_duration = std::chrono::steady_clock::now() - block_start;
    auto block_ms = std::chrono::duration_cast<std::chrono::milliseconds>(block_duration).count();
    BLIM_LOG_WARN("Settings mutex acquired after blocking for %lld ms\n",
             static_cast<long long>(block_ms));
  }
}

DeviceSettings::ImplLock::~ImplLock() noexcept {
  if (owns_lock_) {
    g_settings_mutex.unlock();
  }
}

DeviceSettings::ImplLock::ImplLock(ImplLock&& other) noexcept : owns_lock_(other.owns_lock_) {
  other.owns_lock_ = false;
}

DeviceSettings::ImplLock& DeviceSettings::ImplLock::operator=(ImplLock&& other) noexcept {
  if (this != &other) {
    if (owns_lock_) {
      g_settings_mutex.unlock();
    }
    owns_lock_ = other.owns_lock_;
    other.owns_lock_ = false;
  }
  return *this;
}

const DeviceSettingsImpl* DeviceSettings::ImplLock::operator->() const noexcept {
  return &g_settings_impl;
}

const DeviceSettingsImpl& DeviceSettings::ImplLock::operator*() const noexcept {
  return g_settings_impl;
}

DeviceSettings::ImplLock DeviceSettings::impl() const noexcept {
  return ImplLock();
}

// NVS storage configuration
constexpr const char* NVS_NAMESPACE = "dev_config";
constexpr const char* NVS_KEY_DEVICE_CONFIG = "device_cfg";

// JSON limits (BLE MTU 512 - 12 bytes overhead = 500)
static constexpr size_t MAX_JSON_SIZE = 500;
static constexpr size_t MAX_JSON_NESTING_DEPTH = 10;  // Prevent stack overflow

// Buffer size includes ArduinoJson metadata overhead (~2x input size)
// Note: JsonDocument (v7+) handles buffer allocation automatically
static constexpr size_t JSON_DOC_BUFFER_SIZE = 1024;

using JsonDoc = JsonDocument;

static constexpr DeviceSettingsImpl FACTORY_DEFAULTS = {
  .imu_cal = {
    .accel_zerog = {0.0f, 0.0f, 0.0f},
    .gyro_zerorate = {0.0f, 0.0f, 0.0f},
    .mag_hardiron = {0.0f, 0.0f, 0.0f},
    .mag_field = 50.0f,  // Typical Earth field (µT)
    .mag_softiron = {    // Identity matrix (3×3 row-major)
      1.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 1.0f
    }
  },
  .apply_calibration = false
};

extern "C" uint32_t crc32_le(uint32_t crc, const uint8_t* buf, uint32_t len);

namespace nvs {
  struct NvsBlob {
    DeviceSettingsImpl data;
    uint32_t checksum;
  };

  static uint32_t compute_crc32(const void* data, size_t size) {
    return crc32_le(0, static_cast<const uint8_t*>(data), size);
  }

  static void apply_factory_defaults(DeviceSettingsImpl* impl) {
    *impl = FACTORY_DEFAULTS;
  }

  static bool load_from_nvs(DeviceSettingsImpl* impl) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, true);

    NvsBlob blob;
    size_t bytes_read = prefs.getBytes(NVS_KEY_DEVICE_CONFIG, &blob, sizeof(NvsBlob));

    prefs.end();

    if (bytes_read != sizeof(NvsBlob)) {
      BLIM_LOG_WARN("NVS data size mismatch (%u bytes, expected %u)\n",
               static_cast<unsigned int>(bytes_read),
               static_cast<unsigned int>(sizeof(NvsBlob)));
      return false;
    }

    uint32_t computed_crc = compute_crc32(&blob.data, sizeof(blob.data));
    if (computed_crc != blob.checksum) {
      BLIM_LOG_ERROR("NVS checksum mismatch (computed 0x%08X, stored 0x%08X) - flash corruption detected\n",
                static_cast<unsigned int>(computed_crc),
                static_cast<unsigned int>(blob.checksum));
      return false;
    }

    *impl = blob.data;
    return true;
  }

  static bool save_to_nvs(const DeviceSettingsImpl* impl) {
    NvsBlob blob;
    blob.data = *impl;
    blob.checksum = compute_crc32(&blob.data, sizeof(blob.data));

    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);

    size_t bytes_written = prefs.putBytes(NVS_KEY_DEVICE_CONFIG, &blob, sizeof(NvsBlob));

    prefs.end();

    if (bytes_written == sizeof(NvsBlob)) {
      BLIM_LOG_INFO("Configuration saved to NVS (CRC32: 0x%08X)\n",
               static_cast<unsigned int>(blob.checksum));
      return true;
    } else {
      BLIM_LOG_ERROR("Failed to save configuration to NVS (wrote %u of %u bytes)\n",
                static_cast<unsigned int>(bytes_written),
                static_cast<unsigned int>(sizeof(NvsBlob)));
      return false;
    }
  }
}

// ============================================================================
// Public API Implementation
// ============================================================================

const DeviceSettings& DeviceSettings::get() {
  static DeviceSettings settings;
  static std::once_flag init_flag;

  std::call_once(init_flag, []() {
    if (nvs::load_from_nvs(&g_settings_impl)) {
      BLIM_LOG_INFO("Configuration loaded from NVS (stream: %s)\n",
               g_settings_impl.apply_calibration ? "calibrated" : "raw");
    } else {
      BLIM_LOG_WARN("No saved configuration found, using factory defaults\n");
      nvs::apply_factory_defaults(&g_settings_impl);
    }
  });

  return settings;
}

DeviceSettingsBuilder DeviceSettings::modify() {
  return DeviceSettingsBuilder();
}

bool DeviceSettings::is_calibration_enabled() const {
  return impl()->apply_calibration;
}

const float* DeviceSettings::get_accel_zero_g() const {
  return impl()->imu_cal.accel_zerog;
}

const float* DeviceSettings::get_gyro_zero_rate() const {
  return impl()->imu_cal.gyro_zerorate;
}

const float* DeviceSettings::get_mag_hard_iron() const {
  return impl()->imu_cal.mag_hardiron;
}

float DeviceSettings::get_mag_field() const {
  return impl()->imu_cal.mag_field;
}

const float* DeviceSettings::get_mag_soft_iron() const {
  return impl()->imu_cal.mag_softiron;
}

// ============================================================================
// Builder API Implementation
// ============================================================================

DeviceSettingsBuilder::DeviceSettingsBuilder()
  : working_copy_(new (storage_) DeviceSettingsImpl()), dirty_flags_(0), error_(nullptr) {
}

DeviceSettingsBuilder::~DeviceSettingsBuilder() {
  if (working_copy_) {
    working_copy_->~DeviceSettingsImpl();
  }
}

DeviceSettingsBuilder::DeviceSettingsBuilder(DeviceSettingsBuilder&& other) noexcept
  : dirty_flags_(other.dirty_flags_), error_(other.error_) {
  memcpy(storage_, other.storage_, IMPL_STORAGE_SIZE);
  working_copy_ = reinterpret_cast<DeviceSettingsImpl*>(storage_);
  other.working_copy_ = nullptr;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::operator=(DeviceSettingsBuilder&& other) noexcept {
  if (this != &other) {
    memcpy(storage_, other.storage_, IMPL_STORAGE_SIZE);
    working_copy_ = reinterpret_cast<DeviceSettingsImpl*>(storage_);
    dirty_flags_ = other.dirty_flags_;
    error_ = other.error_;
    other.working_copy_ = nullptr;
  }
  return *this;
}

void DeviceSettingsBuilder::set_error(const char* error) {
  error_ = error;
}

void DeviceSettingsBuilder::clear_error() {
  error_ = nullptr;
}

void DeviceSettingsBuilder::apply_changes_to_merged(DeviceSettingsImpl& merged) const {
  if (dirty_flags_ & DIRTY_ACCEL_ZEROG) {
    memcpy(merged.imu_cal.accel_zerog, working_copy_->imu_cal.accel_zerog,
           sizeof(merged.imu_cal.accel_zerog));
  }
  if (dirty_flags_ & DIRTY_GYRO_ZERORATE) {
    memcpy(merged.imu_cal.gyro_zerorate, working_copy_->imu_cal.gyro_zerorate,
           sizeof(merged.imu_cal.gyro_zerorate));
  }
  if (dirty_flags_ & DIRTY_MAG_HARDIRON) {
    memcpy(merged.imu_cal.mag_hardiron, working_copy_->imu_cal.mag_hardiron,
           sizeof(merged.imu_cal.mag_hardiron));
  }
  if (dirty_flags_ & DIRTY_MAG_FIELD) {
    merged.imu_cal.mag_field = working_copy_->imu_cal.mag_field;
  }
  if (dirty_flags_ & DIRTY_MAG_SOFTIRON) {
    memcpy(merged.imu_cal.mag_softiron, working_copy_->imu_cal.mag_softiron,
           sizeof(merged.imu_cal.mag_softiron));
  }
  if (dirty_flags_ & DIRTY_APPLY_CALIBRATION) {
    merged.apply_calibration = working_copy_->apply_calibration;
  }
}

// ---- IMU Calibration Setters ----

DeviceSettingsBuilder& DeviceSettingsBuilder::set_accel_zero_g(const float v[3]) {
  memcpy(working_copy_->imu_cal.accel_zerog, v, sizeof(working_copy_->imu_cal.accel_zerog));
  dirty_flags_ |= DIRTY_ACCEL_ZEROG;
  return *this;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::set_gyro_zero_rate(const float v[3]) {
  memcpy(working_copy_->imu_cal.gyro_zerorate, v, sizeof(working_copy_->imu_cal.gyro_zerorate));
  dirty_flags_ |= DIRTY_GYRO_ZERORATE;
  return *this;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::set_mag_hard_iron(const float v[3]) {
  memcpy(working_copy_->imu_cal.mag_hardiron, v, sizeof(working_copy_->imu_cal.mag_hardiron));
  dirty_flags_ |= DIRTY_MAG_HARDIRON;
  return *this;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::set_mag_field(float field) {
  working_copy_->imu_cal.mag_field = field;
  dirty_flags_ |= DIRTY_MAG_FIELD;
  return *this;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::set_mag_soft_iron(const float matrix[9]) {
  memcpy(working_copy_->imu_cal.mag_softiron, matrix, sizeof(working_copy_->imu_cal.mag_softiron));
  dirty_flags_ |= DIRTY_MAG_SOFTIRON;
  return *this;
}

// ---- Settings State Setters ----

DeviceSettingsBuilder& DeviceSettingsBuilder::set_apply_calibration(bool apply) {
  working_copy_->apply_calibration = apply;
  dirty_flags_ |= DIRTY_APPLY_CALIBRATION;
  BLIM_LOG_INFO("IMU stream: %s\n", apply ? "CALIBRATED" : "RAW");
  return *this;
}

// ---- JSON API (partial merge) ----

DeviceSettingsBuilder& DeviceSettingsBuilder::merge_json(const char* json) {
  if (!json) {
    set_error("JSON input is null");
    BLIM_LOG_ERROR("JSON input is null\n");
    return *this;
  }

  size_t json_len = strlen(json);
  if (json_len > MAX_JSON_SIZE) {
    set_error("JSON too large");
    BLIM_LOG_ERROR("JSON size (%u bytes) exceeds BLE limit (%u bytes)\n",
              static_cast<unsigned int>(json_len),
              static_cast<unsigned int>(MAX_JSON_SIZE));
    return *this;
  }

  JsonDoc doc;

  DeserializationError error = deserializeJson(
    doc, json,
    DeserializationOption::NestingLimit(MAX_JSON_NESTING_DEPTH)
  );

  if (error) {
    set_error("JSON parse error");
    BLIM_LOG_ERROR("JSON parse error: %s\n", error.c_str());
    return *this;
  }

  if (doc["settings"]["apply_calibration"].is<bool>()) {
    working_copy_->apply_calibration = doc["settings"]["apply_calibration"];
    dirty_flags_ |= DIRTY_APPLY_CALIBRATION;
    BLIM_LOG_DEBUG("  ✓ Updated settings.apply_calibration: %s\n",
              working_copy_->apply_calibration ? "true" : "false");
  }

  if (doc["imu"].is<JsonObject>()) {
    JsonObject imu = doc["imu"];

    if (imu["accel"]["zerog"].is<JsonArray>()) {
      JsonArray zerog = imu["accel"]["zerog"];
      if (zerog.size() >= 3) {
        for (int i = 0; i < 3; i++) {
          working_copy_->imu_cal.accel_zerog[i] = zerog[i];
        }
        dirty_flags_ |= DIRTY_ACCEL_ZEROG;
        BLIM_LOG_DEBUG("  ✓ Updated imu.accel.zerog\n");
      }
    }

    if (imu["gyro"]["zerorate"].is<JsonArray>()) {
      JsonArray zerorate = imu["gyro"]["zerorate"];
      if (zerorate.size() >= 3) {
        for (int i = 0; i < 3; i++) {
          working_copy_->imu_cal.gyro_zerorate[i] = zerorate[i];
        }
        dirty_flags_ |= DIRTY_GYRO_ZERORATE;
        BLIM_LOG_DEBUG("  ✓ Updated imu.gyro.zerorate\n");
      }
    }

    if (imu["mag"].is<JsonObject>()) {
      JsonObject mag = imu["mag"];

      if (mag["hardiron"].is<JsonArray>()) {
        JsonArray hardiron = mag["hardiron"];
        if (hardiron.size() >= 3) {
          for (int i = 0; i < 3; i++) {
            working_copy_->imu_cal.mag_hardiron[i] = hardiron[i];
          }
          dirty_flags_ |= DIRTY_MAG_HARDIRON;
          BLIM_LOG_DEBUG("  ✓ Updated imu.mag.hardiron\n");
        }
      }

      if (mag["field"].is<float>()) {
        working_copy_->imu_cal.mag_field = mag["field"];
        dirty_flags_ |= DIRTY_MAG_FIELD;
        BLIM_LOG_DEBUG("  ✓ Updated imu.mag.field\n");
      }

      if (mag["softiron"].is<JsonArray>()) {
        JsonArray softiron = mag["softiron"];
        int idx = 0;
        for (JsonArray row : softiron) {
          for (float val : row) {
            if (idx < 9) {
              working_copy_->imu_cal.mag_softiron[idx++] = val;
            }
          }
        }
        if (idx == 9) {
          dirty_flags_ |= DIRTY_MAG_SOFTIRON;
          BLIM_LOG_DEBUG("  ✓ Updated imu.mag.softiron\n");
        }
      }
    }
  }

  return *this;
}

DeviceSettingsBuilder& DeviceSettingsBuilder::reset(bool factoryReset) {
  if (factoryReset) {
    nvs::apply_factory_defaults(working_copy_);
    dirty_flags_ = 0xFF;
    BLIM_LOG_INFO("Factory defaults applied (not saved)\n");
  } else {
    if (nvs::load_from_nvs(working_copy_)) {
      dirty_flags_ = 0xFF;
      BLIM_LOG_INFO("Settings reloaded from NVS\n");
    } else {
      nvs::apply_factory_defaults(working_copy_);
      dirty_flags_ = 0xFF;
      BLIM_LOG_INFO("No saved settings, applying factory defaults\n");
    }
  }
  return *this;
}

bool DeviceSettingsBuilder::validate_merged(const DeviceSettingsImpl& merged) {
  if (merged.imu_cal.mag_field <= 0.0f) {
    set_error("Invalid mag_field: must be > 0");
    return false;
  }

  for (int i = 0; i < 3; i++) {
    if (!isfinite(merged.imu_cal.accel_zerog[i])) {
      set_error("Invalid accel_zerog: non-finite value");
      return false;
    }
    if (!isfinite(merged.imu_cal.gyro_zerorate[i])) {
      set_error("Invalid gyro_zerorate: non-finite value");
      return false;
    }
    if (!isfinite(merged.imu_cal.mag_hardiron[i])) {
      set_error("Invalid mag_hardiron: non-finite value");
      return false;
    }
  }

  for (int i = 0; i < 9; i++) {
    if (!isfinite(merged.imu_cal.mag_softiron[i])) {
      set_error("Invalid mag_softiron: non-finite value");
      return false;
    }
  }

  return true;
}

bool DeviceSettingsBuilder::validate() {
  clear_error();
  std::lock_guard<std::timed_mutex> lock(g_settings_mutex);
  DeviceSettingsImpl merged = g_settings_impl;
  apply_changes_to_merged(merged);
  return validate_merged(merged);
}

bool DeviceSettingsBuilder::commit(bool save) {
  if (error_ != nullptr) {
    BLIM_LOG_ERROR("Commit failed: %s\n", error_);
    return false;
  }

  std::lock_guard<std::timed_mutex> lock(g_settings_mutex);

  DeviceSettingsImpl merged = g_settings_impl;
  apply_changes_to_merged(merged);

  if (!validate_merged(merged)) {
    return false;
  }

  if (save) {
    if (!nvs::save_to_nvs(&merged)) {
      set_error("NVS save failed");
      return false;
    }
  }

  g_settings_impl = merged;
  dirty_flags_ = 0;
  clear_error();
  return true;
}

bool DeviceSettingsBuilder::is_modified() const noexcept {
  return dirty_flags_ > 0;
}

const char* DeviceSettingsBuilder::get_last_error() const noexcept {
  return error_;
}

// ============================================================================
// JSON Serialization
// ============================================================================

size_t DeviceSettings::to_json(char* buffer, size_t buffer_size) const {
  if (!buffer || buffer_size == 0) {
    BLIM_LOG_ERROR("to_json: invalid buffer (nullptr or zero size)\n");
    return 0;
  }

  DeviceSettingsImpl snapshot;
  {
    auto locked_impl = impl();
    snapshot = *locked_impl;
  }

  JsonDoc doc;

  JsonObject settings_obj = doc["settings"].to<JsonObject>();
  settings_obj["apply_calibration"] = snapshot.apply_calibration;

  JsonObject imu = doc["imu"].to<JsonObject>();

  JsonArray accel_zerog_arr = imu["accel"]["zerog"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    accel_zerog_arr.add(snapshot.imu_cal.accel_zerog[i]);
  }

  JsonArray gyro_zerorate_arr = imu["gyro"]["zerorate"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    gyro_zerorate_arr.add(snapshot.imu_cal.gyro_zerorate[i]);
  }

  JsonObject mag = imu["mag"].to<JsonObject>();

  JsonArray mag_hardiron_arr = mag["hardiron"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    mag_hardiron_arr.add(snapshot.imu_cal.mag_hardiron[i]);
  }

  mag["field"] = snapshot.imu_cal.mag_field;

  JsonArray softiron = mag["softiron"].to<JsonArray>();
  for (int row = 0; row < 3; row++) {
    JsonArray softiron_row = softiron.add<JsonArray>();
    for (int col = 0; col < 3; col++) {
      softiron_row.add(snapshot.imu_cal.mag_softiron[row * 3 + col]);
    }
  }

  JsonObject metadata = doc["metadata"].to<JsonObject>();
  metadata["version"] = DeviceSettings::SCHEMA_VERSION;
  metadata["timestamp"] = millis();

  size_t required = measureJson(doc);
  if (required >= buffer_size) {
    BLIM_LOG_ERROR("toJson: buffer too small (need %u bytes, have %u)\n",
              static_cast<unsigned int>(required + 1),
              static_cast<unsigned int>(buffer_size));
    return 0;
  }

  size_t written = serializeJson(doc, buffer, buffer_size);

  if (written != required) {
    BLIM_LOG_ERROR("toJson: serialization size mismatch (expected %u, got %u)\n",
              static_cast<unsigned int>(required),
              static_cast<unsigned int>(written));
    buffer[0] = '\0';
    return 0;
  }

  return written;
}