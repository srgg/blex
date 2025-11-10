/*
 * Device Settings Data and State Management
 *
 * Defines core data structures for device configuration and provides
 * API for managing device settings with persistent storage.
 *
 * Memory: Static allocation only, no heap usage
 * Storage: ESP32 NVS
 */

#ifndef DEVICE_SETTINGS_H
#define DEVICE_SETTINGS_H

#include <stddef.h>
#include <stdint.h>

// IMU calibration parameters (MotionCal format)
 struct ImuCalibration {
     float accel_zerog[3];      // m/s²
     float gyro_zerorate[3];    // rad/s or dps
     float mag_hardiron[3];     // µT
     float mag_field;           // µT
     float mag_softiron[9];     // 3×3 row-major
 };
// Forward declarations
struct DeviceSettingsImpl;
class DeviceSettingsBuilder;

/**
 * Device Settings (NVS-backed persistent configuration)
 *
 * Access: DeviceSettings::get() (read-only)
 * Modify: DeviceSettings::modify().setField(value).commit()
 */
struct DeviceSettings {
  static constexpr uint32_t SCHEMA_VERSION = 1;  // JSON schema version

  // ============================================================================
  // Fluent Read Helpers
  // ============================================================================

    /** @return true if calibration is enabled */
    bool is_calibration_enabled() const;

    /** @return Accelerometer zero-g offset [X, Y, Z] */
    const float* get_accel_zero_g() const;

    /** @return Gyroscope zero-rate offset [X, Y, Z] */
    const float* get_gyro_zero_rate() const;

    /** @return Magnetometer hard-iron offset [X, Y, Z] */
    const float* get_mag_hard_iron() const;

    /** @return Magnetic field strength (µT) */
    float get_mag_field() const;

    /** @return Magnetometer soft-iron matrix (3×3 row-major, 9 floats) */
    const float* get_mag_soft_iron() const;

    /**
     * RAII lock for thread-safe read access (1000ms timeout, logs if blocked)
     */
    class ImplLock {
    public:
        ImplLock() noexcept;
        ~ImplLock() noexcept;

        ImplLock(const ImplLock&) = delete;
        ImplLock& operator=(const ImplLock&) = delete;

        ImplLock(ImplLock&& other) noexcept;
        ImplLock& operator=(ImplLock&& other) noexcept;

        const DeviceSettingsImpl* operator->() const noexcept;
        const DeviceSettingsImpl& operator*() const noexcept;

    private:
        bool owns_lock_;
    };

    [[nodiscard]] ImplLock impl() const noexcept;

  // ============================================================================
  // Singleton Accessors
  // ============================================================================

  /** @return Current runtime settings (read-only) */
  static const DeviceSettings& get();

  /** @return Builder for modifying settings (call commit() to persist) */
  static DeviceSettingsBuilder modify();

  /**
   * Serialize current settings to JSON
   * @return Number of bytes written (0 on error)
   */
  [[nodiscard]] size_t to_json(char* buffer, size_t buffer_size) const;

 private:
    // Singleton pattern - prevent direct construction and copying
    // NOLINTBEGIN
    DeviceSettings() = default;
    ~DeviceSettings() = default;
    DeviceSettings(const DeviceSettings&) = delete;
    DeviceSettings(DeviceSettings&&) = delete;
    DeviceSettings& operator=(const DeviceSettings&) = delete;
    DeviceSettings& operator=(DeviceSettings&&) = delete;
    // NOLINTEND
};

/**
 * Builder for partial device settings updates
 *
 * Transactional: changes applied atomically on commit() (validate → NVS → runtime).
 * Thread safety: last-writer-wins for same field, non-overlapping fields preserved.
 */
class DeviceSettingsBuilder {
 public:
  // ---- IMU Calibration Setters ----
  DeviceSettingsBuilder& set_accel_zero_g(const float v[3]);
  DeviceSettingsBuilder& set_gyro_zero_rate(const float v[3]);
  DeviceSettingsBuilder& set_mag_hard_iron(const float v[3]);
  DeviceSettingsBuilder& set_mag_field(float field);
  DeviceSettingsBuilder& set_mag_soft_iron(const float matrix[9]);

  // ---- Settings State Setters ----
  DeviceSettingsBuilder& set_apply_calibration(bool apply);

  // Storage size for DeviceSettingsImpl (verified by static_assert in .cpp)
  static constexpr size_t IMPL_STORAGE_SIZE = 128;

  // ---- JSON API (partial merge) ----
  /**
   * Merge JSON (partial updates supported)
   * @return Builder for chaining
   */
  DeviceSettingsBuilder& merge_json(const char* json);

  // ---- Reset ----
  /**
   * Reset to factory defaults (true) or reload from NVS (false)
   * @return Builder for chaining (call commit() to persist)
   */
  DeviceSettingsBuilder& reset(bool factoryReset);

  // ---- Validation ----
  /**
   * Validate current settings state
   * @return true if valid, false otherwise (check get_last_error())
   */
  [[nodiscard]] bool validate();

  /**
   * Transactional commit: validate → save to NVS → swap into runtime
   * @param save If true, persist to NVS before applying to runtime
   * @return true if successful, false otherwise (check get_last_error())
   */
  [[nodiscard]] bool commit(bool save = true);

  /** @return true if any modifications were made */
  bool is_modified() const noexcept;

  /** @return Error message string, or nullptr if no error */
  const char* get_last_error() const noexcept;

  // Public destructor (required for RAII/temporaries)
  ~DeviceSettingsBuilder();

 private:
  friend DeviceSettingsBuilder DeviceSettings::modify();
  DeviceSettingsBuilder();

  // Prevent copying - builder is created via DeviceSettings::modify()
  // NOLINTBEGIN
  DeviceSettingsBuilder(const DeviceSettingsBuilder&) = delete;
  DeviceSettingsBuilder& operator=(const DeviceSettingsBuilder&) = delete;

  // Allow move (private, only accessible via modify())
  DeviceSettingsBuilder(DeviceSettingsBuilder&& other) noexcept;
  DeviceSettingsBuilder& operator=(DeviceSettingsBuilder&& other) noexcept;
  // NOLINTEND

  void set_error(const char* error);
  void clear_error();

  // Apply dirty changes to a merged DeviceSettingsImpl (used by validate/commit)
  void apply_changes_to_merged(DeviceSettingsImpl& merged) const;
  bool validate_merged(const DeviceSettingsImpl& merged);

  // Dirty flags for tracking which fields were modified
  enum DirtyBits : uint8_t {
    DIRTY_ACCEL_ZEROG       = 0x01,
    DIRTY_GYRO_ZERORATE     = 0x02,
    DIRTY_MAG_HARDIRON      = 0x04,
    DIRTY_MAG_FIELD         = 0x08,
    DIRTY_MAG_SOFTIRON      = 0x10,
    DIRTY_APPLY_CALIBRATION = 0x20
  };

  DeviceSettingsImpl* working_copy_;  // PImpl pointer to internal storage
  alignas(8) uint8_t storage_[IMPL_STORAGE_SIZE];  // Stack storage for impl (no heap)
  uint8_t dirty_flags_;               // Bitfield tracking which fields were modified
  const char* error_;                 // Error message (nullptr = no error)
};

#endif // DEVICE_SETTINGS_H