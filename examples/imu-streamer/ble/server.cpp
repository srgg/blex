#include <Arduino.h>


// Uncomment for NoLock policy (single-core/pinned)
// #define BLE_ON_SINGLE_CORE

#define CONFIG_BT_NIMBLE_ROLE_OBSERVER_DISABLED
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED
#define CONFIG_BT_NIMBLE_ROLE_BROADCASTER_DISABLE

#ifdef BLE_ON_SINGLE_CORE
    #define CONFIG_BT_NIMBLE_PINNED_TO_CORE   ARDUINO_RUNNING_CORE
#endif

#include <NimBLEDevice.h>


#include "services/manufacturer_data.hpp"
#include "services/device_info.hpp"
#include "device_settings_service.hpp"
#include "imu_servce.hpp"
#include "services/ota.hpp"

// Example: https://github.com/h2zero/esp-nimble-cpp/blob/master/examples/NimBLE_Server/main/main.cpp

// Device name variables with external linkage for template parameters
inline constexpr char deviceName[] = DEVICE_NAME;
inline constexpr char deviceNameShort[] = DEVICE_NAME_SHORT;

// ---------------------- Lock Policy Selection ----------------------
// Choose lock policy based on deployment configuration
#ifdef BLE_ON_SINGLE_CORE
    using blim = blex<NoLock>;  // Zero overhead for single-core/pinned execution
#else
    using blim = blexDefault;   // Thread-safe for multi-core concurrent access
#endif

// ---------------------- Server Callbacks ----------------------

static void onConnect(NimBLEConnInfo& conn) {
    Serial.printf("🔗 Device connected: %s\n", conn.getAddress().toString().c_str());
    Serial.printf("   Connection ID: %u\n", conn.getConnHandle());
    Serial.printf("   MTU: %u bytes\n", conn.getMTU());
}

static void onDisconnect(NimBLEConnInfo& conn, int reason) {
    Serial.printf("❌ Device disconnected: %s (reason=%d)\n",
                  conn.getAddress().toString().c_str(), reason);

    // Auto-restart advertising
    NimBLEDevice::startAdvertising();
    Serial.println("📡 Advertising restarted");
}

// ---------------------- BLE Server Configuration ----------------------
using ImuDevice = blim::Server<
    deviceName,
    deviceNameShort,
    blim::AdvertisingConfig<>
        ::WithTxPower<9>
        ::WithAppearance<blim::BleAppearance::kSensor>
        ::WithIntervals<120, 140>
        ::WithManufacturerData<blimco::ManufacturerData<>::data>,
    blim::ConnectionConfig<>
        ::WithMTU<517>
        ::WithIntervals<8, 10>        // 8-10ms - optimized for DFU throughput
        ::WithLatency<0>
        ::WithTimeout<4000>,          // 4s
    blim::SecurityConfig<>
        ::WithIOCapabilities<DisplayYesNo>
        ::WithMITM<true>
        ::WithBonding<true>
        ::WithSecureConnections<true>
        ::WithPasskey<123456>,            // Static 6-digit passkey for pairing
    blim::ServerCallbacks<>
        ::WithOnConnect<onConnect>
        ::WithOnDisconnect<onDisconnect>,
    blim::PassiveAdvService<DeviceSettingsService<blim>>,
    blim::ActiveAdvService<DeviceInfoService<blim>>,
    IMUService<blim>,
    blim::PassiveAdvService<OtaService<blim>>
>;

bool setup_ble() {
    // Verify NimBLE logging is configured
    #ifdef CONFIG_NIMBLE_CPP_LOG_LEVEL
        Serial.printf("[BLE] CONFIG_NIMBLE_CPP_LOG_LEVEL = %d\n", CONFIG_NIMBLE_CPP_LOG_LEVEL);
    #else
        Serial.println("[BLE] WARNING: CONFIG_NIMBLE_CPP_LOG_LEVEL not defined!")       ;
    #endif

    bool success = ImuDevice::init();
    if (!success) {
        BLIM_LOG_ERROR("BLE initialization failed\n");
        return false;
    }
    return true;
}

void update_imu(const float (&data)[9]) {
    IMUService<blim>::IMUChar::setValue(data);
}