/**
 * @file basic_server.ino
 * @brief Basic BLE peripheral example with essential BLEX configuration
 *
 * @details
 * Demonstrates:
 * - Advertising configuration (TX power, intervals, appearance, manufacturer data)
 * - Connection configuration (MTU, intervals)
 * - Server callbacks (connect, disconnect)
 * - Standard BLE characteristics (temperature with user description descriptor)
 * - Device Information Service
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <blex.hpp>
#include <services/device_info.hpp>


// ============================================================================
// Application State
// ============================================================================
static constexpr float TEMP_INITIAL = 25.0f;
static constexpr float TEMP_MIN = 20.0f;
static constexpr float TEMP_MAX = 30.0f;
static constexpr float TEMP_CHANGE_MAX = 1.0f;  // Max change per update: ±1.0°C
static constexpr uint32_t UPDATE_INTERVAL_MS = 5000;  // Update every 5 seconds

static float temperature = TEMP_INITIAL;

// ============================================================================
// Characteristic callbacks
// ============================================================================
static void onTemperatureRead(float& value) {
    value = temperature;
    Serial.printf("Temperature read: %.1f°C\n", value);
}

// ============================================================================
// Server callbacks
// ============================================================================

static void onConnect(const NimBLEConnInfo& conn) {
    Serial.printf("Connected: %s\n", conn.getAddress().toString().c_str());
}

static void onDisconnect(const NimBLEConnInfo& conn, const int reason) {
    Serial.printf("Disconnected: %s (reason: %d)\n\n"
        "Waiting for connection..\n",
        conn.getAddress().toString().c_str(),
        reason
    );
    NimBLEDevice::startAdvertising();
}

// ============================================================================
// BLE Server Configuration
// ============================================================================
inline constexpr char deviceName[] = "BlexBasic";
inline constexpr char deviceNameLong[] = "Blex Basic Peripheral";

/// User Description text for temperature characteristic
static constexpr char TEMP_DESC_TEXT[] = "Ambient temperature sensor reading in Celsius";

using MyBlex = blexDefault;

/// Temperature characteristic (standard BLE UUID 0x2A6E)
/// Includes UserDescription and PresentationFormat descriptors
using TempChar = MyBlex::Characteristic<
    float,
    0x2A6E,
    MyBlex::Permissions<>::AllowRead::AllowNotify,
    MyBlex::CharacteristicCallbacks<>::WithOnRead<onTemperatureRead>,
    blex_standard::descriptors::UserDescription<TEMP_DESC_TEXT>,
    blex_standard::descriptors::PresentationFormat<
        MyBlex::GattFormat::kFloat32,       // IEEE-754 32-bit float
        0,                                   // Exponent: 10^0 (no scaling)
        MyBlex::GattUnit::kDegreeCelsius    // Unit: °C (Namespace and Description use defaults)
    >
>;

/// Environmental Sensing Service (standard BLE UUID 0x181A)
using EnvironmentalService = MyBlex::Service<
    0x181A,
    TempChar
>;

using MyDevice = MyBlex::Server<
    deviceName,

    // Advertising Configuration
    MyBlex::AdvertisementConfig<>
        ::WithLongName<deviceNameLong>
        ::WithManufacturerData<>
            ::WithManufacturerId<0xFFFE>        // Unofficial Blim company id
            ::WithDeviceType<0xFE>              // uint8_t device type
        ::WithTxPower<3>                        // TX power: 3 dBm
        ::WithAppearance<MyBlex::BleAppearance::kSensor>
        ::WithIntervals<100, 200>,              // Advertising: 100-200ms


    // Connection Configuration
    MyBlex::ConnectionConfig<>
        ::WithMTU<517>                      // MTU: 517 bytes (BLE 4.2+ max)
        // ::WithIntervals<100, 200>        // Connection: 100-200ms (slow, not for OTA)
        ::WithIntervals<8, 15>              // Connection: 8-15ms (fast for OTA, 7.5ms is BLE min)
        ::WithTimeout<4000>,                // Timeout: 4s

    // Server Callbacks
    MyBlex::ServerCallbacks<>
        ::WithOnConnect<onConnect>
        ::WithOnDisconnect<onDisconnect>,

    // Services
    MyBlex::PassiveAdvService<DeviceInfoService<MyBlex>>,       // out-of-the-box Device Info service
    MyBlex::ActiveAdvService<ota::OtaService<MyBlex>>,          // out-of-the-box OTA/DFY service
    MyBlex::ActiveAdvService<EnvironmentalService>
>;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Blex Basic Peripheral Example");

    // startAllServices() auto-initializes if needed
    if (!MyDevice::startAllServices()) {
        Serial.println("ERROR: BLE initialization/service start failed");
        return;
    }

    Serial.printf("BLE server started:\n"
                  "     Device Name: %s\n"
                  "     Device Address: %s\n\n"
                  "     Services:\n"
                  "         Device Information (0x180A)\n"
                  "         Environmental Sensing (0x181A)\n"
                  "             - Temperature (0x2A6E): read, notify\n\n"
                  "Waiting for connection..\n",
                  deviceNameLong,
                  MyDevice::getAddress());
}

void loop() {
    // Simulate temperature changes: random walk within ±TEMP_CHANGE_MAX
    const float change = static_cast<float>(random(-10, 11)) / 10.0f * TEMP_CHANGE_MAX;
    temperature += change;
    temperature = constrain(temperature, TEMP_MIN, TEMP_MAX);

    // // Notify connected clients
    // if (MyDevice::isConnected()) {
    //     TempChar::setValue(temperature);
    //     Serial.printf("Temperature updated: %.1f°C\n", temperature);
    // }
    //
    // // Print active connections
    // const auto connections = MyDevice::getConnections();
    // Serial.printf("\nActive connections: %u\n", connections.size());
    // for (const auto& conn : connections) {
    //     Serial.printf("  - %s (RSSI: %d dBm)\n",
    //                   conn.getAddress().toString().c_str(),
    //                   MyDevice::getRSSI(conn.getConnHandle()));
    // }
    // Serial.println();
    //Let
    // delay(UPDATE_INTERVAL_MS);
    delay(1);
}