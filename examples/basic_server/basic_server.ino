#include <Arduino.h>
#include <NimBLEDevice.h>
#include <blex.hpp>

// Forward declarations for callbacks (declared after includes so NimBLEConnInfo is available)
static void onConnect(NimBLEConnInfo& conn);
static void onDisconnect(NimBLEConnInfo& conn, int reason);

// Define your device
inline constexpr char deviceName[] = "MyDevice";
inline constexpr char deviceNameShort[] = "MyDev";

// Choose a lock policy
using MyBlex = blexDefault;  // Thread-safe for multi-core
// using MyBlex = blex<NoLock>;  // Zero-overhead for single-core

// Declare a BLE peripheral device using fluent API
using MyDevice = MyBlex::Server<
    deviceName,
    deviceNameShort,

    // // Advertising Configuration
    // MyBlex::AdvertisingConfig<>
    //     ::WithTxPower<9>
    //     ::WithAppearance<MyBlex::BleAppearance::kGenericComputer>
    //     ::WithIntervals<100, 200>,

    // // Connection Configuration
    // MyBlex::ConnectionConfig<>
    //     ::WithMTU<517>
    //     ::WithIntervals<12, 24>
    //     ::WithLatency<0>
    //     ::WithTimeout<4000>,
    //
    // // Security Configuration
    // MyBlex::SecurityConfig<>
    //     ::WithIOCapabilities<DisplayYesNo>
    //     ::WithMITM<true>
    //     ::WithBonding<true>
    //     ::WithSecureConnections<true>
    //     ::WithPasskey<123456>,

    // Server Callbacks
    MyBlex::ServerCallbacks<>
        ::WithOnConnect<onConnect>
        ::WithOnDisconnect<onDisconnect>,

    // Services
    MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
>;

// Callback implementations
static void onConnect(NimBLEConnInfo& conn) {
    Serial.printf("Connected: %s\n", conn.getAddress().toString().c_str());
}

static void onDisconnect(NimBLEConnInfo& conn, int reason) {
    Serial.printf("Disconnected: reason=%d\n", reason);
    NimBLEDevice::startAdvertising();
}

void setup() {
    Serial.begin(115200);

    // Initialize BLE server
    if (!MyDevice::init()) {
        Serial.println("BLE initialization failed!");
        return;
    }

    Serial.println("BLE server started");
}

void loop() {
    delay(1000);
}