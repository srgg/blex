/**
 * @file simple_server.ino
 * @brief Minimal BLE peripheral example demonstrating zero-configuration BLEX usage
 *
 * @details
 * Demonstrates the simplest possible BLE peripheral - a single line of code
 * creates a fully functional BLE server with default settings. No callbacks,
 * no services, just a discoverable device.
 */

#include <Arduino.h>
// ReSharper disable once CppUnusedIncludeDirective
#include <NimBLEDevice.h>
#include <blex.hpp>

inline constexpr char shortDeviceName[] = "SimpleBlex";

// Declare a BLE peripheral device in a single line of code
using PeripheralDevice = blexDefault::Server<shortDeviceName>;

void setup() {
    Serial.begin(115200);
    delay(500);

    // Initialize BLE server
    if (!PeripheralDevice::startAllServices()) {
        Serial.println("Blex failed initialize PeripheralDevice!");
        return;
    }

    Serial.println("Blex simple peripheral device started!");
}

void loop() {
    delay(1000);
}