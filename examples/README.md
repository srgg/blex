# BLEX Examples

Comprehensive examples demonstrating BLEX library capabilities, from minimal configurations to production-ready sensor streaming.

## Quick Start

### Prerequisites

- **PlatformIO CLI** installed
- **ESP32-S3** development board (tested: FeatherS3 by Unexpected Maker)
- **USB cable** for programming and serial monitor
- **BLE scanner app**: [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) or [LightBlue](https://punchthrough.com/lightblue/)

### Building and Uploading

```bash
# Upload firmware to device
pio run -e <example-name> -t upload

# Open serial monitor
pio device monitor -e <example-name>
```

**Available example:**

| Example | Complexity | Hardware Required | Description                                                                           |
|---------|-----------|-------------------|---------------------------------------------------------------------------------------|
| **simple_server** | Beginner | ESP32-S3 only | Minimal BLE peripheral - single line device declaration                               |
| **basic_server** | Intermediate | ESP32-S3 only | Essential BLE features: advertising, connections, callbacks, standard characteristics |
| **imu_streamer** | Advanced | ESP32-S3 + 9-DOF IMU | Real-time sensor streaming with over-the-air (OTA) updates                            |


### Example: Running basic_server

```bash
# Upload
pio run -e basic_server -t upload

# Monitor serial output
pio device monitor -e basic_server
```

**Expected output:**
```
Blex Basic Peripheral Example
BLE server started:
     Device Name: Blex Basic Peripheral
     Device Address: a4:cf:12:xx:xx:xx

     Services:
         Device Information (0x180A)
         Environmental Sensing (0x181A)
             - Temperature (0x2A6E): read, notify

Waiting for connection..
```

**Connect with nRF Connect:**
1. Scan for "BlexBasic"
2. Connect to device
3. Discover services
4. Read/enable notifications on Temperature characteristic (0x2A6E)

## Example Details

### 1. simple_server - Minimal BLE Peripheral

**Lines of code:** 19
**Demonstrates:** Zero-configuration BLEX API

Single-line device declaration with default settings. Perfect starting point for understanding BLEX basics.

[→ See detailed README](simple_server/README.md)

### 2. basic_server - Essential BLE Features

**Lines of code:** 147
**Demonstrates:**
- Advertising configuration (TX power, intervals, appearance, manufacturer data)
- Connection parameters (MTU, intervals, timeout)
- Server callbacks (connect, disconnect)
- Standard BLE services and characteristics
- Device Information Service

Temperature sensor simulation with notifications and connection management.

[→ See detailed README](basic_server/README.md)

### 3. imu_streamer - Production Sensor Streaming

**Lines of code:** 230
**Hardware:** 9-DOF IMU (auto-detects LSM6DSOX, LIS3MDL, and others)
**Demonstrates:**
- Hardware abstraction with Adafruit SensorLab
- Custom BLE services with proper GATT descriptors
- High-frequency data streaming (50Hz, 36 bytes/sample)
- Graceful degradation (magnetometer optional)
- Security configuration (MITM, bonding, secure connections)
- BLE Presentation Format descriptors

Real-time 9-axis IMU streaming for motion capture and sensor fusion applications.

[→ See detailed README](imu_streamer/README.md)

## Troubleshooting

### Device not found in BLE scanner

- Check serial output for device address and name
- Ensure device is advertising (check "Waiting for connection" message)
- Some phones require location permissions for BLE scanning
- Try restarting the device

### Upload fails

```bash
# List connected devices
pio device list

# Specify upload port explicitly
pio run -e <example-name> -t upload --upload-port /dev/cu.usbserial-xxxxx
```

### Serial monitor shows garbage characters

- Ensure baud rate matches: `115200` (set in all examples)
- Press the reset button on the device after opening the monitor
- Check the USB cable supports data (not just charging)

### IMU not detected (imu_streamer only)

- Verify I2C wiring: SDA=GPIO8, SCL=GPIO9 (FeatherS3)
- Check sensor power (3.3V)
- Enable I2C pull-up resistors if using breakout boards
- See serial output for supported sensor list

## BLE Scanning Tools

**Mobile:**
- [nRF Connect (Android/iOS)](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) - Recommended
- [LightBlue (iOS)](https://punchthrough.com/lightblue/)

**Desktop:**
- [nRF Connect (Windows/macOS/Linux)](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-desktop)
- `bluetoothctl` (Linux command-line)

## Additional Resources

- [BLEX Documentation](../README.md)
- [NimBLE-Arduino Documentation](https://github.com/h2zero/NimBLE-Arduino)
- [Bluetooth SIG Specifications](https://www.bluetooth.com/specifications/specs/)
- [GATT Service & Characteristic UUIDs](https://www.bluetooth.com/specifications/assigned-numbers/)