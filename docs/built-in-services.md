# Built-in Services

BLEX provides ready-to-use implementations of standard BLE services with zero-configuration setup and build-time automation.

## Device Information Service

Standard Bluetooth Device Information Service (UUID: 0x180A) with automated build-time configuration:

```cpp
#include "services/device_info.hpp"

MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
```

### BLE Characteristics Exposed

| Characteristic UUID | Define Used | Description |
|---------------------|-------------|-------------|
| 0x2A29 (Manufacturer Name) | `MANUFACTURER_NAME` | Company name |
| 0x2A27 (Hardware Revision) | `HARDWARE_VERSION` | Hardware version string |
| 0x2A24 (Model Number) | `MODEL_NUMBER` | Board type (from PlatformIO) |
| 0x2A26 (Firmware Revision) | `FIRMWARE_VERSION` | Semantic version (from Git tags) |
| 0x2A28 (Software Revision) | `SOFTWARE_REVISION` | Git commit hash |
| 0x2A25 (Serial Number) | `SERIAL_NUMBER` | Device serial (BLE MAC without colons) |

### Automated Configuration

All defines are automatically generated at build-time by [`version.py`](scripts.md#versionpy---automated-versioning--ble-device-configuration), which uses PlatformIO environment variables (`BUILD_DIR`, `PROJECT_DIR`, `BOARD`, `UPLOAD_PORT`) to detect device identity, read MAC addresses via esptool, and inject configuration from `device_config.json`.

See [Build Scripts Reference](scripts.md#versionpy---automated-versioning--ble-device-configuration) for complete documentation on automated configuration and environment variables.

## OTA/DFU Service

Nordic DFU-compatible over-the-air firmware update service:

```cpp
#include "services/ota.hpp"

MyBlex::PassiveAdvService<ota::OtaService<MyBlex>>
```

### Features

- Resumable updates after power loss or disconnect
- CRC32 verification with incremental computation
- NVS-backed progress persistence (every 32KB)
- Packet receipt notifications (PRN) for reliability
- Compatible with Nordic DFU protocol

### PlatformIO Integration

The `platformio.ini` includes pre-configured environments for OTA updates:

```bash
# First flash: Upload basic_server via USB to enable OTA support
pio run -e basic_server -t upload
```

**Important:** The device must first be flashed via USB with an OTA-enabled firmware (e.g., `basic_server`) before BLE OTA updates can be used.

```bash
# Subsequent updates: Build and upload via BLE OTA (by device address)
PLATFORMIO_UPLOAD_FLAGS="-d YOUR_DEVICE_ADDRESS" pio run -e ota_server -t upload

# Or by device name
PLATFORMIO_UPLOAD_FLAGS="-n DeviceName" pio run -e ota_server -t upload

# Upload only (skip rebuild if firmware already built)
PLATFORMIO_UPLOAD_FLAGS="-d YOUR_DEVICE_ADDRESS" pio run -e ota_server -t nobuild -t upload
```

The `ota_server` environment extends `basic_server` and uses the `dfu_upload` protocol, which invokes `dfu-ble-uploader.py` automatically.

### Python Upload Tool

```bash
# Upload firmware via BLE
python scripts/dfu-ble-uploader.py firmware.bin -d aa:bb:cc:dd:ee:ff
# Or by device name
python scripts/dfu-ble-uploader.py firmware.bin -n "MyDevice"
```

See [Build Scripts Reference](scripts.md) for complete documentation.