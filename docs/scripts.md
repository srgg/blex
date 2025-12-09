# Build Scripts Reference

BLEX comes with the three core build scripts that integrate with PlatformIO to automate:
- **Semantic versioning** from Git tags - [version.py](#versionpy---automated-versioning--ble-device-configuration)
- **Device-specific configuration** management - [version.py](#versionpy---automated-versioning--ble-device-configuration)
- **C++ compiler flags** for embedded optimization - [cpp-build-flags.py](#cpp-build-flagspy---c-compiler-configuration)
- **Over-the-air firmware updates** via Bluetooth - [dfu-ble-uploader.py](#dfu-ble-uploaderpy---ble-firmware-upload-tool)

All scripts could be configured to run automatically during PlatformIO build phases unless invoked manually.

## version.py - Automated Versioning & BLE Device Configuration

**Location:** `scripts/version.py`
**Runs:** Pre-build phase (automatic)
**Configured in:** `platformio.ini` line 37

### Problem Statement

Embedded BLE devices require consistent device identification and versioning across firmware builds:
- **Version management:** Manual version updates in code are error-prone
- **Device identity:** Each hardware unit needs unique MAC addresses and serial numbers
- **Build reproducibility:** Builds must inject the correct configuration without manual edits
- **Multi-device support:** Different boards require different configurations

### What It Does

1. **Git-based versioning:**
   - Extracts semantic version from the latest Git tag (v1.2.3 format)
   - Generates short commit hash with dirty-tree detection
   - Fallback to `0.0.0-dev` if no tags exist

2. **Device MAC detection:**
   - Reads ESP32 WiFi MAC via esptool
   - Calculates BLE MAC using ESP32 hardware convention (WiFi MAC + 2)
   - Caches results in PlatformIO's build directory (via `BUILD_DIR` env var)

3. **Device configuration mapping:**
   - Loads `device_config.json` from project root
   - Maps detected MAC addresses to device-specific settings
   - Auto-generates defaults if device not in config

4. **C++ define injection:**
   - Injects version, manufacturer, MAC addresses as C++ defines
   - Makes values available to Device Information Service (DIS)
   - Sets `BLE_DEVICE_ADDRESS` environment variable for DFU uploads

### Configuration File Format

Create `device_config.json` in the project root:

```json
{
  "manufacturer": "Your Company Name",
  "manufacturer_id": 65534,
  "hardware_version": "1.0",
  "devices": {
    "ecda3b5c836a": {
      "serial_number": "DEV-001",
      "ble_address": "ec:da:3b:5c:83:6a",
      "ble_device": "e20e664a4716aba3abc6b9a0329b5b2e",
      "wifi_address": "ec:da:3b:5c:83:68",
      "name": "MyDevice",
      "long_name": "My BLE Device"
    }
  }
}
```

**Field descriptions:**
- `manufacturer`: Company name (shows in BLE Device Information Service)
- `manufacturer_id`: 16-bit company ID (use 0xFFFE for unofficial/testing)
- `hardware_version`: Hardware revision string
- `devices`: Map of device serial numbers (MAC without colons) to configurations
  - `serial_number`: Human-readable serial number
  - `ble_address`: BLE MAC address
  - `ble_device`: Optional BLE random private address (for privacy)
  - `wifi_address`: WiFi MAC address
  - `name`: Short BLE device name (max 20 chars)
  - `long_name`: Optional full device name

### Automatic Behavior

If `device_config.json` is missing or the device not found:
- Prints a warning in the build output
- Uses auto-generated defaults:
  - Manufacturer: None (not included in DIS)
  - Device name: `BX-<last4>` (using last 4 chars of MAC)
  - Serial number: MAC address without colons

### Build Output Example

```
============================================================
Device Configuration:
  Board Type:        um_feathers3
  Manufacturer:      BLIMCo
  Manufacturer ID:   0xfffe
  BLE MAC:           ec:da:3b:5c:83:6a
  BLE Device Addr:   e20e664a4716aba3abc6b9a0329b5b2e
  WiFi MAC:          ec:da:3b:5c:83:68
  Serial Number:     ecda3b5c836a
  Hardware Version:  1.0
  Firmware Version:  1.0.0
  Software Revision: 48687d0-dirty
  Device Name:       D1
  Long Name:         BLIM IMU Stream
============================================================
```

### Generated/Injected C Defines

Available in all source files during the build:

```cpp
FIRMWARE_VERSION    // "1.0.0"
SOFTWARE_REVISION   // "48687d0" or "48687d0-dirty"
MODEL_NUMBER        // Board type from platformio.ini
MANUFACTURER_NAME   // From device_config.json
MANUFACTURER_ID     // From device_config.json (numeric)
HARDWARE_VERSION    // From device_config.json
SERIAL_NUMBER       // BLE MAC without colons
DEVICE_BLE_MAC      // "aa:bb:cc:dd:ee:ff"
DEVICE_WIFI_MAC     // "aa:bb:cc:dd:ee:ff"
DEVICE_NAME         // Short name
DEVICE_NAME_LONG    // Long name (optional)
DEVICE_BLE_ADDR     // Random private address (optional)
```

### Manual Invocation

Not recommended (runs automatically), but possible:

```bash
pio run -e <env_name>  # Triggers pre-build script
```

## dfu-ble-uploader.py - BLE Firmware Upload Tool

**Location:** `scripts/dfu-ble-uploader.py`
**Runs:** Manual or via PlatformIO custom upload
**Protocol:** Nordic DFU over BLE

### Problem Statement

Firmware updates over USB require physical access to devices:
- **Field updates:** Deployed devices need wireless firmware updates
- **Development speed:** USB cable swapping slows iteration
- **Device access:** Some devices are physically inaccessible after deployment
- **Multi-device updates:** USB updates don't scale to multiple units

### What It Does

1. **Nordic DFU Protocol Implementation:**
   - Implements Nordic DFU over BLE (UUID 0000fe59-0000-1000-8000-00805f9b34fb)
   - Transfers firmware binary in 512-byte packets
   - CRC32 verification with incremental computation
   - Packet Receipt Notifications (PRN) for reliability

2. **Reliable Transfer:**
   - Connection retry logic (max 3 attempts)
   - Automatic reconnection on connection loss
   - Packet-level acknowledgment
   - Progress reporting

3. **Dependency Management:**
   - Auto-installs `bleak` BLE library if missing
   - Auto-detects and relaunches in PlatformIO venv
   - Zero manual dependency setup required

4. **Device Selection:**
   - By BLE MAC address (`-d aa:bb:cc:dd:ee:ff`)
   - By device name scan (`-n "DeviceName"`)
   - Via `BLE_DEVICE_ADDRESS` environment variable (set by version.py)

### Usage

#### Direct Command-Line Usage

```bash
# Upload by MAC address
python dfu-ble-uploader.py firmware.bin -d ec:da:3b:5c:83:6a

# Upload by device name (will perform scan)
python dfu-ble-uploader.py firmware.bin -n "MyDevice"

# Adjust packet receipt notification interval (default: 32 packets)
python dfu-ble-uploader.py firmware.bin -d ec:da:3b:5c:83:6a -p 8

# Verbose logging
python dfu-ble-uploader.py firmware.bin -d ec:da:3b:5c:83:6a -v
```

#### PlatformIO Integration

Add to `platformio.ini`:

```ini
[env:my_device]
upload_protocol = custom
upload_command = scripts/dfu-ble-uploader.py $SOURCE
```

Then upload via PlatformIO:

```bash
pio run -e my_device -t upload
```

**Note:** Requires `BLE_DEVICE_ADDRESS` environment variable (automatically set by version.py if device has `ble_device` field in device_config.json).

### Command-Line Arguments

```
positional arguments:
  firmware_path         Path to firmware binary (.bin file)

optional arguments:
  -d, --device ADDR     BLE device MAC address
  -n, --name NAME       BLE device name (triggers scan)
  -p, --prn-interval N  Packet receipt notification interval (default: 32)
  -v, --verbose         Enable verbose logging
```

**Note:** Either `-d` or `-n` required unless `BLE_DEVICE_ADDRESS` environment variable is set.

### Exit Codes

- `0` - Success
- `1` - Device not found
- `2` - Connection failed
- `3` - DFU control point write failed
- `4` - CRC verification failed
- `5` - Connection lost during transfer
- `6` - User interrupted (Ctrl+C)
- `7` - Invalid arguments

### Transfer Process

1. **Discovery:** Scan for the device, skipped if device address provided
2. **Connection:** Connect to a BLE device
3. **MTU Negotiation:** Request maximum MTU for faster transfers
4. **Transfer:**
   - Send firmware size and CRC to the DFU control point
   - Transfer firmware in 512-byte packets
   - Wait for packet receipt notifications every N packets
5. **Verification:** Device verifies CRC and activates new firmware
6. **Completion:** Disconnect and report success

### Example Output

```
14:32:10,123 INFO [dfu_cli] Connecting to device ec:da:3b:5c:83:6a...
14:32:11,456 INFO [dfu_cli] Connected successfully
14:32:11,789 INFO [dfu_cli] Starting DFU upload...
14:32:11,800 INFO [dfu_cli] Firmware size: 523456 bytes, CRC32: 0x1a2b3c4d
14:32:12,100 INFO [dfu_cli] Transfer progress: 512/523456 bytes (0%)
14:32:15,200 INFO [dfu_cli] Transfer progress: 262144/523456 bytes (50%)
14:32:18,500 INFO [dfu_cli] Transfer complete: 523456/523456 bytes (100%)
14:32:18,900 INFO [dfu_cli] DFU upload successful!
```

### Troubleshooting

**Device not found:**
- Ensure the device is powered on and advertising
- Check the device name/address is correct
- Try scanning: `python scripts/dfu-ble-uploader.py firmware.bin -n "MyDevice" -v`

**Connection timeout:**
- Device may be too far away (weak signal)
- Bluetooth adapter may be busy (close other BLE apps)
- Try increasing timeout by reducing a PRN interval: `-p 8`

**CRC verification failed:**
- Firmware corruption during transfer (retry upload)
- MTU negotiation failed (retry with lower MTU)

**Connection lost during transfer:**
- Device reset unexpectedly (check device logs)
- Signal too weak (move closer)
- Automatic retry will attempt reconnection

## cpp-build-flags.py - C++ Compiler Configuration

**Location:** `scripts/cpp-build-flags.py`
**Runs:** Pre-build phase (automatic)
**Configured in:** `platformio.ini` line 38

### Problem Statement

PlatformIO applies global build flags to both C and C++ files:
- **Incompatible flags:** C++-only flags trigger warnings on .c files
- **Third-party noise:** Library warnings pollute build output
- **Optimization conflicts:** Embedded constraints need specific settings
- **Standard compatibility:** Need C++20 features without breaking C compilation

### What It Does

1. **C++-specific flags:**
   - Applies flags only to .cpp/.hpp files (via CXXFLAGS)
   - Enables C++20 with GNU extensions (`-std=gnu++2a`)
   - Enables concepts support (`-fconcepts`)

2. **Embedded optimizations:**
   - Disables exceptions (`-fno-exceptions`)
   - Disables RTTI (`-fno-rtti`)
   - Enables function sections (`-ffunction-sections`)
   - Enables data sections (`-fdata-sections`)
   - Allows linker garbage collection to remove unused code

3. **Warning management:**
   - Strict warnings on project code
   - Suppresses third-party library warnings
   - Prevents build output pollution

### Applied Compiler Flags

```cpp
-std=gnu++2a              // C++20 with GNU extensions
-fconcepts                // Enable C++20 concepts
-fno-exceptions           // Disable exceptions (embedded)
-fno-rtti                 // Disable RTTI (size optimization)
-ffunction-sections       // Each function in separate section
-fdata-sections           // Each data item in separate section
```

**Note:** Pairs with linker flags `-Wl,--gc-sections` to remove unused code/data.

### Automatic Behavior

Script runs automatically before every build. No configuration needed.

### Manual Invocation

Not applicable (build-phase hook only).

## Integration with PlatformIO

All scripts are configured in `platformio.ini`:

```ini
[platformio]
; Build scripts run automatically during the pre-build phase
extra_scripts =
    pre:scripts/version.py
    pre:scripts/cpp-build-flags.py

; Custom upload protocol for BLE DFU
[env:my_device]
upload_protocol = custom
upload_command = python scripts/dfu-ble-uploader.py $SOURCE
```

Also declared in `library.json` for library distribution:

```json
{
  "build": {
    "extraScript": [
      "scripts/version.py",
      "scripts/cpp-build-flags.py"
    ]
  }
}
```

## Common Workflows

### Initial Device Setup

1. **Create device configuration:**
   ```bash
   # Copy device_config.json to project root
   cp device_config.json.example device_config.json
   # Edit with your device MAC addresses and details
   ```

2. **First build:**
   ```bash
   pio run -e my_device
   # Scripts auto-detect MAC, create cache, inject defines
   ```

3. **Verify configuration:**
   ```bash
   # Check build output for device info
   # Should show detected MAC addresses and config
   ```

### Firmware Update Workflow

1. **Build firmware:**
   ```bash
   pio run -e my_device
   ```

2. **Upload via BLE:**
   ```bash
   pio run -e my_device -t upload
   # OR
   python scripts/dfu-ble-uploader.py .pio/build/my_device/firmware.bin -d aa:bb:cc:dd:ee:ff
   ```

3. **Verify new version:**
   ```bash
   # Read Device Information Service via BLE
   # Check firmware version matches built version
   ```

### Multi-Device Management

1. **Add devices to config:**
   ```json
   {
     "devices": {
       "aabbccddeeff": { "name": "Device1", ... },
       "112233445566": { "name": "Device2", ... }
     }
   }
   ```

2. **Build for each device:**
   ```bash
   # Script auto-detects which device is connected
   pio run -e device1
   pio run -e device2
   ```

3. **Batch updates:**
   ```bash
   # Upload to multiple devices sequentially
   python scripts/dfu-ble-uploader.py firmware.bin -n "Device1"
   python scripts/dfu-ble-uploader.py firmware.bin -n "Device2"
   ```

## Troubleshooting

### version.py Issues

**"Device configuration file not found"**
- Create `device_config.json` in project root
- Or build will use auto-generated defaults

**"No ESP32 upload port detected"**
- Connect ESP32 via USB
- Or set `upload_port` in platformio.ini
- Or set `UPLOAD_PORT` environment variable

**"Failed to parse WiFi MAC from output"**
- Ensure esptool is installed (PlatformIO should handle this)
- Check device is properly connected and recognized
- Try manual MAC detection: `esptool.py --port /dev/cu.usbmodem* read_mac`

### dfu-ble-uploader.py Issues

**"Device required: provide -d, -n, or set BLE_DEVICE_ADDRESS"**
- Add device to device_config.json with `ble_device` field
- Or specify manually: `-d aa:bb:cc:dd:ee:ff`

**"ModuleNotFoundError: No module named 'bleak'"**
- Script should auto-install bleak
- If fails, manually install: `pip install bleak`

**"Connection failed after 3 retries"**
- Check device is advertising (not already connected elsewhere)
- Check Bluetooth is enabled on host machine
- Move device closer to reduce signal interference

### cpp-build-flags.py Issues

**C++ compilation errors**
- Check C++ source uses valid C++20 syntax
- Verify third-party libraries are C++20 compatible
- Check for exceptions/RTTI usage (disabled for embedded)

## Security Considerations

### version.py
- `device_config.json` may contain sensitive device identities
- Consider adding to `.gitignore` for production deployments
- MAC cache files in `.pio/build/` are local-only (not committed)

### dfu-ble-uploader.py
- **No encryption:** DFU transfers are unencrypted over BLE
- **No authentication:** Device does not verify the uploader's identity
- Suitable for development/trusted environments only
- For production: implement secure DFU with encrypted/signed firmware

## References

- **Nordic DFU Protocol:** [Nordic DFU documentation](https://infocenter.nordicsemi.com/topic/sdk_nrf5_v17.1.0/lib_dfu.html)
- **ESP32 MAC Addresses:** [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/misc_system_api.html#mac-address)
- **PlatformIO Build Scripts:** [PlatformIO documentation](https://docs.platformio.org/en/latest/scripting/index.html)
- **Semantic Versioning:** [semver.org](https://semver.org/)