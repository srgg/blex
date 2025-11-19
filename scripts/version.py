"""
Automated versioning and device configuration script for PlatformIO.

Extracts the semantic version from Git tags and manages device-specific configuration
including MAC address detection, serial number generation, and build flag injection.

This script runs during PlatformIO's pre-build phase to:
- Detect Git version and revision
- Read ESP32 MAC addresses via esptool (with caching)
- Map devices to configuration via serial number
- Generate C++ defines for Device Information Service
- Set BLE_DEVICE_ADDRESS environment variable for DFU uploads
"""
Import("env")
import subprocess
import re
import sys
import os
import glob
import json
from pathlib import Path

# Load device configuration from external JSON file
def load_device_config():
    """
    Load device configuration from device_config.json in project root.

    Returns:
        dict: Device configuration with manufacturer info and device-specific settings,
              or None if a config file doesn't exist (will use defaults)

    Raises:
        RuntimeError: If a config file exists but has invalid JSON or missing required keys
    """
    config_path = Path(env.get("PROJECT_DIR")) / "device_config.json"

    if not config_path.exists():
        print(f"Warning: Device configuration file not found: {config_path}")
        print("Device-specific configuration will not be available. Using default values.")
        return None

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in {config_path}: {e}")

    # Validate required keys
    required_keys = ["manufacturer", "manufacturer_id", "hardware_version", "devices"]
    missing_keys = [key for key in required_keys if key not in config]
    if missing_keys:
        raise RuntimeError(
            f"Missing required keys in {config_path}: {', '.join(missing_keys)}"
        )

    return config

DEVICE_CONFIG = load_device_config()

# ESP32 BLE MAC offset from WiFi MAC (hardware convention)
ESP32_BLE_MAC_OFFSET = 2

# MAC address regex pattern for esptool output parsing
MAC_ADDRESS_PATTERN = r'MAC:\s*([0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2})'

def detect_upload_port(env):
    """
    Detect the ESP32 upload port from platformio.ini, the environment, or an auto-scan.

    Priority order:
    1. platformio.ini upload_port setting
    2. UPLOAD_PORT environment variable
    3. Auto-detection via USB device patterns

    Args:
        env: PlatformIO environment object

    Returns:
        str: Serial port path (e.g., '/dev/cu.usbmodem123') or None if not found

    Note:
        Auto-detection returns the first matching USB serial device.
        On multi-device systems, explicit configuration is recommended.
    """
    # 1. Try explicit upload_port from platformio.ini
    try:
        port = env.GetProjectOption("upload_port", None)
        if port:
            print(f"Using upload_port from platformio.ini: {port}")
            return port
    except Exception:
        pass

    # 2. Try UPLOAD_PORT from the environment (set during upload)
    port = env.get("UPLOAD_PORT")
    if port:
        print(f"Using UPLOAD_PORT from env: {port}")
        return port

    # 3. Auto-detect serial ports
    # Look for common ESP32 USB serial devices
    patterns = [
        '/dev/cu.usbmodem*',   # macOS USB CDC
        '/dev/cu.usbserial*',  # macOS FTDI/CH340
        '/dev/cu.SLAB_USBtoUART*',  # macOS CP2102
        '/dev/ttyUSB*',        # Linux
        '/dev/ttyACM*',        # Linux CDC
        'COM*'                 # Windows
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            port = ports[0]  # Use first match
            print(f"Auto-detected serial port: {port}")
            return port

    print("Warning: No serial port detected")
    return None


def get_esp32_mac_addresses(env):
    """
    Read ESP32 BLE and WiFi MAC addresses via esptool.

    Executes esptool.py read_mac command on a connected ESP32 device and parses
    the output to extract MAC addresses. BLE MAC is derived from Wi-Fi MAC using
    ESP32 hardware convention (WiFi MAC + 2).

    Args:
        env: PlatformIO environment object

    Returns:
        dict: MAC addresses as {'ble': 'aa:bb:cc:dd:ee:ff', 'wifi': 'aa:bb:cc:dd:ee:ff'}

    Raises:
        RuntimeError: If no upload port is detected, esptool fails, or MAC parsing fails

    Note:
        Requires a connected ESP32 device accessible via USB serial.
        Uses ESP32_BLE_MAC_OFFSET constant for BLE MAC calculation.
    """
    port = detect_upload_port(env)
    if not port:
        raise RuntimeError(
            "No ESP32 upload port detected.\n"
            "Solutions:\n"
            "  1. Connect ESP32 device via USB\n"
            "  2. Set upload_port in platformio.ini\n"
            "  3. Set UPLOAD_PORT environment variable"
        )

    # Path to esptool.py - use PlatformIO's package manager
    platform = env.PioPlatform()
    esptool_path = platform.get_package_dir("tool-esptoolpy") + "/esptool.py"

    # Run esptool.py read_mac
    try:
        cmd = [sys.executable, esptool_path, "--port", port, "read_mac"]
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

        if result.returncode != 0:
            error_msg = f"esptool.py failed with exit code {result.returncode}"
            if result.stderr:
                error_msg += f"\nSTDERR: {result.stderr}"
            if result.stdout:
                error_msg += f"\nSTDOUT: {result.stdout}"
            raise RuntimeError(error_msg)

        output = result.stdout
        print(f"esptool output:\n{output}")
    except subprocess.TimeoutExpired:
        raise RuntimeError("esptool.py timed out - device may be unresponsive")
    except FileNotFoundError:
        raise RuntimeError(f"esptool.py not found at {esptool_path}")

    # Parse MAC addresses from output
    # esptool output format for ESP32-S3:
    # MAC: ec:da:3b:5c:83:68
    detected_wifi_mac = None
    detected_ble_mac = None

    for line in output.splitlines():
        line = line.strip()
        # Look for the MAC address line
        if line.startswith("MAC:"):
            # Extract MAC from "MAC: ec:da:3b:5c:83:68"
            mac_match = re.search(MAC_ADDRESS_PATTERN, line)
            if mac_match and not detected_wifi_mac:  # Take the first MAC occurrence
                detected_wifi_mac = mac_match.group(1).lower()
                # Calculate BLE MAC from WiFi MAC using ESP32 hardware convention
                mac_bytes = [int(x, 16) for x in detected_wifi_mac.split(':')]
                mac_bytes[5] = (mac_bytes[5] + ESP32_BLE_MAC_OFFSET) & 0xFF
                detected_ble_mac = ':'.join(f'{b:02x}' for b in mac_bytes)
                break

    if not detected_wifi_mac:
        raise RuntimeError(f"Failed to parse WiFi MAC from output:\n{output}")

    return {'ble': detected_ble_mac, 'wifi': detected_wifi_mac}

def get_firmware_version():
    """
    Get the semantic version from the Git tag.
    Expected tag format: v1.2.3 or 1.2.3
    Returns: "1.2.3" or "0.0.0-dev" if no tags
    """
    try:
        # Get the latest tag
        tag = subprocess.check_output(
            ["git", "describe", "--tags", "--abbrev=0"],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()

        # Strip 'v' prefix if present
        version = tag.lstrip('v')

        # Validate semantic version format (MAJOR.MINOR.PATCH)
        if re.match(r'^\d+\.\d+\.\d+$', version):
            return version
        else:
            print(f"Warning: Git tag '{tag}' is not valid semver, using 0.0.0-dev")
            return "0.0.0-dev"
    except subprocess.CalledProcessError:
        # No tags found
        return "0.0.0-dev"

def get_git_revision():
    """
    Get a Git commit hash (short) with a dirty flag.
    Returns: "abc1234" or "abc1234-dirty"
    """
    try:
        # Get a short commit hash
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()

        # Check if the working tree is dirty
        status = subprocess.check_output(
            ["git", "status", "--porcelain"],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()

        if status:
            return f"{commit}-dirty"
        else:
            return commit
    except subprocess.CalledProcessError:
        return "unknown"

# Get version info
firmware_version = get_firmware_version()
git_revision = get_git_revision()

# Get board name from PlatformIO environment (used for MODEL_NUMBER only)
board_name = env.get("BOARD", "unknown")

# Store cache in PlatformIO's build directory (per-environment)
cache_file = Path(env.subst("$BUILD_DIR")) / "device_macs.json"
cache_file.parent.mkdir(parents=True, exist_ok=True)  # Ensure the build directory exists

# Try to load from the cache file first
mac_addresses = None
if cache_file.exists():
    try:
        with open(cache_file, 'r') as f:
            cached = json.load(f)
            # Use cache regardless of board - it's whatever device is/was connected
            mac_addresses = cached
            print(f"Using cached MAC addresses: BLE={cached['ble']}, WiFi={cached['wifi']}")
    except Exception as e:
        print(f"Failed to load cache: {e}")

# If no cache, try to detect from the connected device
if not mac_addresses:
    try:
        mac_data = get_esp32_mac_addresses(env)
        mac_addresses = {
            'ble': mac_data['ble'],
            'wifi': mac_data['wifi']
        }
        # Save to cache - overwrites previous device's MACs
        try:
            with open(cache_file, 'w') as f:
                json.dump(mac_addresses, f, indent=2)
            print(f"Detected and saved MAC addresses: BLE={mac_addresses['ble']}, WiFi={mac_addresses['wifi']}")
        except Exception as e:
            print(f"Warning: Failed to save cache: {e}")
    except RuntimeError as e:
        print(f"Warning: {e}, using fallback values")
        mac_addresses = None

# Extract MAC values and serial number
if mac_addresses:
    ble_mac = mac_addresses['ble']
    wifi_mac = mac_addresses['wifi']
    serial_number = ble_mac.replace(":", "")  # Serial = BLE MAC without colons
else:
    ble_mac = "00:00:00:00:00:00"
    wifi_mac = "00:00:00:00:00:00"
    serial_number = "000000000000"

# Look up device-specific configuration by serial number
device_config = None
if DEVICE_CONFIG:
    device_config = DEVICE_CONFIG["devices"].get(serial_number, None)

# Warn if device is not configured
if not device_config:
    print(f"Warning: No device config for serial '{serial_number}', using defaults")
    device_config = {
        "serial_number": serial_number,
        "ble_address": ble_mac,
        "wifi_address": wifi_mac,
        "name": f"BX-{serial_number[-4:]}"  # Use the last 4 chars of serial
        # no long_name - will only use short name
    }
else:
    print(f"Found device config for serial '{serial_number}': {device_config.get('name', 'Unknown')}")

    # If device config has ble_device, add it to cache
    if 'ble_device' in device_config and device_config['ble_device']:
        ble_device_addr = device_config['ble_device']
        # Update cache with ble_device if not already present
        if 'ble_device' not in mac_addresses:
            mac_addresses['ble_device'] = ble_device_addr
            try:
                with open(cache_file, 'w') as f:
                    json.dump(mac_addresses, f, indent=2)
                print(f"Added BLE device address to cache: {ble_device_addr}")
            except Exception as e:
                print(f"Warning: Failed to update cache: {e}")

# Extract device names
device_name = device_config['name']
device_long_name = device_config.get('long_name', None)  # Optional
ble_device_addr = device_config.get('ble_device', None)  # Optional - BLE random private address

# Build the C++ defines list
# Use DEVICE_CONFIG values if available, otherwise use None
manufacturer_name = DEVICE_CONFIG["manufacturer"] if DEVICE_CONFIG else None
manufacturer_id = DEVICE_CONFIG["manufacturer_id"] if DEVICE_CONFIG else None
hardware_version = DEVICE_CONFIG["hardware_version"] if DEVICE_CONFIG else None

cpp_defines = [
    # Version info (from Git)
    ("FIRMWARE_VERSION", f'\\"{firmware_version}\\"'),
    ("SOFTWARE_REVISION", f'\\"{git_revision}\\"'),
    ("MODEL_NUMBER", f'\\"{board_name}\\"'),

    # Device Information Service values (from DEVICE_CONFIG or None)
    ("MANUFACTURER_NAME", f'\\"{manufacturer_name}\\"' if manufacturer_name else None),
    ("MANUFACTURER_ID", f'{manufacturer_id}' if manufacturer_id is not None else None),
    ("HARDWARE_VERSION", f'\\"{hardware_version}\\"' if hardware_version else None),
    ("SERIAL_NUMBER", f'\\"{serial_number}\\"'),
    ("DEVICE_BLE_MAC", f'\\"{ble_mac}\\"'),
    ("DEVICE_WIFI_MAC", f'\\"{wifi_mac}\\"'),

    # BLE device name (always required)
    ("DEVICE_NAME", f'\\"{device_name}\\"'),
]

# Filter out None values
cpp_defines = [(k, v) for k, v in cpp_defines if v is not None]

# Add long name only if defined
if device_long_name:
    cpp_defines.append(("DEVICE_NAME_LONG", f'\\"{device_long_name}\\"'))

# Add BLE device address only if defined (random private address for BLE privacy)
if ble_device_addr:
    cpp_defines.append(("DEVICE_BLE_ADDR", f'\\"{ble_device_addr}\\"'))

env.Append(CPPDEFINES=cpp_defines)

# Set BLE_DEVICE_ADDRESS environment variable for DFU uploader
# Priority: 1. ble_device (random private address), 2. long_name, 3. name
if ble_device_addr:
    os.environ["BLE_DEVICE_ADDRESS"] = ble_device_addr
    print(f"Set BLE_DEVICE_ADDRESS={ble_device_addr} (ble_device)")

# Print build configuration
print("="*60)
if DEVICE_CONFIG is None:
    print("Device Configuration: [WARNING: device_config.json NOT FOUND - using defaults]")
elif not DEVICE_CONFIG["devices"].get(serial_number):
    print(f"Device Configuration: [WARNING: Device '{serial_number}' NOT in config - using defaults]")
else:
    print("Device Configuration:")
print(f"  Board Type:        {board_name}")
if manufacturer_name:
    print(f"  Manufacturer:      {manufacturer_name}")
if manufacturer_id is not None:
    print(f"  Manufacturer ID:   0x{manufacturer_id:x}")
print(f"  BLE MAC:           {ble_mac}")
if ble_device_addr:
    print(f"  BLE Device Addr:   {ble_device_addr}")
print(f"  WiFi MAC:          {wifi_mac}")
print(f"  Serial Number:     {serial_number}")
if hardware_version:
    print(f"  Hardware Version:  {hardware_version}")
print(f"  Firmware Version:  {firmware_version}")
print(f"  Software Revision: {git_revision}")
print(f"  Device Name:       {device_name}")
if device_long_name:
    print(f"  Long Name:         {device_long_name}")
print("="*60)