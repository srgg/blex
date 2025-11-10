"""
Automated versioning and device configuration script for PlatformIO
Extracts the semantic version from Git tags and manages device info per board
"""
Import("env")
import subprocess
import re

# Device Information Configuration
# Adds new boards here with their specific configuration
DEVICE_CONFIG = {
    "manufacturer": "BLIMCo",
    "manufacturer_id": 0xFFFE,
    "hardware_version": "1.0",

    # Board-specific configuration
    "boards": {
        "um_feathers3": {
            "serial_number": "FS3-001",
            "device_name": "BLIM IMU Stream",
            "device_name_short": "BLIM-IMU"
        },
        # Example for additional boards:
        # "esp32_devkit": {
        #     "serial_number": "DK1-001",
        #     "device_name": "BLIM IMU Dev",
        #     "device_name_short": "BLIM-DEV"
        # }
    }
}

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

# Get board name from PlatformIO environment
board_name = env.get("BOARD", "unknown")

# Get board-specific configuration
board_config = DEVICE_CONFIG["boards"].get(board_name, {})

# Warn if board not configured
if not board_config:
    print(f"Warning: No device config for board '{board_name}', using defaults")
    board_config = {
        "serial_number": "UNKNOWN",
        "device_name": "Unknown Device",
        "device_name_short": "UNKNOWN"
    }

# Define all build flags
env.Append(CPPDEFINES=[
    # Version info (from Git)
    ("FIRMWARE_VERSION", f'\\"{firmware_version}\\"'),
    ("SOFTWARE_REVISION", f'\\"{git_revision}\\"'),
    ("MODEL_NUMBER", f'\\"{board_name}\\"'),

    # Device Information Service values (from DEVICE_CONFIG)
    ("MANUFACTURER_NAME", f'\\"{DEVICE_CONFIG["manufacturer"]}\\"'),
    ("MANUFACTURER_ID", f'{DEVICE_CONFIG["manufacturer_id"]}'),
    ("HARDWARE_VERSION", f'\\"{DEVICE_CONFIG["hardware_version"]}\\"'),
    ("SERIAL_NUMBER", f'\\"{board_config["serial_number"]}\\"'),

    # BLE device names (from board config)
    ("DEVICE_NAME", f'\\"{board_config["device_name"]}\\"'),
    ("DEVICE_NAME_SHORT", f'\\"{board_config["device_name_short"]}\\"')
])

# Print build configuration
print("="*60)
print("Device Configuration:")
print(f"  Board:             {board_name}")
print(f"  Manufacturer:      {DEVICE_CONFIG['manufacturer']}")
print(f"  Manufacturer ID:   0x{DEVICE_CONFIG['manufacturer_id']:x}")
print(f"  Serial Number:     {board_config['serial_number']}")
print(f"  Hardware Version:  {DEVICE_CONFIG['hardware_version']}")
print(f"  Firmware Version:  {firmware_version}")
print(f"  Software Revision: {git_revision}")
print(f"  Device Name:       {board_config['device_name']}")
print(f"  Short Name:        {board_config['device_name_short']}")
print("="*60)