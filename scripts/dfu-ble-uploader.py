#!/usr/bin/env python3
"""
ESP32-S3 BLE DFU Firmware Uploader for PlatformIO

Uploads firmware to ESP32-S3 devices over BLE using Nordic DFU protocol.
Designed to integrate with PlatformIO's upload system.

Usage:
    python dfu-ble-uploader.py firmware.bin -d "BLIM IMU Stream" [-p 16] [-v]

PlatformIO.ini configuration:
    upload_protocol = custom
    upload_command = python dfu-ble-uploader.py $SOURCE -d "Device Name"
"""

import sys
import os
import subprocess
import argparse
import asyncio
import zlib
from pathlib import Path
from typing import Optional

# ============== Relaunch into PlatformIO venv if present =================
PIO_PENV = os.path.expanduser("~/.platformio/penv/bin/python")
if os.path.exists(PIO_PENV) and not sys.executable.startswith(os.path.dirname(PIO_PENV)):
    os.execv(PIO_PENV, [PIO_PENV] + sys.argv)

# ============== Ensure required packages are available =================
def ensure_packages():
    packages_to_install = []

    try:
        import bleak  # noqa
    except Exception:
        packages_to_install.append("bleak")

    try:
        import rich  # noqa
    except Exception:
        packages_to_install.append("rich")

    if packages_to_install:
        print(f"Installing required packages: {', '.join(packages_to_install)}...", flush=True)
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q"] + packages_to_install)

    # Final imports to expose names
    try:
        from bleak import BleakClient, BleakScanner  # noqa: F401
        from rich.console import Console  # noqa: F401
        from rich.progress import Progress, BarColumn, TextColumn, TimeRemainingColumn, TransferSpeedColumn  # noqa: F401
        from rich.live import Live  # noqa: F401
        from rich.layout import Layout  # noqa: F401
        from rich.text import Text  # noqa: F401
    except Exception as e:
        print(f"Failed to import required packages: {e}", flush=True)
        raise

ensure_packages()
from bleak import BleakClient, BleakScanner  # type: ignore
from rich.console import Console, Group
from rich.progress import Progress, BarColumn, TextColumn, TimeRemainingColumn, TransferSpeedColumn, TaskID
from rich.live import Live
from rich.layout import Layout
from rich.text import Text
from rich.panel import Panel

# ============== DFU UUIDs and opcodes (match your firmware) ================
DFU_SERVICE_UUID = "0000fe59-0000-1000-8000-00805f9b34fb"   # full 128-bit advertised UUID
DFU_CTRL_UUID    = "8ec90001-f315-4f60-9fb8-838830daea50"   # control (notify + write)
DFU_DATA_UUID    = "8ec90002-f315-4f60-9fb8-838830daea50"   # data (write without response)

# DFU Opcodes
OP_CREATE_OBJECT    = 0x01
OP_SET_RECEIPT      = 0x02
OP_CALCULATE_CRC    = 0x03
OP_EXECUTE_OBJECT   = 0x04
OP_SELECT_OBJECT    = 0x06
OP_RESPONSE         = 0x60

# DFU Object Types
DFU_OBJ_DATA    = 0x02

# DFU Status Codes
DFU_STATUS_SUCCESS = 0x01
DFU_STATUS_OPCODE_NOT_SUPPORTED = 0x02
DFU_STATUS_INVALID_COMMAND = 0x03
DFU_STATUS_INVALID_PARAMETER = 0x04
DFU_STATUS_OPERATION_FAILED = 0x05

# defaults
DEFAULT_PRN = 32  # CRC verification every 16KB (32 × 512 bytes)
DEFAULT_CHUNK = 512

# ============== Helper: parse response bytes into structures =================
def parse_response(data: bytes):
    """
    dfu_response_t:
      response_opcode (1) == 0x60
      request_opcode_id (1)
      status (1)
      optional payload:
        - For CALCULATE_CRC / SELECT: offset(4) + crc32(4) [+ max_size(4) for select]
    Returns tuple (req_opcode, status, payload_bytes)
    """
    if not data or len(data) < 3:
        return None
    if data[0] != OP_RESPONSE:
        # It's not a response opcode; caller may handle alternate notifications
        return None
    req_opcode = data[1]
    status = data[2]
    payload = data[3:] if len(data) > 3 else b""
    return (req_opcode, status, payload)

# ============== Control response waiter (maps req_opcode -> future) ===========
class ControlWaiter:
    def __init__(self):
        self._resp_futures = {}  # req_opcode -> Future
        self._prn_future: Optional[asyncio.Future] = None

    def notification_handler(self, sender, data: bytes):
        # Called on every notification from control char
        parsed = parse_response(data)
        if parsed:
            req_opcode, status, payload = parsed

            # Check if this is a PRN notification (unsolicited CALCULATE_CRC response)
            if req_opcode == OP_CALCULATE_CRC and self._prn_future and not self._prn_future.done():
                # This is a PRN notification - extract offset and CRC from payload
                if len(payload) >= 8:
                    offset = int.from_bytes(payload[0:4], "little")
                    crc = int.from_bytes(payload[4:8], "little")
                    self._prn_future.set_result((offset, crc))
                    return

            # Otherwise, it's a regular command response
            fut = self._resp_futures.pop(req_opcode, None)
            if fut and not fut.done():
                fut.set_result((status, payload))
            return

        # else: maybe a PRN-style notification: match length 8 => offset(4), crc(4)
        if len(data) == 8 and self._prn_future and not self._prn_future.done():
            offset = int.from_bytes(data[0:4], "little")
            crc = int.from_bytes(data[4:8], "little")
            self._prn_future.set_result((offset, crc))
            return

        # Unknown/extra notifications: print for debugging
        print("🔔 Ctrl notification (raw):", data.hex(), flush=True)

    async def wait_response(self, req_opcode: int, timeout=5.0):
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._resp_futures[req_opcode] = fut
        try:
            status, payload = await asyncio.wait_for(fut, timeout=timeout)
            return status, payload
        finally:
            self._resp_futures.pop(req_opcode, None)

    async def wait_prn(self, timeout=12.0):
        loop = asyncio.get_running_loop()
        self._prn_future = loop.create_future()
        try:
            res = await asyncio.wait_for(self._prn_future, timeout=timeout)
            return res
        finally:
            self._prn_future = None

# ============== DFU uploader class ===========================================
class DFUUploader:
    def __init__(self, firmware_path: Path, device_name: str, prn: int = DEFAULT_PRN, chunk_size: int = DEFAULT_CHUNK, verbose: bool = False):
        self.firmware_path = Path(firmware_path)
        self.device_name = device_name
        self.prn = prn
        self.chunk_size = chunk_size
        self.verbose = verbose
        self.client: Optional[BleakClient] = None
        self.ctrl_uuid = DFU_CTRL_UUID
        self.data_uuid = DFU_DATA_UUID
        self.waiter = ControlWaiter()
        self.disconnect_event = asyncio.Event()
        self.console = Console(markup=False, highlight=False)
        self.start_time = None
        self.show_details = False
        self.detail_lines = []
        self.stdin_fd = None
        self.old_tty_settings = None

    async def scan_for_device(self, timeout=10):
        from rich.status import Status

        dfu_device = None
        seen = set()
        found_event = asyncio.Event()

        def detection_cb(device, adv):
            nonlocal dfu_device
            addr = device.address
            if addr not in seen:
                seen.add(addr)
                if self.verbose:
                    services = adv.service_uuids or []
                    has_dfu = DFU_SERVICE_UUID.lower() in [s.lower() for s in services]
                    mark = "✓" if has_dfu else " "
                    name = device.name or "Unknown"
                    self.console.print(f"  {mark} {name:25} ({addr})")
            if not dfu_device and device.name == self.device_name and DFU_SERVICE_UUID.lower() in [s.lower() for s in (adv.service_uuids or [])]:
                dfu_device = device
                found_event.set()

        with Status(f"Scanning for '{self.device_name}'...", console=self.console, spinner="dots"):
            async with BleakScanner(detection_callback=detection_cb) as _:
                try:
                    await asyncio.wait_for(found_event.wait(), timeout=timeout)
                except asyncio.TimeoutError:
                    pass

        if not dfu_device:
            self.console.print(f"Device '{self.device_name}' not found. Please verify:")
            self.console.print("  1. Device powered on")
            self.console.print(f"  2. Device advertising DFU service 0xfe59 or {DFU_SERVICE_UUID}")
            self.console.print(f"  3. Name matches exactly: '{self.device_name}'")
            raise RuntimeError(f"Device not found")
        return dfu_device

    def _on_disconnect(self, _):
        self.disconnect_event.set()

    async def connect(self, device):
        from rich.status import Status

        try:
            with Status(f"Connecting to {device.name}...", console=self.console, spinner="dots"):
                self.client = BleakClient(device.address, disconnected_callback=self._on_disconnect)
                await self.client.connect()
                await self.client.start_notify(self.ctrl_uuid, self.waiter.notification_handler)
            self.console.print(f"Connected to {device.name} ({device.address})")
        except Exception as e:
            self.console.print(f"Found {device.name} ({device.address})")
            raise

    async def disconnect(self):
        if self.client:
            try:
                await self.client.stop_notify(self.ctrl_uuid)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass  # Already disconnected
            self.client = None

    async def write_control(self, payload: bytes, with_response=True):
        assert self.client
        await self.client.write_gatt_char(self.ctrl_uuid, payload, response=with_response)

    async def write_packet(self, data: bytes):
        assert self.client
        await self.client.write_gatt_char(self.data_uuid, data, response=False)

    # High-level protocol ops
    async def set_prn(self):
        payload = bytes([OP_SET_RECEIPT]) + int(self.prn).to_bytes(2, "little")
        if self.verbose:
            self.console.print(f"[dim]Set PRN={self.prn}[/dim]")
        await self.write_control(payload, with_response=True)
        status, _ = await self.waiter.wait_response(OP_SET_RECEIPT, timeout=5.0)
        if status != DFU_STATUS_SUCCESS:
            raise RuntimeError(f"SET_PRN failed status=0x{status:02x}")

    async def select_object(self, obj_type):
        payload = bytes([OP_SELECT_OBJECT, obj_type])
        await self.write_control(payload, with_response=True)
        status, payload = await self.waiter.wait_response(OP_SELECT_OBJECT, timeout=5.0)
        if status != DFU_STATUS_SUCCESS:
            raise RuntimeError(f"SELECT_OBJECT failed status=0x{status:02x}")

        # Parse response: offset(4), crc(4), [max_size(4) optional]
        if len(payload) >= 8:
            offset = int.from_bytes(payload[0:4], "little")
            crc = int.from_bytes(payload[4:8], "little")
            max_size = int.from_bytes(payload[8:12], "little") if len(payload) >= 12 else None
            return (offset, crc, max_size)
        return (0, 0, None)

    async def create_object(self, obj_type, size):
        payload = bytes([OP_CREATE_OBJECT, obj_type]) + int(size).to_bytes(4, "little")

        # Add to the detail log
        self._add_detail(f"Create object: type={obj_type} size={size:,} bytes")

        if self.verbose:
            self.console.print(f"Create object: type={obj_type} size={size:,} bytes")
        await self.write_control(payload, with_response=True)
        status, _ = await self.waiter.wait_response(OP_CREATE_OBJECT, timeout=5.0)
        if status != DFU_STATUS_SUCCESS:
            raise RuntimeError(f"CREATE_OBJECT failed status=0x{status:02x}")

    async def calculate_crc(self):
        await self.write_control(bytes([OP_CALCULATE_CRC]), with_response=True)
        status, payload = await self.waiter.wait_response(OP_CALCULATE_CRC, timeout=8.0)
        if status != DFU_STATUS_SUCCESS:
            raise RuntimeError(f"CALCULATE_CRC failed status=0x{status:02x}")
        if len(payload) >= 8:
            offset = int.from_bytes(payload[0:4], "little")
            crc = int.from_bytes(payload[4:8], "little")
            # If select included max_size, payload may be longer.
            return offset, crc
        else:
            raise RuntimeError("CALCULATE_CRC returned unexpected payload length")

    async def execute_object(self):
        await self.write_control(bytes([OP_EXECUTE_OBJECT]), with_response=True)
        try:
            status, _ = await self.waiter.wait_response(OP_EXECUTE_OBJECT, timeout=10.0)
            if status != DFU_STATUS_SUCCESS:
                raise RuntimeError(f"EXECUTE_OBJECT failed status=0x{status:02x}")
        except asyncio.TimeoutError:
            # Device may reboot before sending a response - this is OK
            if self.verbose:
                self.console.print("No response (device may have rebooted)")

    def _setup_stdin_reader(self, loop):
        """Setup non-blocking stdin reader for 'o' key detection"""
        import termios
        import tty

        # Save terminal settings
        self.stdin_fd = sys.stdin.fileno()
        self.old_tty_settings = termios.tcgetattr(self.stdin_fd)

        # Set terminal to raw mode for character-by-character input
        tty.setcbreak(self.stdin_fd)

        # Add stdin reader to event loop
        loop.add_reader(self.stdin_fd, self._handle_stdin_input)

    def _handle_stdin_input(self):
        """Handle stdin input (called by asyncio when data available)"""
        try:
            ch = sys.stdin.read(1)
            if ch.lower() == 'o':
                self.show_details = not self.show_details
        except Exception:
            pass

    def _cleanup_stdin(self, loop):
        """Restore terminal settings"""
        import termios

        try:
            loop.remove_reader(self.stdin_fd)
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_tty_settings)
        except Exception:
            pass

    def _add_detail(self, line: str):
        """Add a detail line to the buffer"""
        self.detail_lines.append(line)
        # Keep only last 20 lines
        if len(self.detail_lines) > 20:
            self.detail_lines.pop(0)

    def _build_display(self, progress: Progress, task):
        """Build the display with progress bar and optional details"""
        elements = [progress]

        # Add toggle hint and details if enabled
        if self.show_details:
            elements.append(Text("Details (press 'o' to hide):"))
            for line in self.detail_lines:
                elements.append(Text(f"  {line}"))
        else:
            elements.append(Text("Press 'o' to show details"))

        return Group(*elements)

    async def upload(self):
        # Validate firmware file
        if not self.firmware_path.exists():
            raise RuntimeError(f"Firmware file not found: {self.firmware_path}")
        if not self.firmware_path.is_file():
            raise RuntimeError(f"Not a file: {self.firmware_path}")

        file_size = self.firmware_path.stat().st_size
        if file_size == 0:
            raise RuntimeError("Firmware file is empty")
        if file_size > 4 * 1024 * 1024:
            raise RuntimeError(f"Firmware too large: {file_size:,} bytes (max 4MB)")

        firmware = self.firmware_path.read_bytes()
        total = len(firmware)
        print(f"Starting BLE firmware upload (Python {sys.version_info.major}.{sys.version_info.minor}): {total:,} bytes, CRC32: 0x{zlib.crc32(firmware) & 0xFFFFFFFF:08X}", flush=True)

        device = await self.scan_for_device(timeout=10)
        await self.connect(device)

        try:
            # set PRN first
            await self.set_prn()

            # Use DFU_OBJ_DATA for data objects
            obj_type = DFU_OBJ_DATA

            # Check if we can resume from previous upload
            device_offset, device_crc, max_size = await self.select_object(obj_type)

            resume_offset = 0
            if device_offset > 0:
                # Device has partial data - check if it matches our firmware
                local_crc = zlib.crc32(firmware[0:device_offset]) & 0xFFFFFFFF

                if device_offset == total and local_crc == device_crc:
                    # Upload already complete! Just execute
                    print(f"✓ Firmware already uploaded ({device_offset:,} bytes, CRC=0x{device_crc:08X}), executing...", flush=True)
                    await self.execute_object()

                    # Wait for reboot
                    if not self.disconnect_event.is_set():
                        print("⏳ Waiting for device to reboot...", flush=True)
                        try:
                            await asyncio.wait_for(self.disconnect_event.wait(), timeout=5.0)
                        except asyncio.TimeoutError:
                            print("⚠️  Device did not disconnect within 5 seconds", flush=True)
                    print("✅ DFU complete - device rebooted successfully", flush=True)
                    return

                elif device_offset < total and local_crc == device_crc:
                    # Partial match - resume from where we left off
                    resume_offset = device_offset
                    print(f"✓ Resuming upload from {resume_offset:,} bytes ({resume_offset*100//total}% complete, CRC=0x{device_crc:08X})", flush=True)
                else:
                    # CRC mismatch - start fresh
                    print(f"⚠ Previous upload: {device_offset:,} bytes, CRC mismatch - starting fresh", flush=True)
                    device_offset = 0

            # Setup stdin reader for interactive toggle
            loop = asyncio.get_running_loop()
            try:
                self._setup_stdin_reader(loop)
            except Exception:
                # If stdin setup fails (e.g., not a TTY), continue without interactivity
                pass

            # Upload loop with Rich progress bar
            try:
                progress = Progress(
                    TextColumn("[progress.description]{task.description}"),
                    BarColumn(),
                    TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
                    TextColumn("{task.completed}/{task.total} bytes"),
                    TransferSpeedColumn(),
                    TimeRemainingColumn(),
                    console=self.console
                )

                with Live(self._build_display(progress, None), console=self.console, refresh_per_second=4) as live:
                    # Start with "Preparing" at 0%
                    task = progress.add_task("Preparing", total=total, completed=0)
                    live.update(self._build_display(progress, task))

                    # Create object if starting fresh
                    if device_offset == 0:
                        await self.create_object(obj_type, total)

                    # Change to "Uploading" and set the correct offset
                    progress.update(task, description="Uploading", completed=resume_offset)
                    live.update(self._build_display(progress, task))

                    offset = resume_offset
                    packets = resume_offset // self.chunk_size

                    while offset < total:
                        chunk = firmware[offset:offset + self.chunk_size]
                        await self.write_packet(chunk)
                        offset += len(chunk)
                        packets += 1

                        # Update progress
                        progress.update(task, completed=offset)
                        live.update(self._build_display(progress, task))

                        # Yield every 16 packets to prevent BLE RX buffer overflow
                        if packets % 16 == 0:
                            await asyncio.sleep(0.01)

                        # PRN: wait when device expects receipt notification
                        if packets % self.prn == 0:
                            try:
                                prn_off, prn_crc = await self.waiter.wait_prn(timeout=12.0)
                                local_crc = zlib.crc32(firmware[0:prn_off]) & 0xFFFFFFFF
                                match_status = "OK" if local_crc == prn_crc else "MISMATCH"

                                # Add to detail log
                                self._add_detail(f"CRC check @ {prn_off:,} bytes: {match_status}")
                                if match_status == "OK":
                                    self._add_detail(f"  Device: 0x{prn_crc:08X}, Local: 0x{local_crc:08X}")

                                live.update(self._build_display(progress, task))

                                if local_crc != prn_crc:
                                    raise RuntimeError(f"PRN CRC mismatch at {prn_off}: device=0x{prn_crc:08X} local=0x{local_crc:08X}")
                            except asyncio.TimeoutError:
                                raise RuntimeError("PRN timeout")

                    # Clear the hint at the end - just show the completed progress bar
                    live.update(progress)
            finally:
                self._cleanup_stdin(loop)

            # All data sent - verify final CRC
            calc_off, calc_crc = await self.calculate_crc()
            local_crc = zlib.crc32(firmware[0:calc_off]) & 0xFFFFFFFF
            if local_crc != calc_crc:
                raise RuntimeError(f"CRC mismatch: device=0x{calc_crc:08X} local=0x{local_crc:08X}")

            print(f"🎉 Firmware uploaded: {calc_off} bytes, CRC=0x{calc_crc:08X}", flush=True)
            try:
                await self.execute_object()
            except Exception as e:
                # Device may disconnect during execution - this is expected behavior
                if "disconnect" not in str(e).lower():
                    raise  # Re-raise if it's not a disconnect error

            # Wait for device to reboot (disconnect) if not already detected
            if not self.disconnect_event.is_set():
                from rich.status import Status

                with Status("Waiting for device to reboot...", console=self.console, spinner="dots"):
                    try:
                        await asyncio.wait_for(self.disconnect_event.wait(), timeout=5.0)
                    except asyncio.TimeoutError:
                        pass

                if not self.disconnect_event.is_set():
                    self.console.print("Warning: Device did not disconnect within 5 seconds")
                    self.console.print("This may indicate an issue - check device manually")
                    return

            self.console.print("DFU complete - device rebooted successfully")

        except KeyboardInterrupt:
            # Graceful shutdown on Ctrl+C
            raise  # Re-raise to be caught by main()
        finally:
            try:
                await self.disconnect()
            except Exception:
                pass  # Device may have already disconnected due to reboot

# ============== CLI entrypoint ============================================
def main():
    parser = argparse.ArgumentParser(
        description="ESP32-S3 BLE DFU uploader for PlatformIO",
        epilog="Designed for PlatformIO upload system integration"
    )
    parser.add_argument("firmware", type=Path, help="Path to firmware .bin")
    parser.add_argument("-d", "--device", required=True, help="Exact DFU device name to target")
    parser.add_argument("-p", "--prn", type=int, default=DEFAULT_PRN, help="Packet Receipt Notification interval (packets per CRC check)")
    parser.add_argument("--chunk", type=int, default=DEFAULT_CHUNK, help="BLE packet chunk size (<= MTU)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show detailed scan progress")
    args = parser.parse_args()

    if not args.firmware.exists():
        print("❌ Firmware file not found:", args.firmware, flush=True)
        sys.exit(2)

    uploader = DFUUploader(args.firmware, args.device, prn=args.prn, chunk_size=args.chunk, verbose=args.verbose)
    try:
        asyncio.run(uploader.upload())
    except KeyboardInterrupt:
        print("\nUpload interrupted", flush=True)
        sys.exit(130)
    except Exception:
        sys.exit(1)

if __name__ == "__main__":
    main()
