#!/usr/bin/env python3
"""
ESP32-S3 BLE DFU Firmware Uploader for PlatformIO

Uploads firmware to ESP32-S3 devices over BLE using an adapted 'Nordic DFU' protocol.
Designed to integrate with PlatformIO's upload system.

Usage:
    python dfu-ble-uploader.py firmware.bin -d "BLIM IMU Stream" [-p 16] [-v]

PlatformIO.ini configuration:
    upload_protocol = custom
    upload_command = python dfu-ble-uploader.py $SOURCE -d
"""
from __future__ import annotations

import sys
import os
import subprocess
import argparse
import asyncio
import zlib
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Any

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
        from bleak import BleakClient, BleakScanner, BleakError
        from rich.console import Console
        from rich.progress import Progress, BarColumn, TextColumn, TimeRemainingColumn, TransferSpeedColumn
        from rich.live import Live
        from rich.layout import Layout
        from rich.text import Text
    except Exception as e:
        print(f"Failed to import required packages: {e}", flush=True)
        raise

ensure_packages()

# import argparse
# import asyncio
# import logging
# import os
# import sys
# import zlib
# from dataclasses import dataclass
# from pathlib import Path
# from typing import Optional, Tuple, Dict, Any, Callable

# Third-party imports are intentionally at top-level so missing deps fail fast.
try:
    from bleak import BleakClient, BleakScanner, BleakError  # type: ignore
except Exception as exc:  # pragma: no cover - runtime environment
    print("Missing dependency: 'bleak'. Install with: pip install bleak", file=sys.stderr)
    raise

try:
    from rich.console import Console
    from rich.progress import Progress, BarColumn, TextColumn, TimeRemainingColumn, TransferSpeedColumn
    from rich.live import Live
    from rich.text import Text
    from rich.panel import Panel
except Exception as exc:  # pragma: no cover - runtime environment
    print("Missing dependency: 'rich'. Install with: pip install rich", file=sys.stderr)
    raise

# -----------------------------
# Constants and protocol values
# -----------------------------
DFU_SERVICE_UUID = "0000fe59-0000-1000-8000-00805f9b34fb"
DFU_CTRL_UUID = "8ec90001-f315-4f60-9fb8-838830daea50"
DFU_DATA_UUID = "8ec90002-f315-4f60-9fb8-838830daea50"

OP_CREATE_OBJECT = 0x01
OP_SET_RECEIPT = 0x02
OP_CALCULATE_CRC = 0x03
OP_EXECUTE_OBJECT = 0x04
OP_SELECT_OBJECT = 0x06
OP_RESPONSE = 0x60

DFU_OBJ_DATA = 0x02

DFU_STATUS_SUCCESS = 0x01

DEFAULT_PRN = 32
DEFAULT_CHUNK = 512
MAX_FIRMWARE_SIZE = 4 * 1024 * 1024  # 4MB

MAX_CONNECTION_RETRIES = 3
CONNECTION_REBOOT_TIMEOUT_S = 10.0

# Exit codes
EXIT_SUCCESS = 0
EXIT_GENERIC_ERROR = 1
EXIT_USAGE_ERROR = 2  # argparse uses 2 for command-line usage errors
EXIT_DEVICE_NOT_FOUND = 3
EXIT_VERIFICATION_FAILED = 4
EXIT_CONNECTION_FAILED = 5
EXIT_INTERRUPTED = 130  # Standard exit code for SIGINT (128 + 2)

# -----------------------------
# Exceptions
# -----------------------------
class DFUError(RuntimeError):
    """Base class for DFU-related errors."""


class DeviceNotFound(DFUError):
    pass


class ConnectionFailed(DFUError):
    pass


class VerificationFailed(DFUError):
    pass


# -----------------------------
# Utilities
# -----------------------------
class IndentLogger:
    """Logger wrapper that adds automatic indentation."""
    def __init__(self, logger: logging.Logger):
        self._logger = logger
        self._indent = 0

    def indent(self):
        self._indent += 1

    def dedent(self):
        self._indent = max(0, self._indent - 1)

    def _log(self, level, msg, *args, **kwargs):
        indent_str = "  " * self._indent
        self._logger.log(level, indent_str + msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self._log(logging.DEBUG, msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self._log(logging.INFO, msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self._log(logging.WARNING, msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self._log(logging.ERROR, msg, *args, **kwargs)


def configure_logging(level: int = logging.WARNING) -> None:
    """Call early to configure logging for CLI usage."""
    fmt = "%(asctime)s %(levelname)s [%(name)s] %(message)s" if level <= logging.DEBUG else "%(levelname)s: %(message)s"
    # Set root logger to WARNING to silence all third-party libraries
    # Use stderr to avoid interfering with the Rich console on stdout
    logging.basicConfig(level=logging.WARNING, format=fmt, stream=sys.stderr)
    # Only enable our own loggers at the requested level
    logging.getLogger("DFUUploader").setLevel(level)
    logging.getLogger("dfu_cli").setLevel(level)


def parse_response_packet(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    """
    Parse an incoming control notification into (req_opcode, status, payload).
    Returns None if not a DFU response packet.
    """
    if not data or len(data) < 3:
        return None
    if data[0] != OP_RESPONSE:
        return None
    req_opcode = data[1]
    status = data[2]
    payload = data[3:] if len(data) > 3 else b""
    return req_opcode, status, payload


# -----------------------------
# Active command pattern
# -----------------------------
@dataclass
class DFUCommand:
    opcode: int
    name: str
    timeout: float = 5.0

    def on_error(self, error: Exception) -> bool:
        """
        Called when the command encounters an error during execution.

        Args:
            error: The exception that occurred

        Returns:
            True if the error was handled (suppress and continue)
            False to re-raise the error
        """
        return False  # Default: re-raise all errors

    async def post_execute(self, uploader: "DFUUploader") -> None:
        """
        Called after successful command execution (after response received).
        Subclasses can override to add verification steps.

        Args:
            uploader: The DFUUploader instance
        """
        pass  # Default: no post-execution steps

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        """
        Execute the command using the uploader. Subclasses implement `build()` and `parse_response()`.
        """
        payload = self.build()
        uploader.logger.debug("→ Sending %s command...", self.name)
        uploader.logger.indent()
        try:
            uploader.logger.debug("Waiting for %s response...", self.name)
            waiter_task = asyncio.create_task(uploader.waiter.wait_response(self.opcode, timeout=self.timeout))
            await uploader.write_control(payload, with_response=True)
            status, resp_payload = await waiter_task

            if status != DFU_STATUS_SUCCESS:
                raise DFUError(f"{self.name} failed with status 0x{status:02x}")

            result = self.parse_response(resp_payload)
            uploader.logger.debug("Received %s response: status=0x%02X", self.name, status)

            # Template method hook for post-execution verification
            await self.post_execute(uploader)

            return result
        except asyncio.CancelledError:
            # Always propagate cancellation immediately (Ctrl+C, task.cancel(), etc.)
            raise
        except Exception as e:
            # Give subclass a chance to handle the error
            if self.on_error(e):
                # Error was suppressed - still run post_execute verification
                await self.post_execute(uploader)
                return None
            raise  # Re-raise if not handled
        finally:
            uploader.logger.dedent()

    def build(self) -> bytes:
        raise NotImplementedError

    def parse_response(self, payload: bytes) -> Optional[Any]:
        return None


@dataclass
class SetPrn(DFUCommand):
    prn: int = DEFAULT_PRN

    def __init__(self, prn: int = DEFAULT_PRN):
        super().__init__(OP_SET_RECEIPT, "SET_PRN")
        self.prn = prn

    def build(self) -> bytes:
        return bytes([self.opcode]) + int(self.prn).to_bytes(2, "little")


@dataclass
class SelectObject(DFUCommand):
    obj_type: int = DFU_OBJ_DATA

    def __init__(self, obj_type: int = DFU_OBJ_DATA):
        super().__init__(OP_SELECT_OBJECT, "SELECT_OBJECT")
        self.obj_type = obj_type

    def build(self) -> bytes:
        return bytes([self.opcode, self.obj_type])

    def parse_response(self, payload: bytes):
        if len(payload) >= 8:
            offset = int.from_bytes(payload[0:4], "little")
            crc = int.from_bytes(payload[4:8], "little")
            max_size = int.from_bytes(payload[8:12], "little") if len(payload) >= 12 else None
            return offset, crc, max_size
        return 0, 0, None


@dataclass
class CreateObject(DFUCommand):
    obj_type: int = DFU_OBJ_DATA
    size: int = 0

    def __init__(self, obj_type: int = DFU_OBJ_DATA, size: int = 0):
        super().__init__(OP_CREATE_OBJECT, "CREATE_OBJECT", timeout=30.0)  # Erase can take ~25s
        self.obj_type = obj_type
        self.size = size

    def build(self) -> bytes:
        return bytes([self.opcode, self.obj_type]) + int(self.size).to_bytes(4, "little")


@dataclass
class CalculateCRC(DFUCommand):
    def __init__(self):
        super().__init__(OP_CALCULATE_CRC, "CALCULATE_CRC", timeout=CONNECTION_REBOOT_TIMEOUT_S)

    def build(self) -> bytes:
        return bytes([self.opcode])

    def parse_response(self, payload: bytes):
        if len(payload) >= 8:
            offset = int.from_bytes(payload[0:4], "little")
            crc = int.from_bytes(payload[4:8], "little")
            return offset, crc
        raise DFUError("CALCULATE_CRC returned unexpected payload length")


@dataclass
class ExecuteObject(DFUCommand):
    def __init__(self):
        super().__init__(OP_EXECUTE_OBJECT, "EXECUTE_OBJECT", timeout=10.0)

    def build(self) -> bytes:
        return bytes([self.opcode])

    def on_error(self, error: Exception) -> bool:
        """Device may disconnect during GATT write before we get DFU response - this is OK."""
        if isinstance(error, (asyncio.TimeoutError, BleakError)):
            return True  # Suppress - device is rebooting, verification happens in post_execute
        return False

    async def post_execute(self, uploader: "DFUUploader") -> None:
        """REQUIRE device reboot - fail if device doesn't disconnect."""
        uploader.logger.debug("Waiting for device reboot (disconnect required)...")
        try:
            await asyncio.wait_for(uploader.disconnect_event.wait(), timeout=5.0)
            uploader.logger.debug("Device rebooted successfully")
        except asyncio.TimeoutError:
            raise DFUError(f"{self.name}: Device did NOT reboot after execute - FAILURE!")


# -----------------------------
# Control wait / notification
# -----------------------------
class ControlWaiter:
    """
    Handles mapping request opcode -> asyncio.Future for responses, plus PRN handling.
    """

    def __init__(self, logger: IndentLogger):
        self._logger = logger
        self._resp_futures: Dict[int, asyncio.Future] = {}
        self._prn_future: Optional[asyncio.Future] = None
        self._lock = asyncio.Lock()

    def notification_handler(self, sender: Any, data: bytes) -> None:
        """
        Handle BLE notifications from control characteristic.
        Notifications can be:
        1. Standard response packets (60 <opcode> <status> <payload>)
        2. Raw 8-byte so called Raw PRN notifications (offset + CRC, no opcode wrapper)
        """
        # Notification logs are not indented - temporarily reset indent
        saved_indent = self._logger._indent
        self._logger._indent = 0
        try:
            self._logger.debug("RAW notification: len=%d data=%s", len(data), data.hex())
            parsed = parse_response_packet(data)
            if parsed:
                req_opcode, status, payload = parsed
                self._logger.debug("Parsed: opcode=0x%02X status=0x%02X payload_len=%d", req_opcode, status, len(payload))
                self._logger.debug("Command response for opcode=0x%02X", req_opcode)
        finally:
            self._logger._indent = saved_indent

        # Try to parse as a standard response packet (starts with OP_RESPONSE 0x60)
        if parsed:
            req_opcode, status, payload = parsed

            # Priority 1: Check if command response future exists - this takes precedence
            # Command responses (11 bytes) must be delivered to wait_response(), not consumed as PRN
            fut = self._resp_futures.pop(req_opcode, None)
            if fut and not fut.done():
                fut.set_result((status, payload))
                return

            # Priority 2: If no command waiting, treat CALCULATE_CRC as PRN notification
            # This handles intermediate progress updates during upload (also 11 bytes but no waiter)
            if req_opcode == OP_CALCULATE_CRC and self._prn_future and not self._prn_future.done():
                if len(payload) >= 8:
                    offset = int.from_bytes(payload[0:4], "little")
                    crc = int.from_bytes(payload[4:8], "little")
                    self._prn_future.set_result((offset, crc))
                    return

            # No handler found for this response
            self._logger.warning("No future for opcode 0x%02X", req_opcode)
            return

        # Try to parse as raw 8-byte OOB PRN notification (offset + CRC, no wrapper)
        if len(data) == 8 and self._prn_future and not self._prn_future.done():
            offset = int.from_bytes(data[0:4], "little")
            crc = int.from_bytes(data[4:8], "little")
            saved_indent = self._logger._indent
            self._logger._indent = 0
            try:
                self._logger.debug("OOB PRN notification: offset=%d crc=0x%08X", offset, crc)
            finally:
                self._logger._indent = saved_indent
            self._prn_future.set_result((offset, crc))
            return

        # Unknown notification format
        self._logger.debug("Ignoring unknown notification")

    async def wait_response(self, req_opcode: int, timeout: float = 5.0) -> Tuple[int, bytes]:
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._resp_futures[req_opcode] = fut
        try:
            status, payload = await asyncio.wait_for(fut, timeout=timeout)
            return status, payload
        finally:
            self._resp_futures.pop(req_opcode, None)

    def register_prn(self) -> asyncio.Future:
        loop = asyncio.get_running_loop()
        self._prn_future = loop.create_future()
        return self._prn_future

    async def wait_prn(self, prn_future: asyncio.Future, timeout: float = 12.0) -> Tuple[int, int]:
        try:
            return await asyncio.wait_for(prn_future, timeout=timeout)
        finally:
            self._prn_future = None


# -----------------------------
# DFU Uploader
# -----------------------------
class DFUUploader:
    def __init__(
            self,
            firmware_path: Path,
            device_address: Optional[str] = None,
            device_name: Optional[str] = None,
            prn: int = DEFAULT_PRN,
            chunk_size: int = DEFAULT_CHUNK,
            verbose: bool = False,
    ):
        self.firmware_path = Path(firmware_path)
        self.device_address = device_address
        self.device_name = device_name
        self.prn = prn
        self.chunk_size = chunk_size
        self.verbose = verbose
        self.client: Optional[BleakClient] = None
        self.ctrl_uuid = DFU_CTRL_UUID
        self.data_uuid = DFU_DATA_UUID
        self.logger = IndentLogger(logging.getLogger(self.__class__.__name__))
        self.waiter = ControlWaiter(self.logger)
        self.disconnect_event = asyncio.Event()
        self.console = Console()
        self.detail_lines: list[str] = []
        self.show_details = False
        self.stdin_fd = None
        self.old_tty_settings = None
        # Interactive mode: progress bar with details toggle (disabled in debug mode or non-TTY)
        self.interactive_mode = self.logger._logger.level > logging.DEBUG and self._is_tty()

    # -------------------------
    # Low-level BLE helpers
    # -------------------------
    async def scan_for_device(self, timeout: int = 10) -> Any:
        """
        Scan for the device by name and DFU service. Returns Bleak device object or raises DeviceNotFound.
        """
        self.logger.debug("Scanning for device name=%s timeout=%s", self.device_name, timeout)
        dfu_device = None
        found_event = asyncio.Event()

        def detection_cb(device, adv):
            nonlocal dfu_device
            if device.name == self.device_name and DFU_SERVICE_UUID.lower() in [s.lower() for s in (adv.service_uuids or [])]:
                dfu_device = device
                found_event.set()

        async with BleakScanner(detection_callback=detection_cb) as scanner:
            try:
                await asyncio.wait_for(found_event.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                pass

        if not dfu_device:
            raise DeviceNotFound(f"Device named '{self.device_name}' advertising DFU service not found")
        return dfu_device

    async def connect(self, device: Any, max_retries: int = MAX_CONNECTION_RETRIES) -> None:
        last_exc: Optional[Exception] = None
        for attempt in range(1, max_retries + 1):
            try:
                self.logger.debug("Connecting attempt %d to %s", attempt, device.address)
                self.client = BleakClient(device.address, disconnected_callback=self._on_disconnect)
                await self.client.connect()
                await self.client.start_notify(self.ctrl_uuid, self.waiter.notification_handler)
                await asyncio.sleep(0.5)  # let subscription settle
                self.console.print(f"Connected to {getattr(device, 'name', device.address)} ({device.address})")
                return
            except Exception as exc:
                last_exc = exc
                self.logger.warning("Connect attempt %d failed: %s", attempt, exc)
                await asyncio.sleep(2 ** (attempt - 1))
                if self.client:
                    try:
                        await self.client.disconnect()
                    except Exception:
                        pass
                    self.client = None
        raise ConnectionFailed(f"Failed to connect to {getattr(device, 'address', device)} after {max_retries} attempts: {last_exc!r}")

    async def disconnect(self) -> None:
        if self.client:
            # Check if still connected before attempting cleanup
            if self.client.is_connected:
                # Note: During Ctrl+C, cleanup may not complete due to cancellation
                # We use short timeouts to avoid blocking shutdown
                try:
                    await asyncio.wait_for(self.client.stop_notify(self.ctrl_uuid), timeout=0.5)
                except (asyncio.TimeoutError, asyncio.CancelledError):
                    # Timeout or cancellation during cleanup is acceptable
                    pass
                except Exception:
                    self.logger.debug("stop_notify failed")

                try:
                    await asyncio.wait_for(self.client.disconnect(), timeout=0.5)
                except (asyncio.TimeoutError, asyncio.CancelledError):
                    # Timeout or cancellation during cleanup is acceptable
                    pass
                except Exception:
                    self.logger.debug("disconnect failed")
            else:
                self.logger.debug("Cleanup: device already disconnected")
            self.client = None

    async def write_control(self, payload: bytes, with_response: bool = True) -> None:
        if not self.client:
            raise ConnectionFailed("Not connected")
        await self.client.write_gatt_char(self.ctrl_uuid, payload, response=with_response)

    async def write_packet(self, data: bytes) -> None:
        if not self.client:
            raise ConnectionFailed("Not connected")
        await self.client.write_gatt_char(self.data_uuid, data, response=False)

    def _add_detail(self, text: str) -> None:
        self.detail_lines.append(text)
        if len(self.detail_lines) > 50:
            self.detail_lines.pop(0)

    def _is_tty(self) -> bool:
        return sys.stdin.isatty()

    def _format_addr(self, addr: str) -> str:
        cleaned = addr.replace(":", "").replace("-", "")
        if len(cleaned) == 32:
            # assume UUID format
            cleaned = cleaned.upper()
            return f"{cleaned[0:8]}-{cleaned[8:12]}-{cleaned[12:16]}-{cleaned[16:20]}-{cleaned[20:32]}"
        return addr

    def _setup_stdin_reader(self, loop: asyncio.AbstractEventLoop) -> None:
        """Setup non-blocking stdin reader for 'o' key detection in interactive mode."""
        import termios
        import tty

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.old_tty_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
            loop.add_reader(self.stdin_fd, self._handle_stdin_input)
        except Exception as e:
            self.logger.debug("Failed to setup stdin reader: %s", e)

    def _handle_stdin_input(self) -> None:
        """Handle stdin input - called by asyncio when data is available."""
        try:
            ch = sys.stdin.read(1)
            if ch.lower() == 'o':
                self.show_details = not self.show_details
        except Exception:
            pass

    def _cleanup_stdin(self, loop: asyncio.AbstractEventLoop) -> None:
        """Restore terminal settings after interactive mode."""
        import termios

        try:
            if self.stdin_fd is not None:
                loop.remove_reader(self.stdin_fd)
                if self.old_tty_settings is not None:
                    termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_tty_settings)
        except Exception:
            pass

    # -------------------------
    # DFU high-level ops
    # -------------------------
    async def set_prn(self) -> None:
        await SetPrn(self.prn).execute(self)

    async def select_object(self, obj_type: int):
        return await SelectObject(obj_type).execute(self)

    async def create_object(self, obj_type: int, size: int) -> None:
        self._add_detail(f"create object: type={obj_type} size={size}")
        await CreateObject(obj_type, size).execute(self)

    async def calculate_crc(self):
        return await CalculateCRC().execute(self)

    async def execute_object(self) -> None:
        await ExecuteObject().execute(self)

    # -------------------------
    # Orchestration: upload
    # -------------------------
    async def upload(self) -> None:
        # Validate firmware
        if not self.firmware_path.exists() or not self.firmware_path.is_file():
            raise DFUError(f"Firmware not found: {self.firmware_path}")
        firmware = self.firmware_path.read_bytes()
        total = len(firmware)
        if total == 0:
            raise DFUError("Firmware file is empty")
        if total > MAX_FIRMWARE_SIZE:
            raise DFUError(f"Firmware too large: {total} bytes (max {MAX_FIRMWARE_SIZE})")

        self.console.print(f"Starting BLE DFU upload: {total:,} bytes, CRC32=0x{zlib.crc32(firmware)&0xFFFFFFFF:08X}")

        # Determine device
        if self.device_address:
            addr = self._format_addr(self.device_address)
            # Build a minimal device-like object
            class _Device:
                def __init__(self, address):
                    self.address = address
                    self.name = f"Device@{address[:8]}"
            device = _Device(addr)
        else:
            device = await self.scan_for_device(timeout=10)

        # Connect with spinner in interactive mode
        if self.interactive_mode:
            from rich.status import Status
            with Status(f"Connecting to {device.address}...", console=self.console, spinner="dots"):
                await self.connect(device)
            self.console.print(f"✓ Connected to {getattr(device, 'name', device.address)} ({device.address})", style="green")
        else:
            await self.connect(device)

        try:
            # Set PRN
            await self.set_prn()

            # Object type
            obj_type = DFU_OBJ_DATA

            # Try resume
            dev_offset, dev_crc, _ = await self.select_object(obj_type)
            resume_offset = 0
            if dev_offset > 0:
                local_crc = zlib.crc32(firmware[:dev_offset]) & 0xFFFFFFFF
                if dev_offset == total and local_crc == dev_crc:
                    self.console.print(f"Firmware already uploaded: {total:,} bytes, CRC=0x{local_crc:08X}")
                    self.console.print("Activating firmware on device...")
                    await self.execute_object()
                    self.console.print("Firmware activated successfully — device rebooted")
                    return
                if local_crc == dev_crc:
                    resume_offset = dev_offset
                    self.console.print(f"Resuming upload from {resume_offset} bytes")
                else:
                    self.console.print("Device partial data CRC mismatch — restarting upload")

            # Create object if starting fresh (can take ~25s for flash erase)
            if resume_offset == 0:
                if self.interactive_mode:
                    from rich.status import Status
                    with Status("Preparing for firmware upload...", console=self.console, spinner="dots"):
                        await self.create_object(obj_type, total)
                else:
                    await self.create_object(obj_type, total)

            # Setup stdin reader for interactive mode (only with progress bar)
            loop = asyncio.get_running_loop()
            if self.interactive_mode:
                self._setup_stdin_reader(loop)

            try:
                if self.interactive_mode:
                    # Interactive mode with rich progress bar
                    from rich.console import Group

                    progress = Progress(
                        TextColumn("[progress.description]{task.description}"),
                        BarColumn(),
                        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
                        TextColumn("{task.completed}/{task.total} bytes"),
                        TransferSpeedColumn(),
                        TimeRemainingColumn(),
                        console=self.console,
                    )
                    task = progress.add_task("Uploading", total=total, completed=resume_offset)
                    upload_complete = False

                    def build_display():
                        """Build display with a progress bar and optional details."""
                        elements = [progress]
                        if self.show_details and self.detail_lines:
                            elements.append(Text("\nDetails (press 'o' to hide):"))
                            for line in self.detail_lines:
                                elements.append(Text(f"  {line}"))
                        elif not upload_complete:
                            elements.append(Text("\nPress 'o' to show details"))
                        return Group(*elements)

                    with Live(build_display(), console=self.console, refresh_per_second=4) as live:
                        offset = resume_offset
                        packets = offset // self.chunk_size
                        prn_future = self.waiter.register_prn()

                        while offset < total:
                            chunk = firmware[offset: offset + self.chunk_size]
                            await self.write_packet(chunk)
                            offset += len(chunk)
                            packets += 1

                            progress.update(task, completed=offset)
                            live.update(build_display())

                            # Soft yield to avoid BLE buffer overload
                            if packets % 16 == 0:
                                await asyncio.sleep(0.01)

                            # PRN handling
                            if packets % self.prn == 0:
                                prn_off, prn_crc = await self.waiter.wait_prn(prn_future, timeout=12.0)
                                local_crc = zlib.crc32(firmware[:prn_off]) & 0xFFFFFFFF
                                if local_crc != prn_crc:
                                    raise VerificationFailed(f"PRN CRC mismatch at {prn_off}: device=0x{prn_crc:08X} local=0x{local_crc:08X}")
                                self._add_detail(f"CRC ok @ {prn_off}")
                                live.update(build_display())
                                prn_future = self.waiter.register_prn()

                        # Upload complete - update display one last time without the help text
                        upload_complete = True
                        live.update(build_display())

                else:
                    # Non-interactive mode - just log progress
                    offset = resume_offset
                    packets = offset // self.chunk_size
                    prn_future = self.waiter.register_prn()

                    while offset < total:
                        chunk = firmware[offset: offset + self.chunk_size]
                        await self.write_packet(chunk)
                        offset += len(chunk)
                        packets += 1

                        # Soft yield to avoid BLE buffer overload
                        if packets % 16 == 0:
                            await asyncio.sleep(0.01)

                        # PRN handling
                        if packets % self.prn == 0:
                            prn_off, prn_crc = await self.waiter.wait_prn(prn_future, timeout=12.0)
                            local_crc = zlib.crc32(firmware[:prn_off]) & 0xFFFFFFFF
                            if local_crc != prn_crc:
                                raise VerificationFailed(f"PRN CRC mismatch at {prn_off}: device=0x{prn_crc:08X} local=0x{local_crc:08X}")
                            self.logger.info("Progress: %d/%d bytes (%.1f%%)", prn_off, total, 100.0 * prn_off / total)
                            prn_future = self.waiter.register_prn()
            finally:
                # Clean up the stdin reader if we set it up
                if self.interactive_mode:
                    self._cleanup_stdin(loop)

            # Final CRC verification
            calc_off, calc_crc = await self.calculate_crc()
            local_crc = zlib.crc32(firmware[:calc_off]) & 0xFFFFFFFF
            if calc_off != total or calc_crc != local_crc:
                raise VerificationFailed(
                    f"Verification failed: device_off={calc_off} device_crc=0x{calc_crc:08X} local_crc=0x{local_crc:08X}"
                )

            self.console.print(f"Upload complete: {calc_off} bytes, CRC=0x{calc_crc:08X}")

            # Execute firmware (verifies reboot internally)
            await self.execute_object()

            self.console.print("Firmware successfully activated — device rebooted")

        finally:
            # Always try to disconnect / cleanup
            try:
                await self.disconnect()
            except Exception:
                self.logger.debug("Disconnect during cleanup failed", exc_info=True)

    # BLE callback
    def _on_disconnect(self, client):
        self.disconnect_event.set()


# -----------------------------
# CLI Entrypoint
# -----------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ESP32-S3 BLE DFU uploader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s firmware.bin -d E20E664A-4716-ABA3-ABC6-B9A0329B5B2E
  %(prog)s firmware.bin -n "Device Name"
  %(prog)s firmware.bin -d e20e664a4716aba3abc6b9a0329b5b2e --log-level debug
"""
    )
    p.add_argument("firmware", type=Path, help="Path to firmware .bin")
    p.add_argument("-d", "--device", dest="device", help="BLE device address (or set BLE_DEVICE_ADDRESS env var)")
    p.add_argument("-n", "--name", dest="name", help="BLE device name (scan)")
    p.add_argument("-p", "--prn", type=int, default=DEFAULT_PRN, help="Packet receipt notification interval")
    p.add_argument("--chunk", type=int, default=DEFAULT_CHUNK, help="BLE packet chunk size")
    p.add_argument("-v", "--verbose", action="store_true", help="Verbose (console output)")
    p.add_argument("--log-level", choices=["debug", "info", "warning", "error"], default="warning", help="Logging level")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    parser = build_arg_parser()

    # Custom error handling for better user experience
    try:
        args = parser.parse_args(argv)
    except SystemExit as e:
        # Handle argparse usage errors
        if e.code == EXIT_USAGE_ERROR:
            # Check if it's a log-level typo and suggest a correction
            if "--log-level" in argv:
                try:
                    log_idx = argv.index("--log-level")
                    if log_idx + 1 < len(argv):
                        typo = argv[log_idx + 1].lower()
                        valid = ["debug", "info", "warning", "error"]
                        # Simple fuzzy match: check first two chars
                        if len(typo) >= 2:
                            suggestions = [v for v in valid if v.startswith(typo[:2])]
                            if suggestions:
                                print(f"\nDid you mean '--log-level {suggestions[0]}'?", file=sys.stderr)
                except (ValueError, IndexError):
                    pass
        raise

    # Logging
    log_level = {"debug": logging.DEBUG, "info": logging.INFO, "warning": logging.WARNING, "error": logging.ERROR}[args.log_level]
    configure_logging(level=log_level)
    logger = logging.getLogger("dfu_cli")

    device_address = args.device or os.environ.get("BLE_DEVICE_ADDRESS")
    device_name = args.name

    if not device_address and not device_name:
        logger.error("Device is required (address or name).")
        parser.print_usage()
        return EXIT_USAGE_ERROR

    if device_address and device_name:
        logger.error("Specify either device address (-d) OR name (-n), not both.")
        return EXIT_USAGE_ERROR

    if not args.firmware.exists():
        logger.error("Firmware file not found: %s", args.firmware)
        return EXIT_USAGE_ERROR

    uploader = DFUUploader(
        firmware_path=args.firmware,
        device_address=device_address,
        device_name=device_name,
        prn=args.prn,
        chunk_size=args.chunk,
        verbose=args.verbose,
    )

    try:
        asyncio.run(uploader.upload())
        return EXIT_SUCCESS
    except DeviceNotFound as e:
        logger.error("Device not found: %s", e)
        return EXIT_DEVICE_NOT_FOUND
    except VerificationFailed as e:
        logger.error("Verification failed: %s", e)
        return EXIT_VERIFICATION_FAILED
    except ConnectionFailed as e:
        logger.error("Connection failed: %s", e)
        return EXIT_CONNECTION_FAILED
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.info("Interrupted by user")
        return EXIT_INTERRUPTED
    except Exception as e:
        logger.exception("Unexpected error during DFU: %s", e)
        return EXIT_GENERIC_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
