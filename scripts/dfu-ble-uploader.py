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
from typing import Optional, Tuple, Any, Dict

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

    if packages_to_install:
        print(f"Installing required packages: {', '.join(packages_to_install)}...", flush=True)
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q"] + packages_to_install)

    # Final imports to expose names
    try:
        from bleak import BleakClient, BleakScanner, BleakError
    except Exception as e:
        print(f"Failed to import required packages: {e}", flush=True)
        raise

ensure_packages()

# Import bleak after ensuring it's installed
from bleak import BleakClient, BleakScanner, BleakError  # type: ignore

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
EXIT_CONNECTION_LOST = 6
EXIT_INTERRUPTED = 130  # Standard exit code for SIGINT (128 + 2)

# -----------------------------
# Exceptions
# -----------------------------
class DFUError(RuntimeError):
    """Base class for DFU-related errors."""
    exit_code = EXIT_GENERIC_ERROR

class DeviceNotFound(DFUError):
    exit_code = EXIT_DEVICE_NOT_FOUND

class ConnectionFailed(DFUError):
    exit_code = EXIT_CONNECTION_FAILED

class VerificationFailed(DFUError):
    exit_code = EXIT_VERIFICATION_FAILED

class ConnectionLost(DFUError):
    exit_code = EXIT_CONNECTION_LOST


# -----------------------------
# Utilities
# -----------------------------
class IndentLogger:
    """Logger wrapper that adds automatic indentation."""
    def __init__(self, logger: logging.Logger):
        self._logger = logger
        self._indent = 0

    def _log(self, level, msg, *args, **kwargs):
        indent_str = "  " * self._indent
        self._logger.log(level, indent_str + msg, *args, **kwargs)

    def indent(self): self._indent += 1
    def dedent(self): self._indent = max(0, self._indent - 1)
    def debug(self, msg, *args, **kwargs): self._log(logging.DEBUG, msg, *args, **kwargs)
    def info(self, msg, *args, **kwargs): self._log(logging.INFO, msg, *args, **kwargs)
    def warning(self, msg, *args, **kwargs): self._log(logging.WARNING, msg, *args, **kwargs)
    def error(self, msg, *args, **kwargs): self._log(logging.ERROR, msg, *args, **kwargs)


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
        """Called after successful command execution for verification."""
        pass

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        """Execute the command using the uploader."""
        payload = self.build()
        uploader.logger.debug("→ Sending %s command...", self.name)
        uploader.logger.indent()
        waiter_task = None
        try:
            uploader.logger.debug("Waiting for %s response...", self.name)
            waiter_task = asyncio.create_task(uploader.waiter.wait_response(self.opcode, timeout=self.timeout))
            await uploader.write_control(payload, with_response=True)
            status, resp_payload = await waiter_task

            if status != DFU_STATUS_SUCCESS:
                raise DFUError(f"{self.name} failed with status 0x{status:02x}")

            result = self.parse_response(resp_payload)
            uploader.logger.debug("Received %s response: status=0x%02X", self.name, status)
            await self.post_execute(uploader)
            return result
        except asyncio.CancelledError:
            raise
        except Exception as e:
            if self.on_error(e):
                await self.post_execute(uploader)
                return None
            raise
        finally:
            if waiter_task and not waiter_task.done():
                waiter_task.cancel()
                try:
                    await waiter_task
                except (asyncio.CancelledError, ConnectionLost):
                    pass
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

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        """Execute the flash erase operation."""
        print(f"Erasing flash ({self.size:,} bytes)...", flush=True)
        return await super().execute(uploader)


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
        if isinstance(error, (asyncio.TimeoutError, BleakError, ConnectionLost)):
            return True  # Suppress - device is rebooting, verification happens in post_execute
        return False

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        """Execute firmware activation."""
        print("Activating firmware...", flush=True)
        result = await super().execute(uploader)
        return result

    async def post_execute(self, uploader: "DFUUploader") -> None:
        """REQUIRE device reboot - fail if device doesn't disconnect."""
        print("✓ Firmware activated successfully, waiting for device to reboot...", flush=True)
        uploader.logger.debug("Waiting for device reboot (disconnect required)...")
        try:
            await asyncio.wait_for(uploader.disconnect_event.wait(), timeout=CONNECTION_REBOOT_TIMEOUT_S)
            uploader.logger.debug("Device rebooted successfully")
            print("✓ Device rebooted successfully", flush=True)
        except asyncio.TimeoutError:
            raise DFUError(f"{self.name}: Device did NOT reboot after execute - FAILURE!")


class UploadData(DFUCommand):
    """
    Uploads firmware data packets to the device.
    Uses PRN (Packet Receipt Notification) for progress verification.
    """

    def __init__(self, firmware: bytes, resume_offset: int = 0):
        super().__init__(opcode=0, name="UploadData")  # No opcode used - direct data writes
        self.firmware = firmware
        self.resume_offset = resume_offset
        self.total = len(firmware)

    def build(self) -> bytes:
        """No command packet - this uses write_packet() directly."""
        raise NotImplementedError("UploadData doesn't use build()")

    def parse_response(self, payload: bytes) -> Any:
        """No response parsing - PRN notifications handled separately."""
        raise NotImplementedError("UploadData doesn't use parse_response()")

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        """Execute firmware upload."""
        uploader.logger.debug("→ Starting firmware upload...")
        uploader.logger.indent()

        try:
            offset = self.resume_offset
            packets = offset // uploader.chunk_size
            prn_future = uploader.waiter.register_prn()

            # Upload loop
            while offset < self.total:
                chunk = self.firmware[offset: offset + uploader.chunk_size]
                await uploader.write_packet(chunk)
                offset += len(chunk)
                packets += 1

                # Soft yield to avoid BLE buffer overload
                if packets % 16 == 0:
                    await asyncio.sleep(0.01)

                # PRN handling
                if packets % uploader.prn == 0:
                    prn_off, prn_crc = await uploader.waiter.wait_prn(prn_future, timeout=12.0)
                    local_crc = zlib.crc32(self.firmware[:prn_off]) & 0xFFFFFFFF
                    if local_crc != prn_crc:
                        raise VerificationFailed(
                            f"PRN CRC mismatch at {prn_off}: "
                            f"device=0x{prn_crc:08X} local=0x{local_crc:08X}"
                        )

                    # Print CRC verification
                    pct = (prn_off * 100) // self.total
                    print(f"  CRC ok @ {prn_off:,} bytes ({pct}%)", flush=True)
                    prn_future = uploader.waiter.register_prn()

            uploader.logger.debug("✓ Firmware upload complete")
            return None

        finally:
            uploader.logger.dedent()


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

    def _parse_response_packet(self, data: bytes) -> Optional[Tuple[int, int, bytes]]:
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

    def notification_handler(self, sender: Any, data: bytes) -> None:
        """
        Handle BLE notifications from control characteristic.
        Notifications can be:
        1. Standard response packets (60 <opcode> <status> <payload>)
        2. Raw 8-byte so-called Raw PRN notifications (offset + CRC, no opcode wrapper)
        """
        # Notification logs are not indented - temporarily reset indent
        saved_indent = self._logger._indent
        self._logger._indent = 0
        try:
            self._logger.debug("RAW notification: len=%d data=%s", len(data), data.hex())
            parsed = self._parse_response_packet(data)
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

    def on_disconnect(self, loop: asyncio.AbstractEventLoop) -> None:
        """Called when device disconnects - fail all pending operations (thread-safe)."""
        def _fail_futures():
            exc = ConnectionLost("Device disconnected unexpectedly")
            # Fail all pending command responses
            for fut in list(self._resp_futures.values()):
                if not fut.done():
                    fut.set_exception(exc)
            self._resp_futures.clear()
            # Fail pending PRN
            if self._prn_future and not self._prn_future.done():
                self._prn_future.set_exception(exc)

        loop.call_soon_threadsafe(_fail_futures)


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

    async def connect(self, device: Any) -> None:
        """Single connection attempt without retry logic."""
        self.logger.debug("Connecting to %s", device.address)
        self.client = BleakClient(device.address, disconnected_callback=self._on_disconnect)
        await self.client.connect()
        await self.client.start_notify(self.ctrl_uuid, self.waiter.notification_handler)
        await asyncio.sleep(0.5)  # let subscription settle

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
        try:
            await self.client.write_gatt_char(self.ctrl_uuid, payload, response=with_response)
        except BleakError as e:
            raise ConnectionLost(f"Device disconnected during control write: {e}")

    async def write_packet(self, data: bytes) -> None:
        if not self.client:
            raise ConnectionFailed("Not connected")
        try:
            await self.client.write_gatt_char(self.data_uuid, data, response=False)
        except BleakError as e:
            raise ConnectionLost(f"Device disconnected during data write: {e}")

    def _is_tty(self) -> bool:
        return sys.stdin.isatty()

    def _format_addr(self, addr: str) -> str:
        cleaned = addr.replace(":", "").replace("-", "")
        if len(cleaned) == 32:
            # assume UUID format
            cleaned = cleaned.upper()
            return f"{cleaned[0:8]}-{cleaned[8:12]}-{cleaned[12:16]}-{cleaned[16:20]}-{cleaned[20:32]}"
        return addr

    # -------------------------
    # DFU high-level ops
    # -------------------------
    async def set_prn(self) -> None:
        await SetPrn(self.prn).execute(self)

    async def select_object(self, obj_type: int):
        return await SelectObject(obj_type).execute(self)

    async def create_object(self, obj_type: int, size: int) -> None:
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

        print(f"Starting BLE DFU upload: {total:,} bytes, CRC32=0x{zlib.crc32(firmware)&0xFFFFFFFF:08X}", flush=True)

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

        # Connect only if not already connected (with retry logic)
        if not (self.client and self.client.is_connected):
            last_exc: Optional[Exception] = None
            for attempt in range(1, MAX_CONNECTION_RETRIES + 1):
                try:
                    print(f"Connecting to {device.address}...", flush=True)
                    await self.connect(device)
                    print(f"✓ Connected to {getattr(device, 'name', device.address)} ({device.address})", flush=True)
                    break  # Success - exit retry loop
                except Exception as exc:
                    last_exc = exc
                    print(f"WARNING: Connect attempt {attempt} failed: {exc}", flush=True)

                    if attempt < MAX_CONNECTION_RETRIES:
                        await asyncio.sleep(2 ** (attempt - 1))
                        # Cleanup before retry
                        if self.client:
                            try:
                                await self.client.disconnect()
                            except Exception:
                                pass
                            self.client = None
                    else:
                        # Final attempt failed
                        raise ConnectionFailed(f"Failed to connect after {MAX_CONNECTION_RETRIES} attempts: {last_exc!r}")

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
                    print(f"Firmware already uploaded: {total:,} bytes, CRC=0x{local_crc:08X}", flush=True)
                    print("Activating firmware on device...", flush=True)
                    await self.execute_object()
                    print("✓ Firmware activated successfully — device rebooted", flush=True)
                    return
                if local_crc == dev_crc:
                    resume_offset = dev_offset
                    print(f"Resuming upload from {resume_offset:,} bytes", flush=True)
                else:
                    print("Device partial data CRC mismatch — restarting upload", flush=True)

            # Create object if starting fresh (can take ~25s for flash erase)
            if resume_offset == 0:
                await CreateObject(obj_type, total).execute(self)

            # Upload firmware data
            await UploadData(firmware, resume_offset).execute(self)

            # Final CRC verification
            calc_off, calc_crc = await CalculateCRC().execute(self)
            local_crc = zlib.crc32(firmware[:calc_off]) & 0xFFFFFFFF
            if calc_off != total or calc_crc != local_crc:
                raise VerificationFailed(
                    f"Verification failed: device_off={calc_off} device_crc=0x{calc_crc:08X} local_crc=0x{local_crc:08X}"
                )

            print(f"✓ Upload complete: {calc_off:,} bytes, CRC=0x{calc_crc:08X}", flush=True)

            # Execute firmware (verifies reboot internally)
            await ExecuteObject().execute(self)

            print("✓ Firmware successfully activated — device rebooted", flush=True)

        finally:
            # Always try to disconnect / cleanup
            try:
                await self.disconnect()
            except Exception:
                self.logger.debug("Disconnect during cleanup failed", exc_info=True)

    # BLE callback
    def _on_disconnect(self, client):
        # Get the event loop (this callback may be called from the BLE thread)
        try:
            loop = asyncio.get_running_loop()
            # Thread-safe: schedule disconnect_event.set() and fail pending futures
            loop.call_soon_threadsafe(self.disconnect_event.set)
            self.waiter.on_disconnect(loop)
        except RuntimeError:
            # No running loop - already shutting down
            pass


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
                        # Simple fuzzy match: check the first two chars
                        if len(typo) >= 2:
                            suggestions = [v for v in valid if v.startswith(typo[:2])]
                            if suggestions:
                                print(f"\nDid you mean '--log-level {suggestions[0]}'?", file=sys.stderr)
                except (ValueError, IndexError):
                    pass
        raise

    # Logging
    log_level = {"debug": logging.DEBUG, "info": logging.INFO, "warning": logging.WARNING, "error": logging.ERROR}[args.log_level]

    # Configure logging: call early to configure logging for CLI usage."""
    fmt = "%(asctime)s %(levelname)s [%(name)s] %(message)s" if log_level <= logging.DEBUG else "%(levelname)s: %(message)s"
    logging.basicConfig(level=logging.WARNING, format=fmt, stream=sys.stdout)
    logging.getLogger("DFUUploader").setLevel(log_level)
    logging.getLogger("dfu_cli").setLevel(log_level)
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
    except DFUError as e:
        logger.error("%s", e)
        return e.exit_code
    except (KeyboardInterrupt, asyncio.CancelledError):
        logger.error("Interrupted by user")
        return EXIT_INTERRUPTED
    except Exception as e:
        logger.exception("Unexpected error during DFU: %s", e)
        return EXIT_GENERIC_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
