#!/usr/bin/env python3
"""
ESP32-S3 BLE DFU Firmware Uploader for PlatformIO

Uploads firmware to ESP32-S3 devices over BLE using an adapted 'Nordic DFU' protocol.
Designed to integrate with PlatformIO's upload system.

Usage:
    python dfu-ble-uploader.py firmware.bin -d <address> [-p 16] [-v]

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
# Auto-relaunch mechanism: If PlatformIO's managed Python environment exists and we're
# not yet running in it, replace the current process with the venv Python interpreter.
# This ensures dependencies installed in PlatformIO's environment are available without
# requiring manual activation. Uses os.execv() to replace the process, preserving all args.
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

import argparse
import asyncio
import logging
import os
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

# ----------------------------
# External dependency
# ----------------------------
try:
    from bleak import BleakClient, BleakScanner, BleakError  # type: ignore
    from bleak.backends.device import BLEDevice  # type: ignore
    from bleak.exc import BleakCharacteristicNotFoundError  # type: ignore
except Exception as e:  # pragma: no cover - dependency error path
    raise SystemExit(
        "Missing dependency 'bleak'. Install with: pip install bleak\n"
        "Or run inside PlatformIO penv.\n"
        f"Original error: {e}"
    )

# ----------------------------
# Constants, exit codes
# ----------------------------
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
MAX_FIRMWARE_SIZE = 4 * 1024 * 1024

MAX_CONNECTION_RETRIES = 3
CONNECTION_REBOOT_TIMEOUT_S = 10.0

EXIT_SUCCESS = 0
EXIT_GENERIC_ERROR = 1
EXIT_USAGE_ERROR = 2
EXIT_DEVICE_NOT_FOUND = 3
EXIT_VERIFICATION_FAILED = 4
EXIT_CONNECTION_FAILED = 5
EXIT_CONNECTION_LOST = 6
EXIT_INTERRUPTED = 130

# ----------------------------
# ESP32 CRC32 compatibility
# ----------------------------
# CRC-32 (reflected) implementation matching esp_rom_crc32_le
# - polynomial (reflected): 0xEDB88320
# - initial CRC value: 0xFFFFFFFF
# - NO final XOR (esp_rom_crc32_le returns raw accumulator)
#
# Device OTA code:
#   cached_crc = 0xFFFFFFFF (init)
#   cached_crc = esp_crc32_le(cached_crc, data, len)  <- accumulates, no XOR
#   getCrc() returns ~cached_crc  <- final XOR applied here

def _make_crc32_table() -> list:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc & 0xFFFFFFFF)
    return table

_CRC32_TABLE = _make_crc32_table()

def esp_crc32(data: bytes) -> int:
    """
    Compute CRC-32 matching ESP32's esp_crc32_le() with device OTA behavior.

    esp_crc32_le uses:
      - Polynomial: 0xEDB88320 (IEEE 802.3 reflected)
      - Init: 0x00000000 (NOT 0xFFFFFFFF!)
      - No final XOR

    Device code passes 0xFFFFFFFF as init, but esp_crc32_le ignores it
    and always starts from 0. The device getCrc() returns ~cached_crc,
    but since there's no init XOR, the result is just the raw CRC.
    """
    c = 0  # esp_crc32_le uses init=0 internally
    for b in data:
        idx = (c ^ b) & 0xFF
        c = (_CRC32_TABLE[idx] ^ (c >> 8)) & 0xFFFFFFFF
    return c  # No final XOR - matches device output directly


# ----------------------------
# Logging helper
# ----------------------------
logging.basicConfig(
    level=logging.CRITICAL, format="%(asctime)s %(levelname)s [%(name)s] %(message)s"
)
logger = logging.getLogger("dfu_cli")


# ----------------------------
# Exceptions
# ----------------------------
class DFUError(RuntimeError):
    exit_code = EXIT_GENERIC_ERROR


class DeviceNotFound(DFUError):
    exit_code = EXIT_DEVICE_NOT_FOUND


class ConnectionFailed(DFUError):
    exit_code = EXIT_CONNECTION_FAILED


class ServiceNotFound(ConnectionFailed):
    """DFU service/characteristic not found - non-retryable."""
    pass


class VerificationFailed(DFUError):
    exit_code = EXIT_VERIFICATION_FAILED


class ConnectionLost(DFUError):
    exit_code = EXIT_CONNECTION_LOST


# ----------------------------
# IndentLogger (kept for readability)
# ----------------------------
class IndentLogger:
    def __init__(self, logger_obj: logging.Logger) -> None:
        self._logger: logging.Logger = logger_obj
        self._indent: int = 0

    def _fmt(self, msg: str) -> str:
        return ("  " * self._indent) + msg

    def indent(self) -> None:
        self._indent += 1

    def dedent(self) -> None:
        self._indent = max(0, self._indent - 1)

    def debug(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.debug(self._fmt(msg), *args, **kwargs)

    def info(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.info(self._fmt(msg), *args, **kwargs)

    def warning(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.warning(self._fmt(msg), *args, **kwargs)

    def error(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._logger.error(self._fmt(msg), *args, **kwargs)


# ----------------------------
# Transport layer (BLE abstraction)
# ----------------------------
class BLETransport:
    """
    Abstracts Bleak-specific details and provides a simple async interface.

    Responsibilities:
    - Connect/disconnect
    - Start/stop notifications on control characteristic
    - Write control and data characteristics
    - Manage disconnected callback mapping into the asyncio loop safely

    Thread Safety:
    - _on_notification() and _on_disconnect() are invoked by Bleak on background threads
    - These callbacks use call_soon_threadsafe() to marshal execution into the asyncio event loop
    - All protocol-layer operations (waiter, uploader) run on the main asyncio thread
    - No locks required: thread boundaries are enforced by asyncio's thread-safe scheduling
    """

    def __init__(
            self,
            ctrl_uuid: str,
            data_uuid: str,
            notification_callback: Any,
            disconnected_callback: Any,
            loop: asyncio.AbstractEventLoop,
            logger: Optional[IndentLogger] = None,
    ) -> None:
        self.ctrl_uuid: str = ctrl_uuid
        self.data_uuid: str = data_uuid
        self._notify_cb: Any = notification_callback
        self._disconnected_cb: Any = disconnected_callback
        self._loop: asyncio.AbstractEventLoop = loop
        self._client: Optional[BleakClient] = None
        self._logger: IndentLogger = logger or IndentLogger(logging.getLogger("BLETransport"))

    async def scan_for_device_by_name(self, name: str, timeout: int = 10) -> BLEDevice:
        self._logger.debug("Scanning for device name=%s timeout=%s", name, timeout)
        found_event = asyncio.Event()

        candidate: Optional[BLEDevice] = None

        def detection_cb(device: BLEDevice, adv):
            nonlocal candidate
            if device.name == name and DFU_SERVICE_UUID.lower() in [
                s.lower() for s in (adv.service_uuids or [])
            ]:
                candidate = device
                # schedule on main loop
                self._loop.call_soon_threadsafe(found_event.set)

        async with BleakScanner(detection_callback=detection_cb) as scanner:
            try:
                await asyncio.wait_for(found_event.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                pass

        if not candidate:
            raise DeviceNotFound(f"Device named '{name}' advertising DFU service not found")
        return candidate

    async def _safe_disconnect(self) -> None:
        """Disconnect without raising - for cleanup after connection failures."""
        if not self._client:
            return
        try:
            if self._client.is_connected:
                await asyncio.wait_for(self._client.disconnect(), timeout=0.5)
        except Exception:
            pass  # Silently ignore cleanup errors
        finally:
            self._client = None

    async def connect(self, device: Any, requested_mtu: int = 517) -> None:
        """
        Connect to the device and negotiate MTU.
        `device` can be a BLEDevice or an address string, depending on the platform.
        `requested_mtu` is the MTU to request (default 517 = BLE max).
        """
        self._logger.debug("Connecting to device %s", getattr(device, "address", str(device)))
        self._client = BleakClient(device, disconnected_callback=self._on_disconnect)
        try:
            await self._client.connect()
            # Query negotiated MTU for chunk size calculation
            self._mtu = self._client.mtu_size
            await self._client.start_notify(self.ctrl_uuid, self._on_notification)
            await asyncio.sleep(0.2)
        except BleakCharacteristicNotFoundError as e:
            await self._safe_disconnect()
            device_addr = getattr(device, "address", str(device))
            # Extract short UUID (first 8 chars) for readability
            short_uuid = self.ctrl_uuid.split("-")[0]
            raise ServiceNotFound(
                f"DFU service ({short_uuid}) not found on device {device_addr}. "
                f"Ensure the device is running firmware with OTA support enabled."
            ) from e
        except Exception as e:
            await self._safe_disconnect()
            raise ConnectionFailed(f"Failed to connect: {e}") from e

    async def disconnect(self) -> None:
        if not self._client:
            return
        try:
            if self._client.is_connected:
                try:
                    await asyncio.wait_for(self._client.stop_notify(self.ctrl_uuid), timeout=0.5)
                except (asyncio.TimeoutError, Exception):
                    self._logger.debug("stop_notify failed (ignored)")
                try:
                    await asyncio.wait_for(self._client.disconnect(), timeout=0.5)
                except (asyncio.TimeoutError, Exception):
                    self._logger.debug("disconnect failed (ignored)")
        finally:
            self._client = None

    async def write_control(self, payload: bytes, with_response: bool = True) -> None:
        if not self._client or not self._client.is_connected:
            raise ConnectionFailed("Not connected")
        try:
            await self._client.write_gatt_char(self.ctrl_uuid, payload, response=with_response)
        except BleakError as e:
            # Map to ConnectionLost, which is handled by the protocol layer
            raise ConnectionLost(f"Device disconnected: {e}") from e

    async def write_data(self, data: bytes) -> None:
        if not self._client or not self._client.is_connected:
            raise ConnectionFailed("Not connected")
        try:
            await self._client.write_gatt_char(self.data_uuid, data, response=False)
        except BleakError as e:
            raise ConnectionLost(f"Device disconnected: {e}") from e

    # called by Bleak on notification thread; adapt into loop
    def _on_notification(self, sender: Any, data: bytes) -> None:
        # Schedule the protocol notification callback into the uploader loop
        try:
            self._loop.call_soon_threadsafe(self._notify_cb, sender, data)
        except Exception:
            # If the loop is closed or other problems, log error—higher layers will get disconnected
            self._logger.error("Failed scheduling notification callback", exc_info=True)

    # called by Bleak in a thread; call into event loop safely
    def _on_disconnect(self, client: Any) -> None:
        try:
            self._loop.call_soon_threadsafe(self._disconnected_cb, client)
        except Exception:
            # If the loop is closed or other problems, log error—higher layers will handle cleanup
            self._logger.error("Failed scheduling disconnected callback", exc_info=True)


# ----------------------------
# ControlWaiter & notification parsing (protocol layer)
# ----------------------------
class ControlWaiter:
    """
    Handles mapping request opcode -> asyncio.Future for responses and PRN handling.

    notification_handler is meant to be called on the asyncio loop (we ensure thread-safety
    by scheduling BLE notifications through BLETransport._on_notification).
    """

    def __init__(self, logger: IndentLogger):
        self._logger = logger
        self._resp_futures: Dict[int, asyncio.Future] = {}
        self._prn_future: Optional[asyncio.Future] = None

    @staticmethod
    def _parse_response_packet(data: bytes) -> Optional[Tuple[int, int, bytes]]:
        if not data or len(data) < 3:
            return None
        if data[0] != OP_RESPONSE:
            return None
        req_opcode = data[1]
        status = data[2]
        payload = data[3:] if len(data) > 3 else b""
        return req_opcode, status, payload

    def notification_handler(self, sender: Any, data: bytes) -> None:
        """This runs on the asyncio loop (scheduled by BLETransport)."""
        # Log raw notification
        self._logger.debug("RAW notification len=%d data=%s", len(data), data.hex())

        parsed = self._parse_response_packet(data)
        if parsed:
            req_opcode, status, payload = parsed
            self._logger.debug("Parsed response opcode=0x%02X status=0x%02X payload_len=%d", req_opcode, status, len(payload))
            fut = self._resp_futures.pop(req_opcode, None)
            if fut and not fut.done():
                fut.set_result((status, payload))
                return

            # If CALCULATE_CRC arrives and no waiter, treat as PRN candidate (offset+crc)
            if req_opcode == OP_CALCULATE_CRC and self._prn_future and not self._prn_future.done():
                if len(payload) >= 8:
                    offset = int.from_bytes(payload[0:4], "little")
                    crc = int.from_bytes(payload[4:8], "little")
                    self._prn_future.set_result((offset, crc))
                    return

            self._logger.warning("No future for opcode 0x%02X", req_opcode)
            return

        # Maybe it's raw 8-byte PRN (offset + CRC)
        if len(data) == 8 and self._prn_future and not self._prn_future.done():
            offset = int.from_bytes(data[0:4], "little")
            crc = int.from_bytes(data[4:8], "little")
            self._logger.debug("OOB PRN notification offset=%d crc=0x%08X", offset, crc)
            self._prn_future.set_result((offset, crc))
            return

        self._logger.debug("Unknown notification format (ignored)")

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

    def on_disconnect(self) -> None:
        """Fail all pending futures on disconnect. This must be called on the loop."""
        exc = ConnectionLost("Device disconnected unexpectedly")
        for fut in list(self._resp_futures.values()):
            if not fut.done():
                fut.set_exception(exc)
        self._resp_futures.clear()
        if self._prn_future and not self._prn_future.done():
            self._prn_future.set_exception(exc)
        self._prn_future = None


# ----------------------------
# DFUCommand pattern (protocol)
# ----------------------------
@dataclass
class DFUCommand:
    opcode: int
    name: str
    timeout: float = 5.0

    def on_error(self, error: Exception) -> bool:
        return False

    async def post_execute(self, uploader: "DFUUploader") -> None:
        return None

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        payload = self.build()
        uploader.log.debug("→ Sending %s", self.name)
        uploader.log.indent()
        waiter_task = None
        try:
            waiter_task = asyncio.create_task(uploader.waiter.wait_response(self.opcode, timeout=self.timeout))
            await uploader.transport.write_control(payload, with_response=True)
            status, resp_payload = await waiter_task
            if status != DFU_STATUS_SUCCESS:
                raise DFUError(f"{self.name} failed with status 0x{status:02x}")
            result = self.parse_response(resp_payload)
            uploader.log.debug("Received %s response (status=0x%02X)", self.name, status)
            await self.post_execute(uploader)
            return result
        except asyncio.CancelledError:
            # propagate cancellation upstream
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
            uploader.log.dedent()

    def build(self) -> bytes:
        raise NotImplementedError

    def parse_response(self, payload: bytes) -> Optional[Any]:
        return None


class SetPrn(DFUCommand):
    def __init__(self, prn: int = DEFAULT_PRN):
        super().__init__(OP_SET_RECEIPT, "SET_PRN")
        self.prn = prn

    def build(self) -> bytes:
        return bytes([self.opcode]) + int(self.prn).to_bytes(2, "little")


class SelectObject(DFUCommand):
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


class CreateObject(DFUCommand):
    def __init__(self, obj_type: int = DFU_OBJ_DATA, size: int = 0):
        super().__init__(OP_CREATE_OBJECT, "CREATE_OBJECT", timeout=30.0)
        self.obj_type = obj_type
        self.size = size

    def build(self) -> bytes:
        return bytes([self.opcode, self.obj_type]) + int(self.size).to_bytes(4, "little")

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        # Keep user informed (prints are preserved for backward compatibility)
        print(f"Erasing flash ({self.size:,} bytes)...", flush=True)
        return await super().execute(uploader)


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
        raise DFUError("CALCULATE_CRC returned unexpected payload")


class ExecuteObject(DFUCommand):
    def __init__(self):
        super().__init__(OP_EXECUTE_OBJECT, "EXECUTE_OBJECT", timeout=10.0)

    def build(self) -> bytes:
        return bytes([self.opcode])

    def on_error(self, error: Exception) -> bool:
        # Device may disconnect during execution (reboot) — treat as acceptable
        if isinstance(error, (asyncio.TimeoutError, BleakError, ConnectionLost)):
            return True
        return False

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        print("Activating firmware...", flush=True)
        result = await super().execute(uploader)
        return result

    async def post_execute(self, uploader: "DFUUploader") -> None:
        # Require device to disconnect (reboot)
        print("✓ Firmware activated successfully, waiting for device to reboot...", flush=True)
        uploader.log.debug("Waiting for device reboot (disconnect required)...")
        try:
            await asyncio.wait_for(uploader.disconnect_event.wait(), timeout=CONNECTION_REBOOT_TIMEOUT_S)
            uploader.log.debug("Device rebooted (disconnect observed)")
            print("✓ Device rebooted successfully", flush=True)
        except asyncio.TimeoutError:
            raise DFUError(f"{self.name}: Device did NOT reboot after execute - FAILURE!")


class UploadData(DFUCommand):
    def __init__(self, firmware: bytes, resume_offset: int = 0):
        super().__init__(opcode=0, name="UploadData")
        self.firmware = firmware
        self.resume_offset = resume_offset
        self.total = len(firmware)

    def build(self) -> bytes:
        raise NotImplementedError("UploadData doesn't use build()")

    def parse_response(self, payload: bytes) -> Any:
        raise NotImplementedError("UploadData doesn't parse responses")

    async def execute(self, uploader: "DFUUploader") -> Optional[Any]:
        uploader.log.debug("→ Starting firmware upload...")
        uploader.log.indent()
        try:
            offset = self.resume_offset
            # Don't carry over packet count on resume - device counter resets on boot
            packets = 0

            prn_future = uploader.waiter.register_prn()

            import time
            batch_start = time.perf_counter()

            while offset < self.total:
                chunk = self.firmware[offset : offset + uploader.chunk_size]
                await uploader.transport.write_data(chunk)
                offset += len(chunk)
                packets += 1

                # Soft yield to avoid BLE buffer overload
                if packets % 16 == 0:
                    await asyncio.sleep(0.01)

                if uploader.prn > 0 and (packets % uploader.prn) == 0:
                    send_time = time.perf_counter() - batch_start
                    t0 = time.perf_counter()
                    prn_off, prn_crc = await uploader.waiter.wait_prn(prn_future, timeout=12.0)
                    t1 = time.perf_counter()
                    # zlib.crc32 uses different polynomial than ESP32's esp_crc32_le
                    # local_crc = zlib.crc32(self.firmware[:prn_off]) & 0xFFFFFFFF
                    local_crc = esp_crc32(self.firmware[:prn_off])
                    t2 = time.perf_counter()
                    uploader.log.debug("PRN timing: send=%.3fs wait=%.3fs crc=%.3fs", send_time, t1-t0, t2-t1)
                    batch_start = time.perf_counter()  # Reset for next batch
                    if local_crc != prn_crc:
                        raise VerificationFailed(
                            f"PRN CRC mismatch at {prn_off}: device=0x{prn_crc:08X} local=0x{local_crc:08X}"
                        )
                    pct = (prn_off * 100) // self.total
                    print(f"  CRC ok @ {prn_off:,} bytes ({pct}%)", flush=True)
                    prn_future = uploader.waiter.register_prn()

            uploader.log.debug("✓ Firmware upload complete")
            return None
        finally:
            uploader.log.dedent()


# ----------------------------
# DFU Uploader (orchestration)
# ----------------------------
class DFUUploader:
    def __init__(
            self,
            firmware_path: Path,
            transport: BLETransport,
            prn: int = DEFAULT_PRN,
            chunk_size: int = DEFAULT_CHUNK,
            logger_obj: Optional[logging.Logger] = None,
    ):
        self.firmware_path = Path(firmware_path)
        self.transport = transport
        self.prn = prn
        self.chunk_size = chunk_size
        self.log = IndentLogger(logger_obj or logging.getLogger(self.__class__.__name__))
        self.waiter = ControlWaiter(self.log)
        self.disconnect_event = asyncio.Event()

    async def set_prn(self) -> None:
        await SetPrn(self.prn).execute(self)

    async def select_object(self, obj_type: int) -> Optional[Any]:
        return await SelectObject(obj_type).execute(self)

    async def create_object(self, obj_type: int, size: int) -> None:
        await CreateObject(obj_type, size).execute(self)

    async def calculate_crc(self) -> Optional[Any]:
        return await CalculateCRC().execute(self)

    async def execute_object(self) -> None:
        await ExecuteObject().execute(self)

    # Called by transport on disconnection (scheduled into loop)
    def _on_disconnect(self, client: Any) -> None:
        # Fail waiter and set disconnect_event — must be called on loop
        self.log.debug("Transport reported disconnect")
        self.waiter.on_disconnect()
        self.disconnect_event.set()

    async def upload(self, device: Any, resume_scan: bool = False) -> None:
        # Validate firmware file
        if not self.firmware_path.exists() or not self.firmware_path.is_file():
            raise DFUError(f"Firmware not found: {self.firmware_path}")
        firmware = self.firmware_path.read_bytes()
        total = len(firmware)
        if total == 0:
            raise DFUError("Firmware file is empty")
        if total > MAX_FIRMWARE_SIZE:
            raise DFUError(f"Firmware too large: {total} (max={MAX_FIRMWARE_SIZE})")

        print(f"Starting BLE DFU upload: {total:,} bytes, CRC32=0x{zlib.crc32(firmware)&0xFFFFFFFF:08X}", flush=True)

        # Connect (transport.connect handles BLEClient creation)
        last_exc: Optional[Exception] = None
        for attempt in range(1, MAX_CONNECTION_RETRIES + 1):
            try:
                print(f"Connecting to {getattr(device, 'name', str(device))}...", flush=True)
                await self.transport.connect(device)
                # Auto-adjust chunk size based on negotiated MTU (MTU - 3 for ATT header)
                max_chunk = self.transport._mtu - 3
                if self.chunk_size > max_chunk:
                    print(f"✓ Connected (MTU={self.transport._mtu}, chunk={self.chunk_size}→{max_chunk})", flush=True)
                    self.chunk_size = max_chunk
                else:
                    print(f"✓ Connected (MTU={self.transport._mtu}, chunk={self.chunk_size})", flush=True)
                break
            except ServiceNotFound:
                # Non-retryable: DFU service not present on device
                raise
            except Exception as exc:
                last_exc = exc
                print(f"WARNING: Connect attempt {attempt} failed: {exc}", flush=True)
                # Cleanup & backoff
                try:
                    await self.transport.disconnect()
                except Exception:
                    pass
                if attempt < MAX_CONNECTION_RETRIES:
                    await asyncio.sleep(2 ** (attempt - 1))
                else:
                    raise ConnectionFailed(f"Failed to connect after {MAX_CONNECTION_RETRIES} attempts: {last_exc}")

        try:
            # Set PRN
            await self.set_prn()

            # Object type
            obj_type = DFU_OBJ_DATA

            # Try resume
            dev_offset, dev_crc, _ = await self.select_object(obj_type)
            resume_offset = 0
            if dev_offset > 0:
                # zlib.crc32 uses different polynomial than ESP32's esp_crc32_le
                # local_crc = zlib.crc32(firmware[:dev_offset]) & 0xFFFFFFFF
                local_crc = esp_crc32(firmware[:dev_offset])
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

            # Create a new object if starting fresh
            if resume_offset == 0:
                await CreateObject(obj_type, total).execute(self)

            # Upload firmware data
            await UploadData(firmware, resume_offset).execute(self)

            # Final CRC verify
            calc_off, calc_crc = await CalculateCRC().execute(self)
            # zlib.crc32 uses different polynomial than ESP32's esp_crc32_le
            # local_crc = zlib.crc32(firmware[:calc_off]) & 0xFFFFFFFF
            local_crc = esp_crc32(firmware[:calc_off])
            if calc_off != total or calc_crc != local_crc:
                raise VerificationFailed(
                    f"Verification failed: device_off={calc_off} device_crc=0x{calc_crc:08X} local_crc=0x{local_crc:08X}"
                )

            print(f"✓ Upload complete: {calc_off:,} bytes, CRC=0x{calc_crc:08X}", flush=True)

            # Execute (requires reboot/disconnect)
            await ExecuteObject().execute(self)
            print("✓ Firmware successfully activated — device rebooted", flush=True)

        finally:
            try:
                await self.transport.disconnect()
            except Exception:
                self.log.debug("Transport disconnect failed during cleanup", exc_info=True)


# ----------------------------
# CLI & orchestration glue
# ----------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ESP32-S3 BLE DFU uploader (refactor)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s firmware.bin -d E20E664A-4716-ABA3-ABC6-B9A0329B5B2E
  %(prog)s firmware.bin -n "Device Name"
  export BLE_DEVICE_ADDRESS=A4:CF:12:XX:XX:XX && %(prog)s firmware.bin
"""
    )
    p.add_argument("firmware", type=Path, help="Path to firmware .bin")

    # Device specification: require either -d, -n, or BLE_DEVICE_ADDRESS env var
    device_group = p.add_mutually_exclusive_group(required=not os.environ.get("BLE_DEVICE_ADDRESS"))
    device_group.add_argument("-d", "--device", dest="device", help="BLE device address")
    device_group.add_argument("-n", "--name", dest="name", help="BLE device name (scan)")

    p.add_argument("-p", "--prn", type=int, default=DEFAULT_PRN, help="Packet receipt notification interval")
    p.add_argument("--chunk", type=int, default=DEFAULT_CHUNK, help="BLE packet chunk size")
    p.add_argument("-v", "--verbose", action="store_true", help="Verbose (console output)")
    p.add_argument("--log-level", choices=["debug", "info", "warning", "error"], default="warning", help="Logging level")
    return p


async def run_upload_with_args(args: argparse.Namespace) -> int:
    # Logging config
    level = {"debug": logging.DEBUG, "info": logging.INFO, "warning": logging.WARNING, "error": logging.ERROR}[args.log_level]
    # Configure root logger to WARNING to suppress third-party library logs
    fmt = "%(asctime)s %(levelname)s [%(name)s] %(message)s" if level <= logging.DEBUG else "%(levelname)s: %(message)s"
    logging.basicConfig(level=logging.WARNING, format=fmt, stream=sys.stdout)
    # Only set our loggers to the user-specified level
    uploader_logger = logging.getLogger("DFUUploader")
    uploader_logger.setLevel(level)
    logging.getLogger("dfu_cli").setLevel(level)

    device_address = (args.device or os.environ.get("BLE_DEVICE_ADDRESS") or "").strip() or None
    device_name = (args.name or "").strip() or None

    if not device_address and not device_name:
        logger.error("Device required (address or name).")
        return EXIT_USAGE_ERROR

    if device_address and device_name:
        logger.error("Specify either device address (-d) OR name (-n), not both.")
        return EXIT_USAGE_ERROR

    if not args.firmware.exists():
        logger.error("Firmware file not found: %s", args.firmware)
        return EXIT_USAGE_ERROR

    # Validate numeric ranges
    if args.chunk <= 0 or args.chunk > 4096:
        logger.error("--chunk must be between 1 and 4096")
        return EXIT_USAGE_ERROR
    if args.prn <= 0 or args.prn > 65535:
        logger.error("--prn must be > 0 and reasonably small")
        return EXIT_USAGE_ERROR

    loop = asyncio.get_running_loop()

    # Circular dependency workaround: BLETransport requires callbacks at construction,
    # but the real callbacks (uploader.waiter.notification_handler and uploader._on_disconnect)
    # only exists after DFUUploader is instantiated. Pattern:
    # 1. Create transport with dummy no-op callbacks
    # 2. Create uploader (which creates the waiter with a real notification handler)
    # 3. Replace transport callbacks with real ones (lines 812-813)
    # This avoids complex factory patterns while maintaining clean separation of concerns.
    temp_log = IndentLogger(uploader_logger)

    def dummy_notify(sender: Any, data: bytes) -> None:
        """ Placeholder notification callback, replaced after uploader creation."""
        pass

    def dummy_disconnect(client: Any) -> None:
        """ Placeholder disconnect callback, replaced after uploader creation."""
        pass

    transport = BLETransport(
        ctrl_uuid=DFU_CTRL_UUID,
        data_uuid=DFU_DATA_UUID,
        notification_callback=dummy_notify,
        disconnected_callback=dummy_disconnect,
        loop=loop,
        logger=temp_log,
    )

    # Build uploader (it will own the ControlWaiter)
    uploader = DFUUploader(
        firmware_path=args.firmware,
        transport=transport,
        prn=args.prn,
        chunk_size=args.chunk,
        logger_obj=uploader_logger,
    )

    # Replace transport callback bindings to route into uploader.waiter/uploader._on_disconnect
    transport._notify_cb = uploader.waiter.notification_handler
    transport._disconnected_cb = lambda client: uploader._on_disconnect(client)

    # Resolve device handle: either address string or BLEDevice via scan
    device: Any
    if device_name:
        device = await transport.scan_for_device_by_name(device_name, timeout=10)
    else:
        # Format UUID address for Bleak
        cleaned = device_address.replace(":", "").replace("-", "")
        if len(cleaned) == 32:
            # Format as UUID: E20E664A-4716-ABA3-ABC6-B9A0329B5B2E
            device = f"{cleaned[0:8]}-{cleaned[8:12]}-{cleaned[12:16]}-{cleaned[16:20]}-{cleaned[20:32]}".upper()
        else:
            device = device_address

    try:
        await uploader.upload(device=device)
        return EXIT_SUCCESS
    except DFUError as e:
        print(f"ERROR: {e}", file=sys.stderr, flush=True)
        return e.exit_code
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("ERROR: Interrupted by user", file=sys.stderr, flush=True)
        return EXIT_INTERRUPTED
    except Exception as e:
        print(f"ERROR: Unexpected error during DFU: {e}", file=sys.stderr, flush=True)
        logger.exception("Full traceback:")
        return EXIT_GENERIC_ERROR


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    parser = build_arg_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        raise

    # Configure root logging early based on args (use warning default until run_upload config)
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Run upload in asyncio and map exceptions to exit codes
    try:
        return asyncio.run(run_upload_with_args(args))
    except KeyboardInterrupt:
        logger.error("Interrupted by user")
        return EXIT_INTERRUPTED
    except Exception as e:
        logger.exception("Unhandled exception in main: %s", e)
        return EXIT_GENERIC_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
