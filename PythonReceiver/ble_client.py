"""BLE transport for the ExoVest receiver.

Scans for the Main microcontroller (advertising as "ExoVest"), connects, and
subscribes to the Nordic UART TX characteristic. Each BLE notification carries
one 100-byte MasterPacket, which is handed to a user-supplied callback as raw
bytes.

This module only moves bytes. Decoding the packet lives in packet.py (Phase 2);
auto-reconnect and recording come in Phase 4.
"""
from __future__ import annotations

import asyncio
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

# Must match the firmware in MainMCcode.ino
DEVICE_NAME = "ExoVest"
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: MC -> Python

EXPECTED_PACKET_LEN = 100

# Callback invoked with the raw bytes of each notification.
PacketCallback = Callable[[bytes], None]


async def find_device(timeout: float = 10.0) -> Optional[BLEDevice]:
    """Scan for the ExoVest by advertised name.

    Returns the BLEDevice, or None if it wasn't seen within `timeout` seconds.
    On failure, prints the other devices that *were* seen as a debugging aid.
    """
    print(f'Scanning for "{DEVICE_NAME}" (up to {timeout:.0f}s)...')
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=timeout)
    if device is not None:
        return device

    # Not found — show what was in range so the user can sanity-check.
    print(f'Could not find "{DEVICE_NAME}". Devices seen during scan:')
    seen = await BleakScanner.discover(timeout=3.0)
    if not seen:
        print("  (none — is Bluetooth on? On macOS, grant the terminal "
              "Bluetooth permission in System Settings > Privacy & Security.)")
    for d in seen:
        print(f"  {d.address}  {d.name or '<no name>'}")
    return None


async def stream(
    on_packet: PacketCallback,
    *,
    scan_timeout: float = 10.0,
    stop_event: Optional[asyncio.Event] = None,
) -> None:
    """Connect to the ExoVest and forward every notification to `on_packet`.

    Runs until the device disconnects or `stop_event` is set. Raises
    RuntimeError if the device can't be found.
    """
    device = await find_device(scan_timeout)
    if device is None:
        raise RuntimeError(f'"{DEVICE_NAME}" not found')

    disconnected = asyncio.Event()

    def _on_disconnect(_client: BleakClient) -> None:
        print("Device disconnected.")
        disconnected.set()

    print(f"Connecting to {device.address} ...")
    async with BleakClient(device, disconnected_callback=_on_disconnect) as client:
        print(f"Connected. Subscribing to TX characteristic {NUS_TX_CHAR_UUID}")

        def _handler(_char, data: bytearray) -> None:
            on_packet(bytes(data))

        await client.start_notify(NUS_TX_CHAR_UUID, _handler)
        print("Subscribed. Waiting for packets (Ctrl+C to stop)...\n")

        # Idle until the link drops or the caller asks us to stop.
        waiters = [asyncio.create_task(disconnected.wait())]
        if stop_event is not None:
            waiters.append(asyncio.create_task(stop_event.wait()))
        done, pending = await asyncio.wait(
            waiters, return_when=asyncio.FIRST_COMPLETED
        )
        for task in pending:
            task.cancel()

        # Best-effort clean unsubscribe if we're still connected.
        if client.is_connected:
            try:
                await client.stop_notify(NUS_TX_CHAR_UUID)
            except Exception:
                pass
