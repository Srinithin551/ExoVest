"""ExoVest Python receiver — entry point.

Phase 1: prove the BLE link. Connects to the ExoVest and prints incoming
notifications so we can confirm 100-byte packets are arriving at ~50 Hz.

Usage:
    python main.py            # summary line: packet count, rate, length
    python main.py --hex      # also print a hex dump of each packet
    python main.py --scan 15  # scan timeout in seconds (default 10)

Later phases add decoding (--dump) and the 3D viewer (default, no flag).
"""
from __future__ import annotations

import argparse
import asyncio
import signal
import time

from ble_client import EXPECTED_PACKET_LEN, stream


class RawMonitor:
    """Prints a running summary of incoming packets, plus optional hex."""

    def __init__(self, hexdump: bool = False) -> None:
        self.hexdump = hexdump
        self.count = 0
        self.bad_len = 0
        self._start = time.monotonic()
        self._window_start = self._start
        self._window_count = 0
        self._rate = 0.0

    def on_packet(self, data: bytes) -> None:
        self.count += 1
        self._window_count += 1

        length = len(data)
        if length != EXPECTED_PACKET_LEN:
            self.bad_len += 1

        # Recompute rate roughly once per second.
        now = time.monotonic()
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self._rate = self._window_count / elapsed
            self._window_start = now
            self._window_count = 0

        flag = "OK " if length == EXPECTED_PACKET_LEN else "BAD"
        line = (
            f"[{self.count:6d}] len={length:3d} {flag} "
            f"rate={self._rate:5.1f} Hz  bad_len={self.bad_len}"
        )
        if self.hexdump:
            line += "\n         " + data.hex(" ")
        # \r keeps the summary on one line when not dumping hex.
        end = "\n" if self.hexdump else "\r"
        print(line, end=end, flush=True)

    def summary(self) -> None:
        total = time.monotonic() - self._start
        avg = self.count / total if total > 0 else 0.0
        print(
            f"\n\nStopped. Received {self.count} packets in {total:.1f}s "
            f"(avg {avg:.1f} Hz), {self.bad_len} with unexpected length."
        )


async def _run(args: argparse.Namespace) -> None:
    monitor = RawMonitor(hexdump=args.hex)
    stop_event = asyncio.Event()

    # Ctrl+C -> ask the stream to stop cleanly.
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass  # add_signal_handler is unavailable on some platforms

    try:
        await stream(
            monitor.on_packet,
            scan_timeout=args.scan,
            stop_event=stop_event,
        )
    finally:
        monitor.summary()


def main() -> None:
    parser = argparse.ArgumentParser(description="ExoVest BLE receiver (Phase 1: raw link check)")
    parser.add_argument("--hex", action="store_true", help="print a hex dump of every packet")
    parser.add_argument("--scan", type=float, default=10.0, help="scan timeout in seconds (default 10)")
    args = parser.parse_args()

    try:
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"\nError: {exc}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
