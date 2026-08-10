"""ExoVest Python receiver — entry point.

Connects to the ExoVest over BLE and displays incoming MasterPackets.

Usage:
    python main.py            # link monitor: receive count, rate, length
    python main.py --hex      # also print a hex dump of each packet
    python main.py --dump     # decode and show live quaternion values
    python main.py --scan 15  # scan timeout in seconds (default 10)

A later phase adds the 3D viewer (default, no flag).
"""
from __future__ import annotations

import argparse
import asyncio
import signal
import time

import numpy as np

from ble_client import EXPECTED_PACKET_LEN, stream
from packet import IMU_LABELS, SequenceTracker, decode, quat_to_euler_deg


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

        # rx = packets received by *Python* since connect (not the firmware's
        # master_seq; that lives inside the payload — see --dump).
        flag = "OK " if length == EXPECTED_PACKET_LEN else "BAD"
        line = (
            f"rx={self.count:6d}  len={length:3d} {flag} "
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


class DecodeMonitor:
    """Decodes each packet and renders a live dashboard of all 10 IMUs.

    The screen is redrawn at a throttled rate (so 40 Hz of packets doesn't
    flood the terminal), but *every* packet is decoded for drop accounting.
    """

    REFRESH_HZ = 5.0

    def __init__(self) -> None:
        self.seq = SequenceTracker()
        self.bad = 0
        self._start = time.monotonic()
        self._last_draw = 0.0
        self._window_start = self._start
        self._window_count = 0
        self._rate = 0.0
        self._latest = None  # most recent decoded MasterPacket

    def on_packet(self, data: bytes) -> None:
        try:
            pkt = decode(data)
        except ValueError:
            self.bad += 1
            return

        self._latest = pkt
        self.seq.update(pkt.master_seq)

        now = time.monotonic()
        self._window_count += 1
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self._rate = self._window_count / elapsed
            self._window_start = now
            self._window_count = 0

        if now - self._last_draw >= 1.0 / self.REFRESH_HZ:
            self._draw()
            self._last_draw = now

    def _draw(self) -> None:
        pkt = self._latest
        if pkt is None:
            return
        # Clear screen + home cursor for a stable dashboard.
        lines = ["\033[2J\033[H"]
        lines.append(
            f"ExoVest  master_seq={pkt.master_seq}  rate={self._rate:4.1f} Hz  "
            f"rx={self.seq.received}  dropped={self.seq.dropped} "
            f"({self.seq.loss_pct:.1f}%)  resets={self.seq.resets}  bad_len={self.bad}"
        )
        lines.append(
            f"t_ms  master={pkt.master_t_ms}  nodeA={pkt.nodeA_t_ms}  "
            f"nodeB={pkt.nodeB_t_ms}  local={pkt.local_t_ms}"
        )
        lines.append("")
        lines.append(
            f"{'IMU':<5}{'roll°':>9}{'pitch°':>9}{'yaw°':>9}{'|q|':>7}   status"
        )
        lines.append("-" * 55)
        for label, q, present in pkt.iter_imus():
            if present:
                roll, pitch, yaw = quat_to_euler_deg(q)
                norm = float(np.linalg.norm(q))
                lines.append(
                    f"{label:<5}{roll:9.1f}{pitch:9.1f}{yaw:9.1f}{norm:7.3f}   live"
                )
            else:
                lines.append(
                    f"{label:<5}{'—':>9}{'—':>9}{'—':>9}{'—':>7}   no data"
                )
        lines.append("")
        lines.append("|q| should stay ~1.00 for live IMUs.  (Ctrl+C to stop)")
        print("\n".join(lines), flush=True)

    def summary(self) -> None:
        total = time.monotonic() - self._start
        print(
            f"\nStopped. rx={self.seq.received} in {total:.1f}s, "
            f"dropped={self.seq.dropped} ({self.seq.loss_pct:.1f}%), "
            f"resets={self.seq.resets}, bad_len={self.bad}."
        )


async def _run(args: argparse.Namespace) -> None:
    monitor = DecodeMonitor() if args.dump else RawMonitor(hexdump=args.hex)
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
    parser = argparse.ArgumentParser(description="ExoVest BLE receiver")
    parser.add_argument("--dump", action="store_true", help="decode packets and show live quaternion values")
    parser.add_argument("--hex", action="store_true", help="print a hex dump of every packet (raw mode)")
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
