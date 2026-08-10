"""Decode the ExoVest MasterPacket.

Wire format (from MainMCcode.ino, #pragma pack(1), little-endian, 100 bytes):

    struct format "<5I 16h 16h 8h"
      5x uint32  header: master_seq, master_t_ms, nodeA_t_ms, nodeB_t_ms, local_t_ms
      16x int16  nodeA[4][4]  (remote node id 1) — 4 IMUs x [w,x,y,z], Q14
      16x int16  nodeB[4][4]  (remote node id 2)
      8x  int16  local[2][4]  (main MC's own 2 IMUs)

Each int16 is a Q14 fixed-point quaternion component: float = value / 16384.0.
A slot that reads [0,0,0,0] means "no data" (a missing IMU, or an absent node
in single-node bench testing) — it is NOT a valid rotation (norm 0), and callers
must treat it as such rather than trying to normalize it.
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterator, Tuple

import numpy as np

STRUCT_FORMAT = "<5I 16h 16h 8h"
PACKET_SIZE = struct.calcsize(STRUCT_FORMAT)  # 100
Q14_SCALE = 16384.0

# A quaternion counts as "present" if its norm is close to 1. Missing slots are
# exactly zero (norm 0); real orientations are unit quaternions. 0.5 cleanly
# separates the two.
VALID_NORM_THRESHOLD = 0.5

# Human labels for the 10 IMU slots, in wire order.
IMU_LABELS = ("A0", "A1", "A2", "A3", "B0", "B1", "B2", "B3", "L0", "L1")


@dataclass
class MasterPacket:
    """One decoded 100-byte MasterPacket. Quaternions are floats in [-1, 1]."""

    master_seq: int
    master_t_ms: int
    nodeA_t_ms: int
    nodeB_t_ms: int
    local_t_ms: int
    nodeA: np.ndarray  # shape (4, 4): [imu][w, x, y, z]
    nodeB: np.ndarray  # shape (4, 4)
    local: np.ndarray  # shape (2, 4)

    def quaternions(self) -> np.ndarray:
        """All 10 quaternions stacked in wire order, shape (10, 4)."""
        return np.vstack((self.nodeA, self.nodeB, self.local))

    def iter_imus(self) -> Iterator[Tuple[str, np.ndarray, bool]]:
        """Yield (label, quat[w,x,y,z], is_present) for each of the 10 IMUs."""
        quats = self.quaternions()
        norms = np.linalg.norm(quats, axis=1)
        for label, quat, norm in zip(IMU_LABELS, quats, norms):
            yield label, quat, bool(norm >= VALID_NORM_THRESHOLD)


def quat_to_euler_deg(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert a [w, x, y, z] quaternion to (roll, pitch, yaw) in degrees.

    Standard aerospace ZYX sequence — inverts the firmware's eulerToQuat(), so
    rotating an IMU about one physical axis should move exactly one of these.
    Returns (0, 0, 0) for a zero (missing) quaternion.
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    if (w, x, y, z) == (0.0, 0.0, 0.0, 0.0):
        return 0.0, 0.0, 0.0

    # roll (x-axis)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # pitch (y-axis) — clamp to guard against tiny out-of-range from rounding
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    # yaw (z-axis)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


def decode(data: bytes) -> MasterPacket:
    """Decode raw notification bytes into a MasterPacket.

    Raises ValueError if the payload isn't exactly PACKET_SIZE bytes.
    """
    if len(data) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} bytes, got {len(data)}")

    fields = struct.unpack(STRUCT_FORMAT, data)
    header = fields[:5]
    quats = np.array(fields[5:], dtype=np.float32) / Q14_SCALE  # 40 floats

    return MasterPacket(
        master_seq=header[0],
        master_t_ms=header[1],
        nodeA_t_ms=header[2],
        nodeB_t_ms=header[3],
        local_t_ms=header[4],
        nodeA=quats[0:16].reshape(4, 4),
        nodeB=quats[16:32].reshape(4, 4),
        local=quats[32:40].reshape(2, 4),
    )


class SequenceTracker:
    """Tracks master_seq to count dropped packets.

    master_seq is a uint32 that increments once per forwarded MasterPacket.
    Gaps mean the BLE link dropped/coalesced notifications. A backward jump
    means the Main MC reset (seq restarts at 0), which we treat as a fresh start
    rather than a huge drop count.
    """

    U32 = 1 << 32

    def __init__(self) -> None:
        self.received = 0
        self.dropped = 0
        self.resets = 0
        self.first_seq: int | None = None
        self.last_seq: int | None = None

    def update(self, seq: int) -> None:
        self.received += 1
        if self.last_seq is None:
            self.first_seq = seq
            self.last_seq = seq
            return

        gap = (seq - self.last_seq) % self.U32
        if gap == 0:
            # Duplicate seq — unusual; ignore for drop accounting.
            pass
        elif gap > (self.U32 // 2):
            # Went backwards by a large amount -> device reset.
            self.resets += 1
            self.first_seq = seq
        else:
            self.dropped += gap - 1
        self.last_seq = seq

    @property
    def loss_pct(self) -> float:
        expected = self.received + self.dropped
        return 100.0 * self.dropped / expected if expected else 0.0
