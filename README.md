# ExoVest — Motion-Tracking Vest

Wearable motion-capture system that reads body orientation from an array of IMUs and streams it to a PC for real-time motion replication.

Two **node** microcontrollers each track a limb with 4 IMUs and broadcast their orientation over ESP-NOW. A **main** microcontroller collects both nodes' data plus its own IMUs, merges everything into one packet, and forwards it to a Python receiver over BLE.

```
[Node A · 4 IMUs]  ──ESP-NOW─┐
                             ├──►  [Main MC · 2 IMUs]  ──BLE──►  [Python receiver]  ──►  motion replication
[Node B · 4 IMUs]  ──ESP-NOW─┘
```

## Hardware

- **3× ESP32-S3** dev boards (2 nodes + 1 main)
- **ICM-20948** 9-DoF IMUs, addressed through a **TCA9548A** I2C multiplexer (nodes: 4 IMUs, main: 2)
- I2C mux at `0x70`, IMUs at `0x69` (fall back to `0x68`)

## Repository layout

| Folder | Runs on | Role |
|--------|---------|------|
| [`NodeAImuDataCode/`](NodeAImuDataCode) | Node A (ESP32-S3) | Reads 4 IMUs, broadcasts quaternions over ESP-NOW (`NODE_ID 1`) |
| [`NodeBImuDataCode/`](NodeBImuDataCode) | Node B (ESP32-S3) | Same as Node A with `NODE_ID 2` |
| [`MainMCcode/`](MainMCcode) | Main MC (ESP32-S3) | Receives both nodes over ESP-NOW, reads its own 2 IMUs, sends a merged packet over BLE |

## How it works

Each IMU is fused with a **Madgwick filter** into an orientation quaternion. Quaternion components are packed as **Q14 fixed-point `int16`** (`value / 16384.0` → float in `[-1, 1]`, order `[w, x, y, z]`) to keep packets small.

- **Nodes → Main:** ESP-NOW broadcast on Wi-Fi channel 6. Each packet carries `node_id`, a sequence counter, a timestamp, and 4 quaternions.
- **Main → PC:** BLE notify using the Nordic UART Service. The main MC merges Node A, Node B, and its local IMUs into a single ~100-byte `MasterPacket` and notifies it to any subscribed client.

### `MasterPacket` layout (`#pragma pack(1)`, little-endian, 100 bytes)

| Field | Type | Notes |
|-------|------|-------|
| `master_seq` | `uint32` | main-loop sequence counter |
| `master_t_ms` | `uint32` | main MC timestamp (ms) |
| `nodeA_t_ms`, `nodeB_t_ms`, `local_t_ms` | `uint32 ×3` | source timestamps |
| `nodeA[4][4]` | `int16` | Node A: 4 IMUs × `[w,x,y,z]` Q14 |
| `nodeB[4][4]` | `int16` | Node B |
| `local[2][4]` | `int16` | main MC's own 2 IMUs |

Python `struct` format: `<5I 16h 16h 8h`.

## Build & flash (Arduino IDE)

**Board:** ESP32-S3 (Arduino-ESP32 core 3.x)

**Libraries:**
- SparkFun 9DoF IMU Breakout - ICM 20948
- Madgwick (Arduino AHRS)

**Main MC only:** set **Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"** — the BLE + Wi-Fi binary won't fit the default partition.

Flash `NodeAImuDataCode` and `NodeBImuDataCode` to the two node boards and `MainMCcode` to the main board.

## Configuration

| Setting | Where | Purpose |
|---------|-------|---------|
| `NODE_ID` | all sketches | 1 = Node A, 2 = Node B, 3 = Main |
| `REQUIRED_NODES` | `MainMCcode` | Nodes that must report before forwarding. `1` for single-node bench testing, `2` for the full vest |
| `WIFI_CHANNEL` | all sketches | ESP-NOW channel — must match across all boards |
| `ICM_ADDR` | all sketches | IMU I2C address (`0x69` / `0x68`) |
| `NUM_IMUS` / `NUM_REMOTE_IMUS` | node / main | Must stay equal — the main MC rejects packets whose size doesn't match |

## Notes

- Testing with fewer physical IMUs is fine — leave `NUM_IMUS = 4`; unread channels pack as zeros and the packet size is unchanged.
- BLE notifications require an MTU larger than the packet; the main MC requests 247. `bleak` negotiates this automatically on most platforms.

## Roadmap

- [ ] Python BLE receiver (`bleak`) that unpacks `MasterPacket` and logs orientation
- [ ] Real-time skeleton visualization / motion replication
- [ ] Dropout handling — flag stale nodes instead of forwarding their last quaternion
