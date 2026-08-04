#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "MadgwickAHRS.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Node ID (set to 1 for Node A, 2 for Node B, 3 for main Node)
#define NODE_ID 3

// How many remote nodes must report in before we forward a master packet.
// Set to 1 for bench testing with a single node; 2 for the full vest.
#define REQUIRED_NODES 1

static const uint8_t MUX_ADDR = 0x70;
static const uint8_t ICM_ADDR = 0x69; // change to 0x68 if not working

static const float FILTER_HZ = 50.0f;
static const uint32_t LOOP_DELAY_MS = 20;

static const uint8_t NUM_LOCAL_IMUS = 2;
static const uint8_t IMU_CHANNELS[NUM_LOCAL_IMUS] = {0, 1};

static const uint8_t NUM_REMOTE_IMUS = 4;
static const int WIFI_CHANNEL = 6;
static const char* blName = "ExoVest";

// BLE service/characteristic UUIDs (Nordic UART Service; TX = notify to Python)
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

ICM_20948_I2C imu;
Madgwick filters[NUM_LOCAL_IMUS];

BLECharacteristic* pTxCharacteristic = nullptr;
volatile bool bleConnected = false;

// Track BLE connect/disconnect so we only notify when Python is listening
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("BLE client disconnected; re-advertising");
    pServer->startAdvertising();
  }
};

#pragma pack(push,1)
struct QuatPacket {
  uint8_t node_id;
  uint32_t seq;
  uint32_t t_ms;
  int16_t quat_q14[NUM_REMOTE_IMUS][4];
};
#pragma pack(pop)

#pragma pack(push,1)
struct LocalQuatPacket {
  uint8_t node_id;
  uint32_t seq;
  uint32_t t_ms;
  int16_t quat_q14[NUM_LOCAL_IMUS][4];
};
#pragma pack(pop)

#pragma pack(push,1)
struct MasterPacket {
  uint32_t master_seq;
  uint32_t master_t_ms;

  uint32_t nodeA_t_ms;
  uint32_t nodeB_t_ms;
  uint32_t local_t_ms;

  int16_t nodeA[4][4];
  int16_t nodeB[4][4];
  int16_t local[2][4];
};
#pragma pack(pop)


QuatPacket nodeAData;
QuatPacket nodeBData;
LocalQuatPacket localData;
MasterPacket masterPkt;

bool nodeAReceived = false;
bool nodeBReceived = false;
uint32_t seqCounter = 0;
uint32_t masterSeqCounter = 0;

// Select the multiplexer channel
void muxSelect(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Activate/wake up IMU on selected channel
bool initImuOnChannel(uint8_t channel) {
  muxSelect(channel);
  delay(10);
  auto stat = imu.begin(Wire, ICM_ADDR);
  return (stat == ICM_20948_Stat_Ok);
}

// Convert float quaternion component to Q14 fixed-point int16
static int16_t toQ14(float x) {
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  return (int16_t)lroundf(x * 16384.0f);
}

// Build a unit quaternion from the filter's Euler angles (radians).
// Lets us read orientation from Madgwick without touching its private members.
static void eulerToQuat(float roll, float pitch, float yaw,
                        float& qw, float& qx, float& qy, float& qz) {
  float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

// ESP-NOW receiving code
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info;

  if (len != sizeof(QuatPacket)) {
    Serial.printf("Received unexpected packet size: %d\n", len);
    return;
  }

  QuatPacket incomingPacket;
  memcpy(&incomingPacket, incomingData, sizeof(incomingPacket));

  if (incomingPacket.node_id == 1) {
    nodeAData = incomingPacket;
    nodeAReceived = true;
    Serial.println("Received packet from Node A");
  } 
  else if (incomingPacket.node_id == 2) {
    nodeBData = incomingPacket;
    nodeBReceived = true;
    Serial.println("Received packet from Node B");
  } 
  else {
    Serial.printf("Unknown node_id received: %u\n", incomingPacket.node_id);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  Wire.setClock(400000);

  // Get all filters ready
  for (uint8_t i = 0; i < NUM_LOCAL_IMUS; i++) {
    filters[i].begin(FILTER_HZ);
  }

  Serial.print("Mux addr: 0x");
  Serial.print(MUX_ADDR, HEX);
  Serial.print(" | IMU addr: 0x");
  Serial.println(ICM_ADDR, HEX);

  // Initialize IMUs
  for (uint8_t i = 0; i < NUM_LOCAL_IMUS; i++) {
    uint8_t ch = IMU_CHANNELS[i];
    bool ok = initImuOnChannel(ch);
    Serial.printf("Channel %u init: %s\n", ch, ok ? "OK" : "FAIL");
    delay(50);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("Main MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Main microcontroller ready.");

  // BLE init (Bluetooth Classic / BluetoothSerial isn't supported on ESP32-S3)
  BLEDevice::init(blName);
  BLEDevice::setMTU(247); // MasterPacket is ~100 B; notify needs MTU > packet + 3
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
      TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("BLE advertising as \"%s\"\n", blName);
}

void loop() {
  localData.node_id = NODE_ID;
  localData.seq = seqCounter++;
  localData.t_ms = millis();

  for (uint8_t i = 0; i < NUM_LOCAL_IMUS; i++) {
    uint8_t ch = IMU_CHANNELS[i];
    muxSelect(ch);

    imu.getAGMT(); // reads sensors; result/status is stored on the imu object
    if (imu.status == ICM_20948_Stat_Ok) {
      float ax = imu.accX(), ay = imu.accY(), az = imu.accZ();
      float gx = imu.gyrX(), gy = imu.gyrY(), gz = imu.gyrZ(); // deg/s
      float mx = imu.magX(), my = imu.magY(), mz = imu.magZ();

      // Madgwick update() expects gyro in rad/s, but the ICM reports deg/s.
      gx *= DEG_TO_RAD; gy *= DEG_TO_RAD; gz *= DEG_TO_RAD;

      filters[i].update(gx, gy, gz, ax, ay, az, mx, my, mz);

      // Pull orientation as a quaternion (via the public Euler getters)
      float fqw, fqx, fqy, fqz;
      eulerToQuat(filters[i].getRollRadians(),
                  filters[i].getPitchRadians(),
                  filters[i].getYawRadians(),
                  fqw, fqx, fqy, fqz);

      int16_t qw = toQ14(fqw);
      int16_t qx = toQ14(fqx);
      int16_t qy = toQ14(fqy);
      int16_t qz = toQ14(fqz);

      localData.quat_q14[i][0] = qw;
      localData.quat_q14[i][1] = qx;
      localData.quat_q14[i][2] = qy;
      localData.quat_q14[i][3] = qz;

      Serial.printf("PACKED CH%u -> Q14 [w=%d x=%d y=%d z=%d] SUCCESS\n", ch, qw, qx, qy, qz);
    } else {
      localData.quat_q14[i][0] = 0;
      localData.quat_q14[i][1] = 0;
      localData.quat_q14[i][2] = 0;
      localData.quat_q14[i][3] = 0;

      Serial.printf("CH%u | read FAIL\n", ch);
    }
  }

  if (nodeAReceived) {
    Serial.printf("Node A seq=%lu\n", (unsigned long)nodeAData.seq);
    for (uint8_t i = 0; i < NUM_REMOTE_IMUS; i++) {
      Serial.printf("  A IMU %u -> [%d, %d, %d, %d]\n",
        i,
        nodeAData.quat_q14[i][0],
        nodeAData.quat_q14[i][1],
        nodeAData.quat_q14[i][2],
        nodeAData.quat_q14[i][3]
      );
    }
  }

  if (nodeBReceived) {
    Serial.printf("Node B seq=%lu\n", (unsigned long)nodeBData.seq);
    for (uint8_t i = 0; i < NUM_REMOTE_IMUS; i++) {
      Serial.printf("  B IMU %u -> [%d, %d, %d, %d]\n",
        i,
        nodeBData.quat_q14[i][0],
        nodeBData.quat_q14[i][1],
        nodeBData.quat_q14[i][2],
        nodeBData.quat_q14[i][3]
      );
    }
  }
  //Create master packet to send everything at once
  // Only build + send master packet once enough nodes have sent data.
  // REQUIRED_NODES controls how many (1 for single-node bench testing, 2 for full vest).
  uint8_t nodesReady = (nodeAReceived ? 1 : 0) + (nodeBReceived ? 1 : 0);
if (nodesReady >= REQUIRED_NODES) {

  // Header
  masterPkt.master_seq = masterSeqCounter++;
  masterPkt.master_t_ms = millis();

  masterPkt.nodeA_t_ms = nodeAData.t_ms;
  masterPkt.nodeB_t_ms = nodeBData.t_ms;
  masterPkt.local_t_ms = localData.t_ms;

  // Copy Node A
  for (uint8_t i = 0; i < NUM_REMOTE_IMUS; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      masterPkt.nodeA[i][j] = nodeAData.quat_q14[i][j];
    }
  }

  // Copy Node B
  for (uint8_t i = 0; i < NUM_REMOTE_IMUS; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      masterPkt.nodeB[i][j] = nodeBData.quat_q14[i][j];
    }
  }

  // Copy Local IMUs
  for (uint8_t i = 0; i < NUM_LOCAL_IMUS; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      masterPkt.local[i][j] = localData.quat_q14[i][j];
    }
  }

  // Send to Python over BLE (notify) — only if a client is subscribed
  if (bleConnected) {
    pTxCharacteristic->setValue((uint8_t*)&masterPkt, sizeof(masterPkt));
    pTxCharacteristic->notify();
    Serial.println("MASTER PACKET SENT TO PYTHON (BLE)");
  } else {
    Serial.println("No BLE client connected; master packet not sent");
  }
}
else {
  Serial.printf("Waiting for nodes... %u/%u ready\n", nodesReady, REQUIRED_NODES);
}

  Serial.println("---");
  delay(LOOP_DELAY_MS);
}