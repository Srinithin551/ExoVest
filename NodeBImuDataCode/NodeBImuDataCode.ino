#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "MadgwickAHRS.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Node ID (set to 1 for Node A, 2 for Node B)
#define NODE_ID 2

static const uint8_t MUX_ADDR = 0x70;
static const uint8_t ICM_ADDR = 0x69; // change to 0x68 if not working

static const float FILTER_HZ = 50.0f;
static const uint32_t LOOP_DELAY_MS = 20;

static const uint8_t NUM_IMUS = 4;
static const uint8_t IMU_CHANNELS[NUM_IMUS] = {0, 1, 2, 3};

static const int WIFI_CHANNEL = 6;

// Broadcast MAC (send to ANYONE listening) --> put in central MC mac address
static uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// IMU + filter objects
ICM_20948_I2C imu;
Madgwick filters[NUM_IMUS];

// Define the packet structure
#pragma pack(push, 1) // no padding - exact bytes on the wire
struct QuatPacket {
  uint8_t  node_id;
  uint32_t seq;
  uint32_t t_ms;
  int16_t  quat_q14[NUM_IMUS][4]; // [imu][w,x,y,z] in Q14
};
#pragma pack(pop)

QuatPacket pkt;
uint32_t seqCounter = 0;

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
  // Quat components should be [-1, 1]. Clamp to be safe.
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

// Check if ESP-NOW send failed (radio-level)
// ESP32 Arduino core 3.x uses wifi_tx_info_t* as the first callback argument.
void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW send FAIL");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  Wire.setClock(400000);

  // Get all filters ready
  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    filters[i].begin(FILTER_HZ);
  }

  Serial.print("Mux addr: 0x");
  Serial.print(MUX_ADDR, HEX);
  Serial.print(" | IMU addr: 0x");
  Serial.println(ICM_ADDR, HEX);

  // Initialize IMUs
  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    uint8_t ch = IMU_CHANNELS[i];
    bool ok = initImuOnChannel(ch);
    Serial.printf("Channel %u init: %s\n", ch, ok ? "OK" : "FAIL");
    delay(50);
  }

  // ESP-NOW works when WiFi is in station mode + disconnected from networks
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Lock in the WiFi channel
  esp_wifi_set_promiscuous(true); //Wifi mode needs to be in promiscuous to change channels
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE); //change to assigned wifi channel with NO secondary channel
  esp_wifi_set_promiscuous(false); //turn off promiscuous once finished

  // Initialize ESP-NOW, if failed freeze program
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);

  // Add broadcast peer (peer is WHO we are sending too)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BCAST_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  //check for failures and freeze program
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    while (true) delay(1000);
  }

  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Setup complete.");
}

void loop() {
  // Packet header
  pkt.node_id = NODE_ID;
  pkt.seq = seqCounter++;
  pkt.t_ms = millis();

  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    uint8_t ch = IMU_CHANNELS[i];
    muxSelect(ch);

    imu.getAGMT(); // reads sensors; result/status is stored on the imu object
    if (imu.status == ICM_20948_Stat_Ok) {
      float ax = imu.accX(), ay = imu.accY(), az = imu.accZ();
      float gx = imu.gyrX(), gy = imu.gyrY(), gz = imu.gyrZ(); // deg/s
      float mx = imu.magX(), my = imu.magY(), mz = imu.magZ();

      // Madgwick update() expects gyro in rad/s, but the ICM reports deg/s.
      gx *= DEG_TO_RAD; gy *= DEG_TO_RAD; gz *= DEG_TO_RAD;

      // Update Madgwick filter
      filters[i].update(gx, gy, gz, ax, ay, az, mx, my, mz);

      // Pull orientation as a quaternion (via the public Euler getters)
      float fqw, fqx, fqy, fqz;
      eulerToQuat(filters[i].getRollRadians(),
                  filters[i].getPitchRadians(),
                  filters[i].getYawRadians(),
                  fqw, fqx, fqy, fqz);

      // Save IMU data to packet (Q14)
      int16_t qw = toQ14(fqw);
      int16_t qx = toQ14(fqx);
      int16_t qy = toQ14(fqy);
      int16_t qz = toQ14(fqz);

      pkt.quat_q14[i][0] = qw;
      pkt.quat_q14[i][1] = qx;
      pkt.quat_q14[i][2] = qy;
      pkt.quat_q14[i][3] = qz;

      // Print packed values + SUCCESS (per IMU)
      Serial.printf("PACKED CH%u -> Q14 [w=%d x=%d y=%d z=%d]  SUCCESS\n", ch, qw, qx, qy, qz);
    } else {
      // Mark as invalid if read fails
      pkt.quat_q14[i][0] = 0;
      pkt.quat_q14[i][1] = 0;
      pkt.quat_q14[i][2] = 0;
      pkt.quat_q14[i][3] = 0;

      Serial.printf("CH%u | read FAIL\n", ch);
    }
  }

  // Send one packet containing all 4 IMU quaternions
  esp_now_send(BCAST_MAC, (uint8_t*)&pkt, sizeof(pkt)); //send pkt. Parameters: Broadcast MAC of who we are sending too, packet memory address("&" signifies memory address), packet byte size
  Serial.println("PACKET SEND CALLED");

  Serial.println("---");
  delay(LOOP_DELAY_MS);
}
