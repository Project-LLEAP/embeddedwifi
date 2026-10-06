//File 3: `LLEAP_TestBench_Sender.ino` (Flash to BOARD A)

// import libraries 
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t JOINT_MESSAGE_MAGIC = 0x4C4C4541; // "LLEA"
constexpr uint8_t JOINT_PROTOCOL_VERSION = 2;

// define message type 
enum class JointMessageType : uint8_t { Data = 1, Fault = 2 };

struct __attribute__((packed)) TrajectoryCommand { // data packets come with position, velocity, and duration info
  float positionInitial; float velocityInitial;
  float positionFinal;   float velocityFinal;
  float duration;
};

struct __attribute__((packed)) JointMessage {
  uint32_t magic; uint8_t version; JointMessageType type;
  TrajectoryCommand command; uint32_t sequence;
};

// TARGET MAC = Board B MAC Address
uint8_t pidBoardMac[6] = { 0xE0, 0x8C, 0xFE, 0xF5, 0x7E, 0x64 }; 

uint32_t sequence = 0;
uint32_t testTimer = 0;
int executionState = 0;

// function to send data packet 
void sendCommand(JointMessageType type, TrajectoryCommand cmd) {
  JointMessage msg = { JOINT_MESSAGE_MAGIC, JOINT_PROTOCOL_VERSION, type, cmd, sequence++ };
  esp_now_send(pidBoardMac, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, pidBoardMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("[BOARD A] Demo Master Controller Running...");
  testTimer = millis();
}

void loop() {
  uint32_t elapsed = millis() - testTimer;

  // Automated script mimicking a real gait progression ending in an incident trip
  if (elapsed >= 5000) {
    testTimer = millis();
    executionState++;

    if (executionState == 1) {
      Serial.println(">> Pushing Profile Milestone 1: Target 45°");
      sendCommand(JointMessageType::Data, {0.0f, 0.0f, 45.0f, 0.0f, 1.0f});
    } 
    else if (executionState == 2) {
      Serial.println(">> Pushing Profile Milestone 2: Target 90°");
      sendCommand(JointMessageType::Data, {45.0f, 0.0f, 90.0f, 0.0f, 1.0f});
    } 
    else if (executionState == 3) {
      Serial.println(">> Pushing Profile Milestone 3: Target 15°");
      sendCommand(JointMessageType::Data, {90.0f, 0.0f, 15.0f, 0.0f, 1.0f});
    } 
    else if (executionState == 4) {
      Serial.println("!!!!! DEMO FLIGHT CRITICAL FAULT INJECTION !!!!!");
      // Fire 3 quick successive packets to overcome any radio packet drop noise
      for(int i=0; i<3; i++) {
        sendCommand(JointMessageType::Fault, {0,0,0,0,0});
      }
    }
  }

  // Regular continuous synchronization heartbeat (Keep-Alive window safety)
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 100 && executionState < 4) {
    lastHeartbeat = millis();
    // Neutral telemetry update to maintain safe network watchdog connectivity
    sendCommand(JointMessageType::Data, {0,0, 0.0f, 0,0}); 
  }
}