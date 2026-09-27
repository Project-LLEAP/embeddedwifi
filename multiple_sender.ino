// Board A = sender
// sends normal heartbeat packets then intentionally sends a fault 
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t ESPNOW_CHANNEL = 1; // both radios are on the same Wi-Fi channel 
constexpr uint32_t MESSAGE_MAGIC = 0x4C4C4541;  // "LLEA" // packets belonging to project 
constexpr uint8_t PROTOCOL_VERSION = 2; // future versions reject imcompatible packets safely 
constexpr uint32_t FAULT_AFTER_HEARTBEATS = 10;

enum MessageType : uint8_t { // data vs fault 
  DATA = 1,
  FAULT = 2
};

enum JointIndex : uint8_t {
  LEFT_HIP = 0;
  LEFT_KNEE = 1;
  RIGHT_HIP = 2;
  RIGHT_KNEE = 3;
}

uint8_t jointMacs[4][6] = {
  {0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C}, // Left Hip
  {0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C}, // Left Knee
  {0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C}, // Right Hip
  {0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C}, // Right Knee
};

struct __attribute__((packed)) JointMessage { // prevents padding bytes - all boards share same packet size 
  uint32_t magic; // checks if one of our packets 
  uint8_t version; // checks expected packet format 
  uint8_t type; // data or fault 
  float position_initial; // cubic spline data values
  float velocity_initial;
  float position_final;
  float velocity_final;
  float duration;
  uint32_t sequence; // packet number 
};

uint32_t sequence = 0;

void sendMessage(JointIndex joint, MessageType type, uint32_t message_sequence) {
  JointMessage message = {
    MESSAGE_MAGIC,
    PROTOCOL_VERSION,
    type,
    0,
    0,
    45,
    0,
    5
    message_sequence
  };

  esp_err_t result = esp_now_send(
      jointMacs[joint],
      reinterpret_cast<uint8_t *>(&message),
      sizeof(message)
  );

  if (type == FAULT) {
    Serial.print("FAULT send result: ");
  } else {
    Serial.print("Data ");
    Serial.print(message_sequence);
    Serial.print(" | send result: ");
  }

  Serial.println(result);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed.");
    return;
  }

  esp_now_peer_info_t peer_info; // Data for each peer
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  for (uint8_t i = 0; i < 4; i++) { // Loop over each 
      
    memcpy(peer_info[i].peer_addr, jointMacs[i], 6);
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
      Serial.print("ERROR: Could not add Board ");
      Serial.print(i+1);
      Serial.println(" as peer.");
      return;
    }
      
  }

  Serial.println("Brain board motor control data sender ready.");
}

void loop() {
  //Send separate test messages to each joint
  sendMessage(LEFT_HIP, sequence);
  sendMessage(LEFT_KNEE, sequence);
  sendMessage(RIGHT_HIP, sequence);
  sendMessage(RIGHT_KNEE, sequence);

  delay(3000);
}

void loop_fault() {
  if (!fault_test_sent && sequence >= FAULT_AFTER_HEARTBEATS) {
    Serial.println("Sending three FAULT packets...");

    // Repeating a safety-critical packet helps tolerate a dropped packet.
    for (int i = 0; i < 3; i++) {
      sendMessage(FAULT, sequence);
      delay(100);
    }

    fault_test_sent = true;
  } else {
    sendMessage(DATA, sequence);
    sequence++;
  }

  delay(1000);
}