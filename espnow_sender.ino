// Board A = sender
// sends normal heartbeat packets then intentionally sends a fault 
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t ESPNOW_CHANNEL = 1; // both radios are on the same Wi-Fi channel 
constexpr uint32_t MESSAGE_MAGIC = 0x4C4C4541;  // "LLEA" // packets belonging to project 
constexpr uint8_t PROTOCOL_VERSION = 1; // future versions reject imcompatible packets safely 
constexpr uint32_t FAULT_AFTER_HEARTBEATS = 10;

enum MessageType : uint8_t { // heartbeat vs fault 
  HEARTBEAT = 1,
  FAULT = 2
};

uint8_t boardBMac[] = {
  0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C
};

struct __attribute__((packed)) JointMessage { // prevents padding bytes - boards a and b share same packet size 
  uint32_t magic; // checks if one of our packets 
  uint8_t version; // checks expected packet format 
  uint8_t type; // heartbeat or fault 
  uint16_t reserved; // reserved for later use 
  uint32_t sequence; // packet number 
};

uint32_t sequence = 0;
bool fault_test_sent = false;

void sendMessage(MessageType type, uint32_t message_sequence) {
  JointMessage message = {
    MESSAGE_MAGIC,
    PROTOCOL_VERSION,
    type,
    0,
    message_sequence
  };

  esp_err_t result = esp_now_send(
      boardBMac,
      reinterpret_cast<uint8_t *>(&message),
      sizeof(message)
  );

  if (type == FAULT) {
    Serial.print("FAULT send result: ");
  } else {
    Serial.print("Heartbeat ");
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

  esp_now_peer_info_t peer_info =  {};
  memcpy(peer_info.peer_addr, boardBMac, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("ERROR: Could not add Board B as a peer.");
    return;
  }

  Serial.println("Board A fault-test sender ready.");
}

void loop() {
  if (!fault_test_sent && sequence >= FAULT_AFTER_HEARTBEATS) {
    Serial.println("Sending three FAULT packets...");

    // Repeating a safety-critical packet helps tolerate a dropped packet.
    for (int i = 0; i < 3; i++) {
      sendMessage(FAULT, sequence);
      delay(100);
    }

    fault_test_sent = true;
  } else {
    sendMessage(HEARTBEAT, sequence);
    sequence++;
  }

  delay(1000);
}