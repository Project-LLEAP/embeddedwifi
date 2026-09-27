// Board B - receiver / safety controller 
// accepts valid hearbeats until a valid fault arrives 
// if valid fault, latches e-stop and ignores later normal messages

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t MESSAGE_MAGIC = 0x4C4C4541;  // "LLEA"
constexpr uint8_t PROTOCOL_VERSION = 2;

enum MessageType : uint8_t {
  DATA = 1,
  FAULT = 2
};

struct __attribute__((packed)) JointMessage { // prevents padding bytes - boards a and b share same packet size 
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

volatile bool data_pending = false;
volatile bool fault_pending = false;
volatile bool estop_latched = false;

volatile uint32_t data_count = 0;
volatile uint32_t fault_sequence = 0;

volatile JointMessage message;

void onDataRecv(const esp_now_recv_info_t *recv_info,
                const uint8_t *incoming_data,
                int len) {
  if (len != sizeof(JointMessage)) return;

  memcpy(&message, incoming_data, sizeof(message));

  if (message.magic != MESSAGE_MAGIC) return;
  if (message.version != PROTOCOL_VERSION) return;

  if (message.type == FAULT) {
    // This remains true until Board B is reset.
    estop_latched = true;
    fault_sequence = message.sequence;
    fault_pending = true;
    return;
  }

  // Ignore ordinary commands once a fault has latched.
  if (estop_latched) return;

  if (message.type == DATA) {
    data_count++;
    data_pending = true;
  }
}

void handleData() {
  //Set PID setpoint to data here.

  //Placeholder print messages
  Serial.print("Received Message");
  Serial.print("Initial Position:");
  Serial.print(message.position_initial);
  Serial.print("Initial Velocity:");
  Serial.print(message.velocity_initial);
  Serial.print("Final Position:");
  Serial.print(message.position_final);
  Serial.print("Final Velocity:");
  Serial.print(message.velolcity_final);
  Serial.print("Duration:");
  Serial.print(message.duration);

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

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("ERROR: Could not register receive callback.");
    return;
  }

  Serial.println("Board B safety receiver ready.");
  Serial.println("State: RUNNING");
}

void loop() {
  if (fault_pending) {
    fault_pending = false;

    Serial.print("FAULT received. Sequence: ");
    Serial.println(fault_sequence);
    Serial.println("State: E-STOP LATCHED. Reset Board B to clear it.");
  }

  if (data_pending) {
    data_pending = false;

    Serial.print("Data sequence ");
    Serial.print(data_count);
    Serial.print("received.");
    handleData(); //Handle sending data to motor controller
  }
}