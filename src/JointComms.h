#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t JOINT_MESSAGE_MAGIC = 0x4C4C4541; // "LLEA"
constexpr uint8_t JOINT_PROTOCOL_VERSION = 2;

enum class JointMessageType : uint8_t {
  Data = 1,
  Fault = 2
};

struct __attribute__((packed)) TrajectoryCommand {
  float positionInitial;
  float velocityInitial;
  float positionFinal;
  float velocityFinal;
  float duration;
};

struct __attribute__((packed)) JointMessage {
  uint32_t magic;
  uint8_t version;
  JointMessageType type;
  TrajectoryCommand command;
  uint32_t sequence;
};

using FaultHook = void (*)();

class JointComms {
public:
  bool begin(const uint8_t peerMac[6], FaultHook faultHook) {
    memcpy(peerMac_, peerMac, sizeof(peerMac_));
    faultHook_ = faultHook;
    instance_ = this;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      return false;
    }

    if (esp_now_init() != ESP_OK) {
      return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac_, sizeof(peerMac_));
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      return false;
    }

    if (esp_now_register_recv_cb(onReceive) != ESP_OK) {
      return false;
    }

    return true;
  }

  bool sendData(const TrajectoryCommand& command, uint32_t sequence) {
    return sendMessage(JointMessageType::Data, command, sequence);
  }

  bool sendFault(uint32_t sequence) {
    TrajectoryCommand emptyCommand = {}; 
    return sendMessage(JointMessageType::Fault, emptyCommand, sequence);
  }

  bool estopLatched() const { return estopLatched_; }

  bool takeData(TrajectoryCommand& command, uint32_t& sequence) {
    portENTER_CRITICAL(&dataMutex_); 
    if (!dataPending_){
      portEXIT_CRITICAL(&dataMutex_); 
      return false; 
    }
    command = latestCommand_; 
    sequence = latestDataSequence_;
    dataPending_ = false;
    portEXIT_CRITICAL(&dataMutex_); 
    return true;
  }

private:
  static JointComms* instance_;

  static void onReceive(const esp_now_recv_info_t* recvInfo, const uint8_t* incomingData, int len) {
    (void) recvInfo; 
    if (instance_ != nullptr) {
      instance_->handleReceive(incomingData, len);
    }
  }

  void handleReceive(const uint8_t* incomingData, int len) {
    if (len != sizeof(JointMessage)) return;

    JointMessage message;
    memcpy(&message, incomingData, sizeof(message));

    if (message.magic != JOINT_MESSAGE_MAGIC || message.version != JOINT_PROTOCOL_VERSION) {
      return;
    }

    if (message.type == JointMessageType::Fault) {
      estopLatched_ = true;
      if (faultHook_ != nullptr) {
        faultHook_();
      }
      return;
    }

    if (estopLatched_) return;

    if (message.type == JointMessageType::Data) {
      portENTER_CRITICAL(&dataMutex_); 
      latestCommand_ = message.command; 
      latestDataSequence_ = message.sequence;
      dataPending_ = true;
      portEXIT_CRITICAL(&dataMutex_); 
    }
  }

  bool sendMessage(JointMessageType type, const TrajectoryCommand& command, uint32_t sequence) {
    JointMessage message = { JOINT_MESSAGE_MAGIC, JOINT_PROTOCOL_VERSION, type, command, sequence };
    return esp_now_send(peerMac_, reinterpret_cast<const uint8_t*>(&message), sizeof(message)) == ESP_OK;
  }

  uint8_t peerMac_[6] = {};
  FaultHook faultHook_ = nullptr;
  volatile bool estopLatched_ = false;
  volatile bool dataPending_ = false;
  TrajectoryCommand latestCommand_ = {};
  uint32_t latestDataSequence_ = 0;
  portMUX_TYPE dataMutex_ = portMUX_INITIALIZER_UNLOCKED;
};

// Allocate the static pointer required for Arduino compilation linkage
JointComms* JointComms::instance_ = nullptr;
