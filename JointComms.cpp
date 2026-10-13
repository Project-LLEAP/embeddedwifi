#include "JointComms.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

JointComms* JointComms::instance_ = nullptr;

bool JointComms::begin(const uint8_t expectedPeerMac[6], FaultHook faultHook) {
  memcpy(expectedPeerMac_, expectedPeerMac, sizeof(expectedPeerMac_));
  faultHook_ = faultHook;
  instance_ = this;

  // ESP-NOW uses the Wi-Fi radio, but it does not need an access point.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(50);

  if (esp_wifi_set_channel(lleap::kEspNowChannel,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  // Register the peer so this board may also send heartbeats or a fault.
  if (esp_now_is_peer_exist(expectedPeerMac_)) {
    esp_now_del_peer(expectedPeerMac_);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, expectedPeerMac_, sizeof(expectedPeerMac_));
  peerInfo.channel = lleap::kEspNowChannel;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }

  return esp_now_register_recv_cb(onReceive) == ESP_OK;
}

bool JointComms::sendCommand(const lleap::TrajectoryCommand& command,
                             uint32_t sequence) {
  return sendPacket(lleap::PacketType::Command, command, sequence);
}

bool JointComms::sendHeartbeat(uint32_t sequence) {
  const lleap::TrajectoryCommand emptyCommand = {};
  return sendPacket(lleap::PacketType::Heartbeat, emptyCommand, sequence);
}

bool JointComms::sendFault(uint32_t sequence) {
  const lleap::TrajectoryCommand emptyCommand = {};
  return sendPacket(lleap::PacketType::Fault, emptyCommand, sequence);
}

bool JointComms::takeCommand(lleap::TrajectoryCommand& command,
                             uint32_t& sequence) {
  portENTER_CRITICAL(&dataMutex_);
  if (!commandPending_) {
    portEXIT_CRITICAL(&dataMutex_);
    return false;
  }

  command = latestCommand_;
  sequence = latestCommandSequence_;
  commandPending_ = false;
  portEXIT_CRITICAL(&dataMutex_);
  return true;
}

bool JointComms::estopLatched() const {
  return estopLatched_;
}

bool JointComms::hasReceivedCommand() const {
  return hasReceivedCommand_;
}

uint32_t JointComms::lastValidPacketMs() const {
  return lastValidPacketMs_;
}

void JointComms::onReceive(const esp_now_recv_info_t* receiveInfo,
                           const uint8_t* incomingData,
                           int length) {
  if (instance_ != nullptr) {
    instance_->handleReceive(receiveInfo, incomingData, length);
  }
}

void JointComms::handleReceive(const esp_now_recv_info_t* receiveInfo,
