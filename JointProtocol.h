#pragma once

// contract between brain ESP32 and one joint ESP32 
// both boards must use same packet layout and protocol version 

#include <Arduino.h>

namespace lleap{
  // both boards use same wifi channel 
  constexpr uint8_t kEspNowChannel = 1;

  constexpr uint32_t kMessageMagic = 0x4C4C4541; 

  // update when packet changes
  constexpr uint8_t kProtocolVersion = 3; 

  // change desired joint state. heartbeats prove that brain is alive. do not overwrite position command 
  enum class PacketType : uint8_t{
    Command = 1, 
    Heartbeat = 2, 
    Fault = 3, 
  }; 

  // trajectory prepped by brain controller. joint controller uses positionFinal as its PID setpoint
  // retain other fields so future trajectory can use same packet 
  struct __attribute__((packed)) TrajectoryCommand{
    float positionInitialDeg; 
    float velocityInitialDegPerSec; 
    float positionFinalDeg; 
    float velocityFinalDegPerSec; 
    float durationSec; 
  }; 

  // ESP-NOW transfers raw bytes so packet contains 30 bytes with no padding 
  struct __attribute__((packed)) JointPacket {
    unit32_t magic; 
    uint8_t version; 
    PacketType type; 
    TrajectoryCommand command; 
    uint32_t sequence; 
  }; 


  static_assert(sizeof(JointPacket) == 30, 
    "Both ESP32 boards must use same packed packet size.");

}
