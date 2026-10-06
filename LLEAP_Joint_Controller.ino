// Board B

// import libraries 
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "JointComms.h"

// Motor control pins (ESCON Controller Output Configuration)
#define DAC1 25
#define enable1 33
#define direction1 32
 
// Encoder (AMT22 SPI Absolute Encoder Configuration)
#define CS_PIN 5
#define AMT22_NOP 0x00
#define AMT22_ZERO 0x70
#define NUM_POSITIONS_PER_REV 16384
 
// Control loop parameters
#define TOLERANCE_STOP 1.5f          
#define TOLERANCE_START 2.0f         
#define CONTROL_PERIOD_US 1000UL     // Strict 1 ms = 1000 Hz Cadence
#define DEBUG_PERIOD_MS 100UL        // Telemetry print at 10 Hz
#define DEADBAND_U 2.0f
#define MAX_QUEUE_SIZE 32

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 250; // Timeout threshold

// TARGET BOARD MAC = Board A
const uint8_t PEER_MAC[6] = { 0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C }; 

JointComms comms; 
bool commsReady = false; 

// Trackers
TrajectoryCommand latestTrajectory = {}; 
uint32_t latestTrajectorySequence = 0; 
bool trajectoryLogPending = false; 
uint32_t lastNetworkUpdateMs = 0;

// High-Resolution Jitter Diagnosis Instrumentation (For your Data Report)
uint32_t maxJitterObservedUs = 0;
uint32_t totalControlLoopsRun = 0;
 
bool motorEnabled = false; // State
 
// Control loop variables
float target_pos = 0.0f;
float prev_pos = 0.0f;
float integral_error = 0.0f;
float derivative_error = 0.0f;
 
float kp = 5.0f; // Gains
float ki = 0.0f;
float kd = 0.0f;
 
int maxSpeedCmd = 255;
int minSpeedCmd = 30;
float integral_limit = 50.0f;
float alpha = 0.1f;
 
char cmdBuffer[64];
int cmdIndex = 0;
 
uint32_t lastControlTimeUs = 0;
uint32_t lastDebugTimeMs = 0;
 
float debug_current_pos = 0.0f;
float debug_error = 0.0f;
float debug_u = 0.0f;
int debug_vel = 0;
int debug_dir = 0;
float curVel = 0;
 
float posQueue[MAX_QUEUE_SIZE];
int queueLength = 0;
int queueIndex = 0;
bool queueRunning = false;

// --------------------------------------------------------------------
// ASYNCHRONOUS CRITICAL EMERGENCY PATH (<20ms hardware cutoff)
// --------------------------------------------------------------------
void immediateFaultStop() {
  digitalWrite(enable1, LOW); // Instantly drop ESCON enable signal low
  dacWrite(DAC1, 0);          // Clamp motor velocity command voltage to 0
  motorEnabled = false;
}

bool motorEStopActive() {
  if (!comms.estopLatched()) return false; 
  immediateFaultStop();
  integral_error = 0.0f; 
  return true; 
}
 
void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Collect and display local board identifier for easy setup debugging
  WiFi.mode(WIFI_STA);
  // print Board B MAC address: 
  // Serial.print("\n>>> BOARD B LOCAL MAC ADDRESS: ");
  // Serial.println(WiFi.macAddress());
  // Serial.println("-------------------------------------------------");
 
  setup_encoder();
  setup_motor_controller();
 
  disableMotor();
  setMotor(0, 0);
  delay(100);
 
  commsReady = comms.begin(PEER_MAC, immediateFaultStop);
  if (commsReady) {
    Serial.println("ESP-NOW Joint Communication System Active."); 
  } else {
    Serial.println("CRITICAL ERROR: ESP-NOW Init Failed!");
  }
 
  target_pos = readEncoderPositionDeg();
  prev_pos = target_pos;
  lastControlTimeUs = micros();
  lastDebugTimeMs = millis();
  lastNetworkUpdateMs = millis();
}
 
void loop() {
  if (motorEStopActive()) return; 

  // Retrieve incoming trajectory messages from decoupled network buffer
  TrajectoryCommand receivedCommand; 
  uint32_t receivedSequence; 
  if (comms.takeData(receivedCommand, receivedSequence)) {
    latestTrajectory = receivedCommand; 
    latestTrajectorySequence = receivedSequence; 
    trajectoryLogPending = true; 
    
    target_pos = latestTrajectory.positionFinal; // Hand tracking milestone over to PID engine
    lastNetworkUpdateMs = millis();              // Refresh safety watchdog window
  }

  // Network Watchdog Check: Shut down if the network peer goes completely silent
  if (commsReady && (millis() - lastNetworkUpdateMs > WATCHDOG_TIMEOUT_MS) && motorEnabled) {
    Serial.println("\n[WATCHDOG TRIP] Peer Link Dropped. System Isolated.");
    immediateFaultStop();
  }
 
  processSerial();
 
  // High-Resolution 1kHz Deterministic Controller Run Window
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlTimeUs) >= CONTROL_PERIOD_US) {
    
    // Performance instrumentation check (Jitter tracking)
    uint32_t actualDeltaUs = nowUs - lastControlTimeUs;
    uint32_t currentJitterUs = (actualDeltaUs > CONTROL_PERIOD_US) ? 
                               (actualDeltaUs - CONTROL_PERIOD_US) : 
                               (CONTROL_PERIOD_US - actualDeltaUs);
                               
    if (currentJitterUs > maxJitterObservedUs && totalControlLoopsRun > 50) {
      maxJitterObservedUs = currentJitterUs;
    }
    totalControlLoopsRun++;

    lastControlTimeUs += CONTROL_PERIOD_US;
    runController();
  }
 
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastDebugTimeMs) >= DEBUG_PERIOD_MS) {
    lastDebugTimeMs += DEBUG_PERIOD_MS;
    printDebug();
  }
}
 
void runController() {
  float deltaT = CONTROL_PERIOD_US / 1.0e6f;
  float current_pos = readEncoderPositionDeg();
  float error = target_pos - current_pos;
 
  if (fabs(error) >= 180 && error < 0) {
    error += 360;
  } else if (fabs(error) >= 180 && error > 0) {
    error -= 360;
  }
 
  debug_current_pos = current_pos;
  debug_error = error;
 
  if (!motorEnabled) {
    setMotor(0, 0);
    integral_error = 0.0f;
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
    return;
  }
 
  static bool inDeadband = false;
  if (fabsf(error) < TOLERANCE_STOP) {
    inDeadband = true;
  } else if (fabsf(error) > TOLERANCE_START) {
    inDeadband = false;
  }
 
  if (inDeadband) {
    setMotor(0, 0);
    prev_pos = current_pos;
    debug_u = 0.0f;
    debug_vel = 0;
    debug_dir = 0;
 
    if (queueRunning && queueIndex < queueLength) {
      target_pos = posQueue[queueIndex++];
      inDeadband = false;
      integral_error = 0.0f;
    } else if (queueRunning && queueIndex >= queueLength) {
      queueRunning = false;
    }
    return;
  }
 
  integral_error += error * deltaT;
  if (integral_error > integral_limit) integral_error = integral_limit;
  if (integral_error < -integral_limit) integral_error = -integral_limit;
 
  float raw_derivative = -(current_pos - prev_pos) / deltaT;
  derivative_error = alpha * raw_derivative + (1.0f - alpha) * derivative_error;
  prev_pos = current_pos;
 
  float u = (kp * error) + (ki * integral_error) + (kd * derivative_error);
  int dir = (u < 0.0f) ? 0 : 1;
 
  float abs_u = fabsf(u);
  curVel = 0;
 
  if (abs_u > DEADBAND_U) {
    curVel = constrain((int)abs_u, 0, maxSpeedCmd);
    if (curVel > 0 && curVel < minSpeedCmd) {
      curVel = (fabsf(error) > 5.0f) ? minSpeedCmd : 0;
    }
  }
 
  setMotor(dir, curVel);
  debug_u = u;
  debug_vel = curVel;
  debug_dir = dir;
}
 
void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) handleCommand(cmdBuffer);
      cmdIndex = 0;
    } else {
      if (cmdIndex < (int)sizeof(cmdBuffer) - 1) cmdBuffer[cmdIndex++] = c;
    }
  }
}
 
void handleCommand(const char* cmd) {
  if (strcmp(cmd, "Z") == 0 || strcmp(cmd, "z") == 0) {
    disableMotor(); setMotor(0, 0); delay(50);
    setZeroSPI(CS_PIN); delay(50);
    target_pos = readEncoderPositionDeg(); prev_pos = target_pos; integral_error = 0.0f;
    return;
  }
  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "s") == 0) {
    printStatus(readEncoderPositionDeg()); return;
  }
  if (strcmp(cmd, "X") == 0 || strcmp(cmd, "x") == 0) {
    queueRunning = false; queueLength = 0; queueIndex = 0; return;
  }
 
  if (cmd[0] == 'T' || cmd[0] == 't') {
    queueLength = 0; queueIndex = 0; queueRunning = false;
    const char* ptr = cmd + 2;
    while (*ptr != '\0' && queueLength < MAX_QUEUE_SIZE) {
      while (*ptr == ' ') ptr++;
      if (*ptr == '\0') break;
      posQueue[queueLength++] = constrain(atof(ptr), 0.0f, 359.9f);
      while (*ptr != ' ' && *ptr != '\0') ptr++;
    }
    target_pos = posQueue[0];
    integral_error = 0.0f;
    return;
  }
 
  char key; float value;
  if (sscanf(cmd, "%c %f", &key, &value) != 2) return;
 
  switch (key) {
    case 'P': case 'p': kp = value; break;
    case 'I': case 'i': ki = value; integral_error = 0.0f; break;
    case 'D': case 'd': kd = value; break;
    case 'M': case 'm': maxSpeedCmd = constrain((int)value, 0, 255); break;
    case 'E': case 'e': if ((int)value == 1) enableMotor(); else disableMotor(); break;
  }
}
 
void printDebug() {
  Serial.print("cur=");         Serial.print(debug_current_pos, 2);
  Serial.print(" tgt=");         Serial.print(target_pos, 2);
  Serial.print(" err=");         Serial.print(debug_error, 2);
  Serial.print(" MaxJitterUs="); Serial.println(maxJitterObservedUs);
 
  if (trajectoryLogPending) {
    trajectoryLogPending = false; 
    Serial.println("--- LLEAP Net Data Inbound ---"); 
    Serial.print("Seq: ");       Serial.println(latestTrajectorySequence); 
    Serial.print("TargetDeg: "); Serial.println(latestTrajectory.positionFinal, 2); 
    Serial.println("------------------------------"); 
  }
}
 
void printStatus(float current_pos) {
  Serial.println("----- STATUS -----");
  Serial.print("enabled: ");       Serial.println(motorEnabled ? "YES" : "NO");
  Serial.print("current_pos: ");   Serial.println(current_pos, 3);
  Serial.print("target_pos: ");    Serial.println(target_pos, 3);
  Serial.print("Max Jitter (us): "); Serial.println(maxJitterObservedUs);
  Serial.println("------------------");
}
 
void enableMotor() {
  if (comms.estopLatched()) return;
  setMotor(0, 0);
  digitalWrite(enable1, HIGH);
  target_pos = readEncoderPositionDeg();
  prev_pos = target_pos;
  integral_error = 0.0f;
  motorEnabled = true;
  }
  void disableMotor() {
  digitalWrite(enable1, LOW);
  setMotor(0, 0);
  motorEnabled = false;
  }
  void setMotor(int dir, int vel) {
  vel = constrain(vel, 0, 255);
  digitalWrite(direction1, (dir == 1) ? LOW : HIGH);
  dacWrite(DAC1, vel);
  }
  void setup_encoder() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  }
  void setup_motor_controller() {
  pinMode(DAC1, OUTPUT); pinMode(enable1, OUTPUT); pinMode(direction1, OUTPUT);
  digitalWrite(direction1, LOW); dacWrite(DAC1, 0); digitalWrite(enable1, LOW);
  }
  uint16_t readEncoderPosition14Bit(void) {
  uint16_t position = 0;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW); delayMicroseconds(3);
  position = SPI.transfer(AMT22_NOP); position <<= 8; delayMicroseconds(3);
  position |= SPI.transfer(AMT22_NOP);
  digitalWrite(CS_PIN, HIGH); SPI.endTransaction();
  return position & 0x3FFF;
  }
  float encoderReadingToDeg(uint16_t position) { return 360.0f * ((float)position / (NUM_POSITIONS_PER_REV - 1)); }
  float readEncoderPositionDeg(void) { return encoderReadingToDeg(readEncoderPosition14Bit()); }
  void setZeroSPI(uint8_t cs_pin) {  /* Left stock from your original backup */ }
