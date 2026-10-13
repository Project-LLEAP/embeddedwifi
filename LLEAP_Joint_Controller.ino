/*
  LLEAP_Joint_Controller.ino

  Flash this sketch to the joint ESP32 (Board B).  This board owns the fast
  encoder/PID/ESCON loop.  ESP-NOW only supplies desired joint data and safety
  messages; it never runs the PID calculation inside the radio callback.

  IMPORTANT: Keep the ESCON/motor power disconnected for the first radio test.
  Confirm every ESCON input's required voltage and polarity before connecting it.
*/

#include <SPI.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <JointComms.h>

// ---------------------------------------------------------------------------
// 1. Board-specific settings: edit these before flashing.
// ---------------------------------------------------------------------------

// Replace with Board A's printed STA MAC address (not its Bluetooth MAC).
const uint8_t BRAIN_MAC[6] = {0xA0, 0xB7, 0x65, 0x21, 0xEE, 0x9C};

// ESCON command wiring.  These are signal pins only; never connect an ESP32
// pin to the ESCON motor-power terminals.
constexpr uint8_t kDacPin = 25;
constexpr uint8_t kEnablePin = 33;
constexpr uint8_t kDirectionPin = 32;

// AMT22 absolute encoder SPI chip-select pin.
constexpr uint8_t kEncoderChipSelectPin = 5;
constexpr uint8_t kAmt22Nop = 0x00;
constexpr uint8_t kAmt22Zero = 0x70;
constexpr uint16_t kEncoderCountsPerRevolution = 16384;

// Verify the ESCON input polarity on the actual controller before changing
// these.  Start at the low DAC limit and increase only after a safe test.
constexpr bool kDirectionLowMeansPositive = true;
constexpr uint8_t kInitialDacLimit = 30;  // 30/255 of the ESP32 DAC range.

// Replace these with the *mechanically safe* range for this one joint before
// mounting the mechanism.  Incoming commands outside this range are rejected.
constexpr float kJointMinDeg = 0.0f;
constexpr float kJointMaxDeg = 359.9f;

// A valid radio message must arrive this often while the ESCON is enabled.
constexpr uint32_t kWatchdogTimeoutMs = 250;

// ---------------------------------------------------------------------------
// 2. PID settings.  These values are starting points, not tuned gains.
// ---------------------------------------------------------------------------

constexpr uint32_t kControlPeriodUs = 1000;  // Nominal 1 kHz local PID loop.
constexpr uint32_t kDebugPeriodMs = 100;     // 10 Hz Serial telemetry.
constexpr float kStopToleranceDeg = 1.5f;
constexpr float kRestartToleranceDeg = 2.0f;
constexpr float kControlDeadband = 2.0f;

float kp = 5.0f;
float ki = 0.0f;
float kd = 0.0f;
float integralLimit = 50.0f;
float derivativeFilterAlpha = 0.1f;

// ---------------------------------------------------------------------------
// 3. State shared by the local PID loop and ESP-NOW application code.
// ---------------------------------------------------------------------------

JointComms comms;
bool commsReady = false;
volatile bool motorEnabled = false;

float targetPositionDeg = 0.0f;
float previousPositionDeg = 0.0f;
float integralError = 0.0f;
float filteredDerivative = 0.0f;
uint8_t dacCommandLimit = kInitialDacLimit;

uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t maxJitterUs = 0;
uint32_t controlCycles = 0;

float debugPositionDeg = 0.0f;
float debugErrorDeg = 0.0f;
float debugControlEffort = 0.0f;
uint8_t debugDacCommand = 0;
int debugDirection = 0;

lleap::TrajectoryCommand lastCommand = {};
uint32_t lastCommandSequence = 0;
bool commandLogPending = false;

char serialBuffer[64] = {};
uint8_t serialIndex = 0;

// ---------------------------------------------------------------------------
// 4. Safety functions.
// ---------------------------------------------------------------------------

// Called directly by the ESP-NOW receive callback after a validated FAULT.
// Do not print, delay, allocate memory, or do SPI work here.
void immediateFaultStop() {
  dacWrite(kDacPin, 0);             // Remove analog speed/torque command first.
  digitalWrite(kEnablePin, LOW);    // Then inhibit the ESCON signal input.
  motorEnabled = false;
}

// Re-applies the safe output state from loop().  A radio fault stays latched
// until the joint ESP32 is reset, so a later command cannot restart the motor.
bool estopIsActive() {
  if (!comms.estopLatched()) {
    return false;
  }
  immediateFaultStop();
  integralError = 0.0f;
  return true;
}

bool communicationIsFresh() {
  return comms.hasReceivedCommand() &&
         (millis() - comms.lastValidPacketMs() <= kWatchdogTimeoutMs);
}

void disableMotor() {
  dacWrite(kDacPin, 0);
  digitalWrite(kEnablePin, LOW);
  motorEnabled = false;
  integralError = 0.0f;
}

void enableMotor() {
  // The joint cannot be enabled until it has a recent valid command.  This
  // avoids enabling into an unknown target after boot or a radio dropout.
  if (!commsReady || comms.estopLatched() || !communicationIsFresh()) {
    disableMotor();
    Serial.println("Enable blocked: need recent command, ready radio, and no E-STOP.");
    return;
  }

  setMotor(0, 0);
  previousPositionDeg = readEncoderPositionDeg();
  integralError = 0.0f;
  digitalWrite(kEnablePin, HIGH);  // Verify HIGH is the safe enable polarity.
  motorEnabled = true;
  Serial.println("Motor enabled.");
}

// ---------------------------------------------------------------------------
// 5. Arduino setup and non-blocking application loop.
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  setupEncoder();
  setupMotorOutputs();
  disableMotor();                 // The motor must start inhibited.

  commsReady = comms.begin(BRAIN_MAC, immediateFaultStop);
  Serial.println();
  Serial.println("LLEAP joint controller starting.");
  Serial.print("This joint board MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println(commsReady ? "ESP-NOW ready." : "ERROR: ESP-NOW setup failed.");
  Serial.println("Commands: E 1, E 0, T <deg>, P <gain>, I <gain>, D <gain>, M <0-255>, S, Z");

  targetPositionDeg = readEncoderPositionDeg();
  previousPositionDeg = targetPositionDeg;
  lastControlUs = micros();
  lastDebugMs = millis();
}

void loop() {
  if (estopIsActive()) {
    return;  // The fault latch is intentionally reset-only.
  }

  receiveNewestCommand();

  // A valid heartbeat or command must keep arriving while the output is live.
  if (motorEnabled && !communicationIsFresh()) {
    Serial.println("WATCHDOG: valid brain traffic stopped; motor inhibited.");
    disableMotor();
  }

  processSerial();
  runControlLoopWhenDue();
  printDebugWhenDue();
}

// Reads the command buffer that JointComms filled in the radio callback.
void receiveNewestCommand() {
  lleap::TrajectoryCommand incoming = {};
  uint32_t sequence = 0;
  if (!comms.takeCommand(incoming, sequence)) {
    return;
  }

  // The current implementation is a setpoint controller: the final position
  // becomes the local PID target.  It does NOT yet interpolate a cubic segment
  // using the other four trajectory fields.
  if (incoming.positionFinalDeg < kJointMinDeg ||
      incoming.positionFinalDeg > kJointMaxDeg) {
    Serial.println("Rejected command: target outside configured joint limits.");
    return;
  }

  lastCommand = incoming;
  lastCommandSequence = sequence;
  targetPositionDeg = incoming.positionFinalDeg;
  commandLogPending = true;
}

// Uses the measured period rather than attempting a burst of catch-up PID
// cycles after a slow Serial print or radio event.
void runControlLoopWhenDue() {
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs < kControlPeriodUs) {
    return;
  }

  const uint32_t jitterUs = (elapsedUs > kControlPeriodUs)
                                ? elapsedUs - kControlPeriodUs
                                : kControlPeriodUs - elapsedUs;
  if (controlCycles > 50 && jitterUs > maxJitterUs) {
    maxJitterUs = jitterUs;
  }
  controlCycles++;
  lastControlUs = nowUs;

  // Cap the integration interval after a long stall to limit integral windup.
  const float dtSeconds = min(elapsedUs, 10000UL) / 1000000.0f;
  runPid(dtSeconds);
}

void runPid(float dtSeconds) {
  const float currentPositionDeg = readEncoderPositionDeg();
  float errorDeg = targetPositionDeg - currentPositionDeg;

  // The encoder wraps at 360 degrees, so use the shortest rotation direction.
  if (errorDeg > 180.0f) errorDeg -= 360.0f;
  if (errorDeg < -180.0f) errorDeg += 360.0f;

  debugPositionDeg = currentPositionDeg;
  debugErrorDeg = errorDeg;

  if (!motorEnabled) {
    setMotor(0, 0);
    integralError = 0.0f;
    previousPositionDeg = currentPositionDeg;
    debugControlEffort = 0.0f;
    debugDacCommand = 0;
    return;
  }

  // Hysteresis prevents the motor rapidly toggling on/off at the target.
  static bool insideTargetBand = false;
  if (fabsf(errorDeg) < kStopToleranceDeg) insideTargetBand = true;
  if (fabsf(errorDeg) > kRestartToleranceDeg) insideTargetBand = false;
  if (insideTargetBand) {
    setMotor(0, 0);
    previousPositionDeg = currentPositionDeg;
    debugControlEffort = 0.0f;
    debugDacCommand = 0;
    return;
  }

  integralError += errorDeg * dtSeconds;
  integralError = constrain(integralError, -integralLimit, integralLimit);

  const float rawDerivative = -(currentPositionDeg - previousPositionDeg) / dtSeconds;
  filteredDerivative = derivativeFilterAlpha * rawDerivative +
                       (1.0f - derivativeFilterAlpha) * filteredDerivative;
  previousPositionDeg = currentPositionDeg;

  const float effort = kp * errorDeg + ki * integralError + kd * filteredDerivative;
  const int direction = effort >= 0.0f ? 1 : -1;
  const float magnitude = fabsf(effort);
  const uint8_t dacValue = magnitude > kControlDeadband
                               ? constrain(static_cast<int>(magnitude), 0, dacCommandLimit)
                               : 0;

  setMotor(direction, dacValue);
  debugControlEffort = effort;
  debugDacCommand = dacValue;
  debugDirection = direction;
}

// ---------------------------------------------------------------------------
// 6. Hardware helpers.
// ---------------------------------------------------------------------------

void setupMotorOutputs() {
  pinMode(kDacPin, OUTPUT);
  pinMode(kEnablePin, OUTPUT);
  pinMode(kDirectionPin, OUTPUT);
  dacWrite(kDacPin, 0);
  digitalWrite(kEnablePin, LOW);
  digitalWrite(kDirectionPin, LOW);
}

void setMotor(int direction, uint8_t dacValue) {
  // Limit is deliberately separate from the PID math, so M <n> cannot exceed
  // the configured safe maximum.
  dacValue = min(dacValue, dacCommandLimit);
  const bool positive = direction > 0;
  const bool level = kDirectionLowMeansPositive ? !positive : positive;
  digitalWrite(kDirectionPin, level ? HIGH : LOW);
  dacWrite(kDacPin, dacValue);
}

void setupEncoder() {
  pinMode(kEncoderChipSelectPin, OUTPUT);
  digitalWrite(kEncoderChipSelectPin, HIGH);
  SPI.begin();
}

uint16_t readEncoderPosition14Bit() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kEncoderChipSelectPin, LOW);
  delayMicroseconds(3);
  uint16_t position = SPI.transfer(kAmt22Nop) << 8;
  delayMicroseconds(3);
  position |= SPI.transfer(kAmt22Nop);
  digitalWrite(kEncoderChipSelectPin, HIGH);
  SPI.endTransaction();

  // AMT22 includes status/parity bits.  This prototype keeps the 14 position
  // bits; add the exact parity check from the encoder's datasheet before a
  // powered wearable test.
  return position & 0x3FFF;
}

float readEncoderPositionDeg() {
  return 360.0f * readEncoderPosition14Bit() /
         static_cast<float>(kEncoderCountsPerRevolution - 1);
}

void zeroEncoder() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kEncoderChipSelectPin, LOW);
  delayMicroseconds(3);
  SPI.transfer(kAmt22Nop);
  delayMicroseconds(3);
  SPI.transfer(kAmt22Zero);
  digitalWrite(kEncoderChipSelectPin, HIGH);
  SPI.endTransaction();
  delay(250);
}

// ---------------------------------------------------------------------------
// 7. Human-readable serial commands and telemetry.
// ---------------------------------------------------------------------------

void processSerial() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') continue;
    if (character == '\n') {
      serialBuffer[serialIndex] = '\0';
      if (serialIndex > 0) handleSerialCommand(serialBuffer);
      serialIndex = 0;
    } else if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = character;
    }
  }
}

void handleSerialCommand(const char* command) {
  if (strcmp(command, "S") == 0 || strcmp(command, "s") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "Z") == 0 || strcmp(command, "z") == 0) {
    disableMotor();
    zeroEncoder();
    targetPositionDeg = readEncoderPositionDeg();
    previousPositionDeg = targetPositionDeg;
    Serial.println("Encoder zeroed; motor remains disabled.");
    return;
  }

  char key = 0;
  float value = 0.0f;
  if (sscanf(command, "%c %f", &key, &value) != 2) {
    Serial.println("Unknown command.");
    return;
  }

  switch (key) {
    case 'E': case 'e':
      if (static_cast<int>(value) == 1) enableMotor(); else disableMotor();
      break;
    case 'T': case 't':
      if (value >= kJointMinDeg && value <= kJointMaxDeg) {
        targetPositionDeg = value;
        integralError = 0.0f;
        Serial.println("Local target changed. A new radio command can replace it.");
      }
      break;
    case 'P': case 'p': kp = value; break;
    case 'I': case 'i': ki = value; integralError = 0.0f; break;
    case 'D': case 'd': kd = value; break;
    case 'M': case 'm':
      dacCommandLimit = constrain(static_cast<int>(value), 0, kInitialDacLimit);
      break;
    default:
      Serial.println("Unknown command.");
      break;
  }
}

void printDebugWhenDue() {
  if (millis() - lastDebugMs < kDebugPeriodMs) return;
  lastDebugMs = millis();

  Serial.print("pos="); Serial.print(debugPositionDeg, 2);
  Serial.print(" target="); Serial.print(targetPositionDeg, 2);
  Serial.print(" error="); Serial.print(debugErrorDeg, 2);
  Serial.print(" dac="); Serial.print(debugDacCommand);
  Serial.print(" jitter_us="); Serial.println(maxJitterUs);

  if (commandLogPending) {
    commandLogPending = false;
    Serial.print("Command seq "); Serial.print(lastCommandSequence);
    Serial.print(": final position = ");
    Serial.print(lastCommand.positionFinalDeg, 2);
    Serial.println(" deg");
  }
}

void printStatus() {
  Serial.print("Motor: "); Serial.println(motorEnabled ? "ENABLED" : "DISABLED");
  Serial.print("ESP-NOW: "); Serial.println(commsReady ? "READY" : "NOT READY");
  Serial.print("E-STOP: "); Serial.println(comms.estopLatched() ? "LATCHED" : "clear");
  Serial.print("Target: "); Serial.println(targetPositionDeg, 2);
  Serial.print("Max DAC command: "); Serial.println(dacCommandLimit);
}
