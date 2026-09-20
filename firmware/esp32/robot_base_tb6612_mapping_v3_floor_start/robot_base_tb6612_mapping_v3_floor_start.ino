/*
  Robot base firmware for ESP32 WROOM DevKit 38-pin (mapping v3 floor start):
    ESP32 + TB6612FNG + two x2 quadrature encoders + MPU6050 + ODROID-C4

  UART2 protocol at 115200 baud:
    ODROID pin 32 TX -> ESP32 GPIO34 RX2
    ODROID pin 26 RX <- ESP32 GPIO23 TX2
    ODROID pin 6 GND -> ESP32 GND

    Host -> CMD,<sequence>,<left_rpm>,<right_rpm>  (repeat <= 300 ms)
            STOP
            CLEAR
            PING
    ESP  -> one fixed 44-byte little-endian telemetry frame with magic 0xA55A
            and CRC-16/CCITT-FALSE. Startup/diagnostic replies remain ASCII.

  Safety:
    - PWM is zero at boot.
    - Missing commands for 300 ms stops both motors.
    - Invalid IMU frames and encoder stalls latch a motor-stop fault.
    - CLEAR releases a latched fault only while commanded speed is zero.
*/
#include <Arduino.h>
#include <Wire.h>
#include "DirectionMonitor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Confirmed wiring.
constexpr int LEFT_A = 25;
constexpr int LEFT_B = 26;
constexpr int RIGHT_A = 35;
constexpr int RIGHT_B = 33;
constexpr int LEFT_ENA = 14;
constexpr int LEFT_IN1 = 18;
constexpr int LEFT_IN2 = 19;
constexpr int RIGHT_ENB = 27;
constexpr int RIGHT_IN3 = 16;
constexpr int RIGHT_IN4 = 17;
constexpr int MOTOR_STBY = 32;
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

// Keep UART0 (GPIO1/GPIO3) free for USB flashing and Arduino Serial Monitor.
// The on-board USB-to-serial bridge is electrically connected to UART0 and can
// contend with an external ODROID TX signal.  Use the independent UART2 link.
constexpr int ROBOT_UART_RX = 34;  // ODROID pin 32 UART-C TX -> ESP32 GPIO34
constexpr int ROBOT_UART_TX = 23;  // ODROID pin 26 UART-C RX <- ESP32 GPIO23
HardwareSerial RobotSerial(2);

constexpr float LEFT_CPR = 2362.4f;   // mark-aligned x2 count/rev
constexpr float RIGHT_CPR = 2367.0f;
constexpr int LEFT_ENCODER_SIGN = -1;
constexpr int RIGHT_ENCODER_SIGN = 1;

constexpr float KP = 0.8f;
constexpr float KI = 1.0f;
constexpr float LEFT_FF_AT_80 = 160.0f;
constexpr float RIGHT_FF_AT_80 = 167.0f;
constexpr float INTEGRAL_LIMIT = 60.0f;
constexpr int MAX_PWM = 240;
constexpr int PWM_STEP = 20;
constexpr float MAX_COMMAND_RPM = 90.0f;

// Static friction on the completed robot is much higher than on the bench.
// Give each wheel a short, bounded kick when starting or reversing, then keep
// enough PWM available for low-speed motion.  The right drivetrain required a
// little more PWM in repeated 40 RPM floor tests.
constexpr int LEFT_MIN_RUNNING_PWM = 72;
constexpr int RIGHT_MIN_RUNNING_PWM = 82;
// A fixed-PWM floor test showed that both drivetrains move reliably around
// PWM 120, but the caster can add a short load spike after a stop or reversal.
// Use a bounded kick, then leave the existing PI loop in control as soon as
// encoder motion is measured.
constexpr int LEFT_START_PWM = 170;
constexpr int RIGHT_START_PWM = 180;
constexpr uint32_t START_BOOST_MS = 700;
constexpr float START_DETECTED_MIN_RPM = 12.0f;
constexpr float START_DETECTED_MAX_RPM = 25.0f;
constexpr float START_DETECTED_TARGET_FRACTION = 0.70f;
// One bounded re-kick can recover a wheel that momentarily loses momentum
// under caster load. A second sustained loss still latches the stall fault.
constexpr uint32_t RECOVERY_TRIGGER_MS = 350;
constexpr uint32_t RECOVERY_BOOST_MS = 500;
constexpr uint8_t MAX_RECOVERY_ATTEMPTS = 1;

constexpr uint32_t CONTROL_MS = 100;
constexpr uint32_t IMU_MS = 10;
// 20 Hz leaves generous UART headroom while remaining fast enough for EKF.
constexpr uint32_t TELEMETRY_MS = 50;
constexpr uint32_t COMMAND_WATCHDOG_MS = 300;
constexpr uint32_t ENCODER_STALL_MS = 600;
constexpr uint32_t STALL_GRACE_MS = 1200;
constexpr uint8_t IMU_FAILURE_LIMIT = 10;

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_IMU = 1,
  FAULT_LEFT_ENCODER = 2,
  FAULT_RIGHT_ENCODER = 3,
  FAULT_ENCODER_DIRECTION = 4,
};

struct EncoderSample {
  int32_t left;
  int32_t right;
  uint32_t la;
  uint32_t lb;
  uint32_t ra;
  uint32_t rb;
};

struct WheelController {
  float rpm;
  float integral;
  int pwm;
  int direction;
  uint32_t boostUntilMs;
  uint8_t recoveryAttempts;
};

struct ImuRaw {
  int16_t ax, ay, az, temp, gx, gy, gz;
};

struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
};

// Compact enough to fit in the ESP32 UART TX ring as one complete write.
// Physical units: RPM x100, acceleration mg, gyro deg/s x100, temperature C x100.
struct __attribute__((packed)) TelemetryFrame {
  uint16_t magic;
  uint8_t version;
  uint8_t length;
  uint32_t sequence;
  uint32_t milliseconds;
  int32_t leftCount;
  int32_t rightCount;
  int16_t leftRpmCenti;
  int16_t rightRpmCenti;
  int16_t axMilliG;
  int16_t ayMilliG;
  int16_t azMilliG;
  int16_t gxCentiDps;
  int16_t gyCentiDps;
  int16_t gzCentiDps;
  int16_t temperatureCenti;
  uint16_t flags;
  uint8_t fault;
  uint8_t reserved;
  uint16_t crc;
};

static_assert(sizeof(TelemetryFrame) == 44, "Unexpected telemetry layout");

EncoderSample readEncoders();
void stopMotors();
void latchFault(FaultCode code, const char *reason);
bool sampleImu(ImuData &out);

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftRaw = 0;
volatile int32_t rightRaw = 0;
volatile uint32_t leftAEdges = 0, leftBEdges = 0;
volatile uint32_t rightAEdges = 0, rightBEdges = 0;

WheelController leftWheel = {};
WheelController rightWheel = {};
EncoderSample previousEncoder = {};
ImuData imu = {};
TelemetryFrame telemetryFrame = {};

uint8_t imuAddress = 0;
float gyroBiasRaw[3] = {};
bool imuReady = false;
bool pwmReady = false;
bool commandSeen = false;
bool watchdogTripped = false;
uint8_t consecutiveImuFailures = 0;
FaultCode faultCode = FAULT_NONE;
DirectionMonitor leftDirectionMonitor, rightDirectionMonitor;

float requestedLeftRpm = 0;
float requestedRightRpm = 0;
uint32_t lastCommandMs = 0;
// Stall grace is wheel-local.  A differential-drive turn can start or reverse
// only one wheel, so a single chassis-wide timer can incorrectly latch a stall.
uint32_t leftMotionCommandMs = 0;
uint32_t rightMotionCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastImuMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastLeftEdgeMs = 0;
uint32_t lastRightEdgeMs = 0;
uint32_t telemetrySequence = 0;

char inputLine[96];
size_t inputLength = 0;
bool inputOverflow = false;

void IRAM_ATTR leftAISR() {
  const int step = digitalRead(LEFT_A) == digitalRead(LEFT_B) ? 1 : -1;
  portENTER_CRITICAL_ISR(&encoderMux);
  leftRaw += step;
  leftAEdges++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR leftBISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  leftBEdges++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightAISR() {
  const int step = digitalRead(RIGHT_A) == digitalRead(RIGHT_B) ? 1 : -1;
  portENTER_CRITICAL_ISR(&encoderMux);
  rightRaw += step;
  rightAEdges++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightBISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  rightBEdges++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

EncoderSample readEncoders() {
  EncoderSample value;
  portENTER_CRITICAL(&encoderMux);
  value = {leftRaw, rightRaw, leftAEdges, leftBEdges,
           rightAEdges, rightBEdges};
  portEXIT_CRITICAL(&encoderMux);
  value.left *= LEFT_ENCODER_SIGN;
  value.right *= RIGHT_ENCODER_SIGN;
  return value;
}

void setWheelPins(bool left, int direction) {
  if (left) {
    digitalWrite(LEFT_IN1, direction > 0 ? HIGH : LOW);
    digitalWrite(LEFT_IN2, direction > 0 ? LOW : HIGH);
  } else {
    digitalWrite(RIGHT_IN3, direction > 0 ? LOW : HIGH);
    digitalWrite(RIGHT_IN4, direction > 0 ? HIGH : LOW);
  }
}

void stopMotors() {
  leftDirectionMonitor = {};
  rightDirectionMonitor = {};
  if (pwmReady) {
    ledcWrite(LEFT_ENA, 0);
    ledcWrite(RIGHT_ENB, 0);
  }
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, LOW);
  // TB6612FNG hardware standby is the final, independent motor cutoff.
  digitalWrite(MOTOR_STBY, LOW);
  leftWheel.pwm = rightWheel.pwm = 0;
  leftWheel.integral = rightWheel.integral = 0;
  leftWheel.direction = rightWheel.direction = 0;
  leftWheel.boostUntilMs = rightWheel.boostUntilMs = 0;
  leftWheel.recoveryAttempts = rightWheel.recoveryAttempts = 0;
}

void latchFault(FaultCode code, const char *reason) {
  if (faultCode != FAULT_NONE) return;
  faultCode = code;
  requestedLeftRpm = requestedRightRpm = 0;
  stopMotors();
  RobotSerial.printf("FAULT,%u,%s\n", unsigned(code), reason);
}

bool readImuBytes(uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(int(imuAddress), int(length), int(true)) != length)
    return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readImuRaw(ImuRaw &raw) {
  uint8_t data[14];
  if (!readImuBytes(0x3B, data, sizeof(data))) return false;
  bool allZero = true, allFF = true;
  for (uint8_t value : data) {
    allZero &= value == 0;
    allFF &= value == 0xFF;
  }
  if (allZero || allFF) return false;
  int16_t *values[] = {&raw.ax, &raw.ay, &raw.az, &raw.temp,
                       &raw.gx, &raw.gy, &raw.gz};
  for (int i = 0; i < 7; ++i)
    *values[i] = int16_t((uint16_t(data[2 * i]) << 8) | data[2 * i + 1]);
  return true;
}

bool sampleImu(ImuData &out) {
  ImuRaw raw;
  if (!readImuRaw(raw)) return false;
  out.ax = raw.ax / 16384.0f;
  out.ay = raw.ay / 16384.0f;
  out.az = raw.az / 16384.0f;
  const float magnitude = sqrtf(out.ax * out.ax + out.ay * out.ay +
                                out.az * out.az);
  if (magnitude < 0.2f || magnitude > 4.0f) return false;
  out.gx = (raw.gx - gyroBiasRaw[0]) / 131.0f;
  out.gy = (raw.gy - gyroBiasRaw[1]) / 131.0f;
  out.gz = (raw.gz - gyroBiasRaw[2]) / 131.0f;
  out.temp = raw.temp / 340.0f + 36.53f;
  return true;
}

bool setupImu() {
  Wire.begin(SDA_PIN, SCL_PIN);
  // This physical MPU6050 and wiring were validated at 400 kHz by the
  // standalone test sketch. Keep the production initialization equivalent.
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  for (uint8_t candidate : {uint8_t(0x68), uint8_t(0x69)}) {
    imuAddress = candidate;
    Wire.beginTransmission(candidate);
    if (Wire.endTransmission() == 0) break;
    imuAddress = 0;
  }
  if (!imuAddress) return false;

  // Wake directly instead of issuing a software reset. Some MPU6050 clones
  // acknowledge immediately after reset but are not yet ready for the next
  // register access, which caused a false startup fault on this unit.
  if (!writeImuRegister(0x6B, 0x01)) return false;
  delay(100);
  if (!writeImuRegister(0x1A, 0x03) ||   // DLPF approximately 44 Hz
      !writeImuRegister(0x19, 0x09) ||   // 100 Hz sample rate
      !writeImuRegister(0x1B, 0x00) ||   // gyro +/-250 dps
      !writeImuRegister(0x1C, 0x00))     // accel +/-2 g
    return false;

  uint8_t who = 0;
  if (!readImuBytes(0x75, &who, 1)) return false;
  RobotSerial.printf("IMU,ADDRESS,0x%02X,WHO_AM_I,0x%02X\n",
                imuAddress, who);
  // Known MPU6050-compatible modules can report a clone-specific WHO_AM_I.
  // An unusual identity is diagnostic information, not a reason to reject a
  // device that acknowledged and successfully returned sensor registers.
  if (who != 0x68 && who != 0x69)
    RobotSerial.printf("IMU,WARN,UNUSUAL_WHO_AM_I,0x%02X\n", who);
  delay(100);

  RobotSerial.println("CALIBRATING,KEEP_STILL,3_SECONDS");
  int64_t sums[3] = {};
  constexpr int sampleAttempts = 1000;
  int validSamples = 0;
  for (int n = 0; n < sampleAttempts; ++n) {
    ImuRaw raw;
    if (!readImuRaw(raw)) {
      delay(2);
      continue;
    }
    const int16_t values[3] = {raw.gx, raw.gy, raw.gz};
    for (int axis = 0; axis < 3; ++axis) {
      sums[axis] += values[axis];
    }
    validSamples++;
    delay(2);
  }
  if (validSamples < 950) return false;
  for (int axis = 0; axis < 3; ++axis) {
    gyroBiasRaw[axis] = float(sums[axis]) / validSamples;
  }
  RobotSerial.printf("IMU,CALIBRATED,%d,BIAS,%.2f,%.2f,%.2f\n",
                validSamples, gyroBiasRaw[0], gyroBiasRaw[1],
                gyroBiasRaw[2]);
  // Do not fail startup for one transient read after calibration.
  for (int attempt = 0; attempt < 20; ++attempt) {
    if (sampleImu(imu)) return true;
    delay(5);
  }
  return false;
}

void updateWheel(WheelController &wheel, bool left, float target,
                 float feedForwardAt80, float dt) {
  if (fabsf(target) < 0.5f) {
    if (left) ledcWrite(LEFT_ENA, 0); else ledcWrite(RIGHT_ENB, 0);
    wheel.pwm = 0;
    wheel.integral = 0;
    wheel.direction = 0;
    wheel.boostUntilMs = 0;
    return;
  }
  const int desiredDirection = target > 0 ? 1 : -1;
  const bool starting = wheel.direction == 0;
  const bool reversing = wheel.direction != 0 &&
                         wheel.direction != desiredDirection;
  if (reversing) {
    if (left) ledcWrite(LEFT_ENA, 0); else ledcWrite(RIGHT_ENB, 0);
    wheel.pwm = 0;
    wheel.integral = 0;
  }
  if (starting || reversing) wheel.boostUntilMs = millis() + START_BOOST_MS;
  if (starting || reversing) wheel.recoveryAttempts = 0;
  wheel.direction = desiredDirection;
  setWheelPins(left, desiredDirection);

  const float targetMagnitude = fabsf(target);
  const float measuredMagnitude = desiredDirection * wheel.rpm;
  const float error = targetMagnitude - measuredMagnitude;
  wheel.integral = constrain(wheel.integral + KI * error * dt,
                             -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  const float feedForward = feedForwardAt80 * targetMagnitude / 80.0f;
  int requested = int(constrain(feedForward + KP * error + wheel.integral,
                                0.0f, float(MAX_PWM)) + 0.5f);
  const int minimumRunningPwm = left ? LEFT_MIN_RUNNING_PWM
                                     : RIGHT_MIN_RUNNING_PWM;
  const int startPwm = left ? LEFT_START_PWM : RIGHT_START_PWM;
  requested = max(requested, minimumRunningPwm);

  // Bypass the normal slew-up only during the short starting window.  Slew
  // limiting remains active afterwards and when reducing PWM.
  bool boosting = int32_t(wheel.boostUntilMs - millis()) > 0;
  const float startDetectedRpm = constrain(
      targetMagnitude * START_DETECTED_TARGET_FRACTION,
      START_DETECTED_MIN_RPM, START_DETECTED_MAX_RPM);
  if (boosting && measuredMagnitude >= startDetectedRpm) {
    wheel.boostUntilMs = 0;
    boosting = false;
  }
  if (boosting) {
    requested = max(requested, startPwm);
  } else {
    requested = constrain(requested, wheel.pwm - PWM_STEP,
                          wheel.pwm + PWM_STEP);
  }
  wheel.pwm = constrain(requested, 0, MAX_PWM);
  if (left) ledcWrite(LEFT_ENA, wheel.pwm);
  else ledcWrite(RIGHT_ENB, wheel.pwm);
}

void updateControl() {
  const uint32_t now = millis();
  if (now - lastControlMs < CONTROL_MS) return;
  const uint32_t nowUs = micros();
  const float dt = float(nowUs - lastControlUs) / 1000000.0f;
  lastControlMs = now;
  lastControlUs = nowUs;
  if (dt <= 0) return;

  if (commandSeen && now - lastCommandMs > COMMAND_WATCHDOG_MS) {
    watchdogTripped = true;
    requestedLeftRpm = requestedRightRpm = 0;
    stopMotors();
  }

  const EncoderSample current = readEncoders();
  leftWheel.rpm = float(current.left - previousEncoder.left) * 60.0f /
                  (LEFT_CPR * dt);
  rightWheel.rpm = float(current.right - previousEncoder.right) * 60.0f /
                   (RIGHT_CPR * dt);
  if (current.la != previousEncoder.la || current.lb != previousEncoder.lb)
    lastLeftEdgeMs = now;
  if (current.ra != previousEncoder.ra || current.rb != previousEncoder.rb)
    lastRightEdgeMs = now;
  previousEncoder = current;

  if (faultCode != FAULT_NONE || !imuReady) {
    stopMotors();
    return;
  }

  if (fabsf(requestedLeftRpm) < 0.5f &&
      fabsf(requestedRightRpm) < 0.5f) {
    stopMotors();
    return;
  }

  digitalWrite(MOTOR_STBY, HIGH);

  const bool leftMoving = fabsf(requestedLeftRpm) >= 20.0f;
  const bool rightMoving = fabsf(requestedRightRpm) >= 20.0f;
  // Recover once from a short loss of wheel momentum before declaring a hard
  // stall. Resetting the wheel-local grace timestamp keeps the retry bounded;
  // if encoder edges still do not return, the normal fault path below wins.
  if (leftMoving &&
      now - leftMotionCommandMs > START_BOOST_MS &&
      now - lastLeftEdgeMs > RECOVERY_TRIGGER_MS &&
      leftWheel.recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
    leftWheel.recoveryAttempts++;
    leftWheel.integral = 0;
    leftWheel.boostUntilMs = now + RECOVERY_BOOST_MS;
    leftMotionCommandMs = now;
  }
  if (rightMoving &&
      now - rightMotionCommandMs > START_BOOST_MS &&
      now - lastRightEdgeMs > RECOVERY_TRIGGER_MS &&
      rightWheel.recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
    rightWheel.recoveryAttempts++;
    rightWheel.integral = 0;
    rightWheel.boostUntilMs = now + RECOVERY_BOOST_MS;
    rightMotionCommandMs = now;
  }
  if (leftMoving && now - leftMotionCommandMs > STALL_GRACE_MS) {
    if (now - lastLeftEdgeMs > ENCODER_STALL_MS)
      return latchFault(FAULT_LEFT_ENCODER, "LEFT_ENCODER_STALL");
  }
  if (rightMoving && now - rightMotionCommandMs > STALL_GRACE_MS) {
    if (now - lastRightEdgeMs > ENCODER_STALL_MS)
      return latchFault(FAULT_RIGHT_ENCODER, "RIGHT_ENCODER_STALL");
  }
  if (leftDirectionMonitor.check(now, requestedLeftRpm, leftWheel.rpm) ||
      rightDirectionMonitor.check(now, requestedRightRpm, rightWheel.rpm))
    return latchFault(FAULT_ENCODER_DIRECTION, "ENCODER_DIRECTION");

  updateWheel(leftWheel, true, requestedLeftRpm, LEFT_FF_AT_80, dt);
  updateWheel(rightWheel, false, requestedRightRpm, RIGHT_FF_AT_80, dt);
}

bool parseFloat(const char *text, float &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == 0 && isfinite(value);
}

void handleLine() {
  if (!strcmp(inputLine, "STOP")) {
    requestedLeftRpm = requestedRightRpm = 0;
    stopMotors();
    watchdogTripped = false;
    RobotSerial.println("ACK,STOP");
    return;
  }
  if (!strcmp(inputLine, "PING")) {
    RobotSerial.printf("PONG,%lu\n", (unsigned long)millis());
    return;
  }
  if (!strcmp(inputLine, "CLEAR")) {
    if (fabsf(requestedLeftRpm) < 0.5f &&
        fabsf(requestedRightRpm) < 0.5f && imuReady) {
      faultCode = FAULT_NONE;
      RobotSerial.println("ACK,CLEAR");
    } else {
      RobotSerial.println("NACK,CLEAR");
    }
    return;
  }

  char *save = nullptr;
  char *kind = strtok_r(inputLine, ",", &save);
  char *sequence = strtok_r(nullptr, ",", &save);
  char *leftText = strtok_r(nullptr, ",", &save);
  char *rightText = strtok_r(nullptr, ",", &save);
  char *extra = strtok_r(nullptr, ",", &save);
  float left = 0, right = 0;
  if (!kind || strcmp(kind, "CMD") || !sequence || !leftText ||
      !rightText || extra || !parseFloat(leftText, left) ||
      !parseFloat(rightText, right) ||
      fabsf(left) > MAX_COMMAND_RPM || fabsf(right) > MAX_COMMAND_RPM) {
    RobotSerial.println("NACK,INVALID_COMMAND");
    return;
  }
  char *sequenceEnd = nullptr;
  strtoul(sequence, &sequenceEnd, 10);
  if (sequenceEnd == sequence || *sequenceEnd) {
    RobotSerial.println("NACK,INVALID_SEQUENCE");
    return;
  }
  if (faultCode != FAULT_NONE) {
    RobotSerial.printf("NACK,FAULT,%u\n", unsigned(faultCode));
    return;
  }
  const float oldLeft = requestedLeftRpm;
  const float oldRight = requestedRightRpm;
  requestedLeftRpm = left;
  requestedRightRpm = right;
  lastCommandMs = millis();
  commandSeen = true;
  watchdogTripped = false;
  const bool leftStartedOrReversed = fabsf(left) >= 0.5f &&
      (fabsf(oldLeft) < 0.5f || (oldLeft > 0.0f) != (left > 0.0f));
  const bool rightStartedOrReversed = fabsf(right) >= 0.5f &&
      (fabsf(oldRight) < 0.5f || (oldRight > 0.0f) != (right > 0.0f));
  if (leftStartedOrReversed) {
    leftMotionCommandMs = lastCommandMs;
    lastLeftEdgeMs = lastCommandMs;
  }
  if (rightStartedOrReversed) {
    rightMotionCommandMs = lastCommandMs;
    lastRightEdgeMs = lastCommandMs;
  }
}

void readSerial() {
  while (RobotSerial.available()) {
    const char c = RobotSerial.read();
    if (c == '\r' || c == '\n') {
      if (inputOverflow) RobotSerial.println("NACK,LINE_TOO_LONG");
      else if (inputLength) {
        inputLine[inputLength] = 0;
        handleLine();
      }
      inputLength = 0;
      inputOverflow = false;
    } else if (!inputOverflow) {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
      else inputOverflow = true;
    }
  }
}

void updateImu() {
  const uint32_t now = millis();
  if (!imuReady || now - lastImuMs < IMU_MS) return;
  lastImuMs = now;
  if (sampleImu(imu)) {
    consecutiveImuFailures = 0;
    return;
  }
  if (consecutiveImuFailures < IMU_FAILURE_LIMIT)
    consecutiveImuFailures++;
  if (consecutiveImuFailures >= IMU_FAILURE_LIMIT)
    latchFault(FAULT_IMU, "IMU_READ_REPEATED");
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021)
                           : uint16_t(crc << 1);
  }
  return crc;
}

int16_t scaledInt16(float value, float scale) {
  const float scaled = roundf(value * scale);
  return int16_t(constrain(scaled, -32768.0f, 32767.0f));
}

void publishTelemetry() {
  const uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_MS) return;
  lastTelemetryMs = now;
  // Never let telemetry block the control loop or starve the ESP32 watchdog.
  const EncoderSample encoder = readEncoders();
  uint16_t flags = 0;
  if (imuReady) flags |= 1;
  if (commandSeen && now - lastCommandMs <= COMMAND_WATCHDOG_MS) flags |= 2;
  if (leftWheel.pwm || rightWheel.pwm) flags |= 4;
  if (watchdogTripped) flags |= 8;
  if (faultCode != FAULT_NONE) flags |= 16;
  // Keep the TX storage persistent. The ESP32 UART driver can finish sending
  // after RobotSerial.write() returns, so a stack buffer is unsafe here.
  TelemetryFrame &frame = telemetryFrame;
  memset(&frame, 0, sizeof(frame));
  frame.magic = 0xA55A;
  frame.version = 1;
  frame.length = sizeof(frame);
  frame.sequence = telemetrySequence++;
  frame.milliseconds = now;
  frame.leftCount = encoder.left;
  frame.rightCount = encoder.right;
  frame.leftRpmCenti = scaledInt16(leftWheel.rpm, 100.0f);
  frame.rightRpmCenti = scaledInt16(rightWheel.rpm, 100.0f);
  frame.axMilliG = scaledInt16(imu.ax, 1000.0f);
  frame.ayMilliG = scaledInt16(imu.ay, 1000.0f);
  frame.azMilliG = scaledInt16(imu.az, 1000.0f);
  frame.gxCentiDps = scaledInt16(imu.gx, 100.0f);
  frame.gyCentiDps = scaledInt16(imu.gy, 100.0f);
  frame.gzCentiDps = scaledInt16(imu.gz, 100.0f);
  frame.temperatureCenti = scaledInt16(imu.temp, 100.0f);
  frame.flags = flags;
  frame.fault = uint8_t(faultCode);
  frame.crc = crc16Ccitt(reinterpret_cast<const uint8_t *>(&frame),
                        sizeof(frame) - sizeof(frame.crc));
  if (RobotSerial.availableForWrite() < int(sizeof(frame))) return;
  RobotSerial.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

void setup() {
  const int outputPins[] = {LEFT_ENA, LEFT_IN1, LEFT_IN2,
                            RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, MOTOR_STBY};
  for (int pin : outputPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  RobotSerial.begin(115200, SERIAL_8N1, ROBOT_UART_RX, ROBOT_UART_TX);
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT);  // GPIO35 has no internal pull-up.
  pinMode(RIGHT_B, INPUT_PULLUP);
  pwmReady = ledcAttach(LEFT_ENA, 1000, 8) &&
             ledcAttach(RIGHT_ENB, 1000, 8);
  stopMotors();
  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), leftBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), rightBISR, CHANGE);

  imuReady = setupImu();
  if (!imuReady) latchFault(FAULT_IMU, "IMU_SETUP_OR_CALIBRATION");
  previousEncoder = readEncoders();
  const uint32_t now = millis();
  lastControlMs = lastImuMs = lastTelemetryMs = now;
  lastControlUs = micros();
  lastLeftEdgeMs = lastRightEdgeMs = now;
  RobotSerial.printf("HELLO,1,%s,0x%02X\n",
                imuReady && pwmReady ? "READY" : "FAULT", imuAddress);
}

void loop() {
  readSerial();
  updateImu();
  updateControl();
  publishTelemetry();
  // Let the ESP32 idle task service the hardware/task watchdog even when the
  // serial link is continuously busy in both directions.
  delay(1);
}
