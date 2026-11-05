/*
  Pololu #998 motor test WITH MT6701 encoder speed feedback
  Raspberry Pi Pico 2 + Waveshare Pico-Motor-Driver

  Arduino core: Earle F. Philhower III RP2040/RP2350 core

  This is a variant of pico_motor_test.ino. The original sketch is left
  untouched for open-loop comparison; this file adds per-tick speed
  correction from two MT6701 magnetic encoders (I2C mode) so that actual
  measured wheel speed, not just elapsed time, governs the commanded duty
  cycle on each motor.

  Sequence (same time-based target profile shape as the original sketch):
    - Forward profile for 5 seconds:
        5% -> 30% over 2.5 seconds
        30% -> 3% over 2.5 seconds
    - Reverse profile for 5 seconds:
        3% -> 7%
    - Active brake and remain stopped

  Expected Waveshare Pico-Motor-Driver mapping:
    Motor 1 on C1/C2: PCA9685 channels 6 = PWM, 7 = IN1, 8 = IN2
    Motor 2 on D1/D2: PCA9685 channels 9 = PWM, 10 = IN1, 11 = IN2

  Confirm the mapping and I2C address against your board revision before
  connecting the motor.

  ENCODER WIRING
  --------------
  The MT6701 I2C device address is fixed at 0x06 (not configurable), so
  the two encoders cannot share a single I2C bus. The RP2350 has two
  independent hardware I2C peripherals, so each encoder gets its own bus:
    - Encoder 1 (Motor 1): shares the existing I2C0 bus (Wire) with the
      PCA9685 at 0x40 -- GP20 = SDA, GP21 = SCL. No new pins required.
    - Encoder 2 (Motor 2): its own I2C1 bus (Wire1) on ENCODER2_SDA_PIN /
      ENCODER2_SCL_PIN, defined below.

  Power BOTH MT6701 boards from 3.3V, not 5V. This is a physical wiring
  concern, not something firmware can enforce.

  MT6701 register map (I2C mode, per MagnTek MT6701 datasheet):
    0x03 = ANGLE_H: bits [13:6] of the 14-bit raw angle
    0x04 = ANGLE_L: bits [5:0] of the 14-bit raw angle in bits [7:2];
           bits [1:0] are status/reserved and are not used here.
    raw14 = (reg[0x03] << 6) | (reg[0x04] >> 2)
  This matches the register usage seen in third-party MT6701 I2C
  references; verify against your specific datasheet revision before
  trusting it on new hardware.
*/

#include <Wire.h>

// Pico 2 I2C0 pins used by the Waveshare motor driver AND encoder 1.
constexpr uint8_t SDA_PIN = 20;
constexpr uint8_t SCL_PIN = 21;

// Pico 2 I2C1 pins, dedicated to encoder 2 (MT6701's address is fixed,
// so it cannot share a bus with encoder 1). Change here if rewired.
constexpr uint8_t ENCODER2_SDA_PIN = 18;
constexpr uint8_t ENCODER2_SCL_PIN = 19;

constexpr uint8_t START_BUTTON_PIN = 15;
constexpr unsigned long BUTTON_DEBOUNCE_TIME_MS = 30;

// Default PCA9685 address when the Waveshare address jumpers are unchanged.
constexpr uint8_t PCA9685_ADDRESS = 0x40;

// C output group / Motor 1 PCA9685 channels.
constexpr uint8_t MOTOR_1_PWM_CHANNEL = 6;
constexpr uint8_t MOTOR_1_IN1_CHANNEL = 7;
constexpr uint8_t MOTOR_1_IN2_CHANNEL = 8;

// D output group / Motor 2 PCA9685 channels.
constexpr uint8_t MOTOR_2_PWM_CHANNEL = 9;
constexpr uint8_t MOTOR_2_IN1_CHANNEL = 10;
constexpr uint8_t MOTOR_2_IN2_CHANNEL = 11;

// The C and D groups are contiguous PCA9685 channels 6 through 11.
constexpr uint8_t MOTOR_CHANNEL_START = 6;
constexpr uint8_t MOTOR_CHANNEL_COUNT = 6;

constexpr uint16_t PCA9685_FULL_SCALE = 4095;

// The Pololu #998 is rated for 6 V, while the motor rail is 9 V.
constexpr float MOTOR_RATED_VOLTAGE = 6.0f;
constexpr float MOTOR_SUPPLY_VOLTAGE = 9.0f;

// Forward profile. The mathematical 6 V / 9 V ceiling is 66.7%;
// do not increase these values without checking current and temperature.
constexpr float START_DUTY_PERCENT = 5.0f;
constexpr float FORWARD_PEAK_DUTY_PERCENT = 30.0f;
constexpr float FORWARD_END_DUTY_PERCENT = 3.0f;

// Reverse profile.
constexpr float REVERSE_START_DUTY_PERCENT = 3.0f;
constexpr float REVERSE_END_DUTY_PERCENT = 7.0f;

constexpr unsigned long FORWARD_PROFILE_TIME_MS = 5000;
constexpr unsigned long FORWARD_ACCELERATION_TIME_MS = 2500;
constexpr unsigned long REVERSE_PROFILE_TIME_MS = 5000;
constexpr unsigned long RAMP_UPDATE_INTERVAL_MS = 50;

// Proportional gain used to correct each motor's commanded duty toward
// the two-motor average measured speed. Units: duty percent per RPM of
// error. Start conservative and tune on the bench while watching the
// telemetry below -- too high will cause the correction to overshoot.
constexpr float SPEED_MATCH_KP_PERCENT_PER_RPM = 0.05f;

// Rate limit for Serial telemetry so it doesn't flood every tick.
constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;

// PCA9685 registers.
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

// MT6701 registers (I2C mode). Address is fixed and not configurable.
constexpr uint8_t MT6701_I2C_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_H_REG = 0x03;
constexpr uint8_t MT6701_ANGLE_L_REG = 0x04;
constexpr int32_t MT6701_COUNTS_PER_REV = 16384; // 14-bit raw angle

void pcaWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t pcaReadRegister(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  if (Wire.requestFrom(PCA9685_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return 0;
  }

  return Wire.read();
}

void setFullOn(uint16_t &onValue, uint16_t &offValue) {
  onValue = 0x1000;
  offValue = 0x0000;
}

void setFullOff(uint16_t &onValue, uint16_t &offValue) {
  onValue = 0x0000;
  offValue = 0x1000;
}

void setPwm(uint16_t pwmValue, uint16_t &onValue, uint16_t &offValue) {
  if (pwmValue == 0) {
    setFullOff(onValue, offValue);
    return;
  }

  if (pwmValue >= PCA9685_FULL_SCALE) {
    setFullOn(onValue, offValue);
    return;
  }

  onValue = 0x0000;
  offValue = pwmValue;
}

// Write PCA9685 channels 6 through 11 in one I2C transaction.
void pcaWriteMotorChannels(const uint16_t onValues[MOTOR_CHANNEL_COUNT],
                           const uint16_t offValues[MOTOR_CHANNEL_COUNT]) {
  Wire.beginTransmission(PCA9685_ADDRESS);

  // Start writing at channel 6 ON_L register.
  Wire.write(
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * MOTOR_CHANNEL_START)));

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    Wire.write((uint8_t)(onValues[channel] & 0xFF));
    Wire.write((uint8_t)((onValues[channel] >> 8) & 0x1F));
    Wire.write((uint8_t)(offValues[channel] & 0xFF));
    Wire.write((uint8_t)((offValues[channel] >> 8) & 0x1F));
  }

  Wire.endTransmission();
}

void pcaSetPwmFrequency(uint16_t frequencyHz) {
  // PCA9685 nominal oscillator frequency is 25 MHz.
  float prescaleValue = (25000000.0f / (4096.0f * frequencyHz)) - 1.0f;

  if (prescaleValue < 3.0f) {
    prescaleValue = 3.0f;
  }
  if (prescaleValue > 255.0f) {
    prescaleValue = 255.0f;
  }

  const uint8_t prescale = static_cast<uint8_t>(prescaleValue);
  const uint8_t oldMode1 = pcaReadRegister(PCA9685_MODE1);

  // PCA9685 requires sleep mode while changing PRE_SCALE.
  pcaWriteRegister(PCA9685_MODE1,
                   static_cast<uint8_t>((oldMode1 & 0x7F) | 0x10));
  pcaWriteRegister(PCA9685_PRESCALE, prescale);
  pcaWriteRegister(PCA9685_MODE1, oldMode1);
  delay(1);
  pcaWriteRegister(PCA9685_MODE1, static_cast<uint8_t>(oldMode1 | 0xA0));
}

uint16_t ratedVoltagePwmLimit() {
  const float limit =
      (MOTOR_RATED_VOLTAGE / MOTOR_SUPPLY_VOLTAGE) * PCA9685_FULL_SCALE;

  if (limit < 0.0f) {
    return 0;
  }
  if (limit > PCA9685_FULL_SCALE) {
    return PCA9685_FULL_SCALE;
  }
  return static_cast<uint16_t>(limit);
}

// This is the final clamp: it is always applied last, after any
// encoder-based correction has been added to the base duty percent.
uint16_t pwmValueForPercent(float dutyPercent) {
  const float requested = (dutyPercent / 100.0f) * PCA9685_FULL_SCALE;
  const uint16_t voltageLimit = ratedVoltagePwmLimit();

  if (requested < 0.0f) {
    return 0;
  }
  if (requested > voltageLimit) {
    return voltageLimit;
  }
  return static_cast<uint16_t>(requested);
}

void motorCoastBoth() {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOff(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

// Unlike the original sketch's motorDriveBoth(), this drives each motor
// with its own PWM value (same direction for both) so the two motors can
// be corrected independently based on their own encoder feedback.
void motorDriveIndividual(bool forward, uint16_t pwm1, uint16_t pwm2) {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOff(onValues[channel], offValues[channel]);
  }

  const uint8_t motor1PwmIndex = MOTOR_1_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In1Index = MOTOR_1_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In2Index = MOTOR_1_IN2_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2PwmIndex = MOTOR_2_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In1Index = MOTOR_2_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In2Index = MOTOR_2_IN2_CHANNEL - MOTOR_CHANNEL_START;

  setPwm(pwm1, onValues[motor1PwmIndex], offValues[motor1PwmIndex]);
  setPwm(pwm2, onValues[motor2PwmIndex], offValues[motor2PwmIndex]);

  if (forward) {
    setFullOn(onValues[motor1In1Index], offValues[motor1In1Index]);
    setFullOff(onValues[motor1In2Index], offValues[motor1In2Index]);
    setFullOn(onValues[motor2In1Index], offValues[motor2In1Index]);
    setFullOff(onValues[motor2In2Index], offValues[motor2In2Index]);
  } else {
    setFullOff(onValues[motor1In1Index], offValues[motor1In1Index]);
    setFullOn(onValues[motor1In2Index], offValues[motor1In2Index]);
    setFullOff(onValues[motor2In1Index], offValues[motor2In1Index]);
    setFullOn(onValues[motor2In2Index], offValues[motor2In2Index]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

void motorActiveBrakeBoth() {
  // TB6612FNG short-brake state:
  // IN1 = HIGH, IN2 = HIGH, PWM = HIGH.
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOn(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

// ---------------------------------------------------------------------
// MT6701 encoder driver (manual register access, no external library)
// ---------------------------------------------------------------------

bool mt6701CheckPresence(TwoWire &bus) {
  bus.beginTransmission(MT6701_I2C_ADDRESS);
  return bus.endTransmission() == 0;
}

// Reads the 14-bit raw angle (0..16383). Returns false if the device did
// not ACK, in which case rawOut is left unmodified.
bool mt6701ReadAngleRaw(TwoWire &bus, uint16_t &rawOut) {
  bus.beginTransmission(MT6701_I2C_ADDRESS);
  bus.write(MT6701_ANGLE_H_REG);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  if (bus.requestFrom(MT6701_I2C_ADDRESS, static_cast<uint8_t>(2)) != 2) {
    return false;
  }

  const uint8_t angleHigh = bus.read();
  const uint8_t angleLow = bus.read();

  // raw14 = angleHigh[7:0] (bits 13:6) concatenated with angleLow[7:2]
  // (bits 5:0). angleLow[1:0] are status/reserved bits, unused here.
  rawOut = (static_cast<uint16_t>(angleHigh) << 6) |
           (static_cast<uint16_t>(angleLow) >> 2);
  return true;
}

// Tracks encoder state so RPM can be derived from consecutive readings.
struct EncoderTracker {
  bool hasLastReading = false;
  uint16_t lastRaw = 0;
  unsigned long lastUpdateMs = 0;
  float rpm = 0.0f;
};

EncoderTracker motor1Encoder; // Wire, shares the PCA9685 bus.
EncoderTracker motor2Encoder; // Wire1, dedicated bus.

// Updates tracker.rpm from a new raw angle reading, correctly handling
// the 0 <-> 16383 wraparound. Returns false (and leaves rpm unchanged)
// if this is the first reading, since there is no prior sample yet.
bool updateEncoderRpm(EncoderTracker &tracker, uint16_t rawNow,
                     unsigned long nowMs) {
  if (!tracker.hasLastReading) {
    tracker.lastRaw = rawNow;
    tracker.lastUpdateMs = nowMs;
    tracker.hasLastReading = true;
    return false;
  }

  const unsigned long dtMs = nowMs - tracker.lastUpdateMs;
  if (dtMs == 0) {
    return false;
  }

  int32_t delta = static_cast<int32_t>(rawNow) - static_cast<int32_t>(tracker.lastRaw);

  // Wraparound correction: take the shorter path around the 14-bit circle.
  if (delta > (MT6701_COUNTS_PER_REV / 2)) {
    delta -= MT6701_COUNTS_PER_REV;
  } else if (delta < -(MT6701_COUNTS_PER_REV / 2)) {
    delta += MT6701_COUNTS_PER_REV;
  }

  const float revolutions = static_cast<float>(delta) /
                            static_cast<float>(MT6701_COUNTS_PER_REV);
  tracker.rpm = revolutions * (60000.0f / static_cast<float>(dtMs));
  tracker.lastRaw = rawNow;
  tracker.lastUpdateMs = nowMs;
  return true;
}

float rawAngleToDegrees(uint16_t raw) {
  return static_cast<float>(raw) * (360.0f / static_cast<float>(MT6701_COUNTS_PER_REV));
}

// ---------------------------------------------------------------------
// Speed profile with encoder-corrected duty cycle
// ---------------------------------------------------------------------

float computeForwardBaseDutyPercent(unsigned long elapsedMs) {
  if (elapsedMs < FORWARD_ACCELERATION_TIME_MS) {
    const float progress = static_cast<float>(elapsedMs) /
                           static_cast<float>(FORWARD_ACCELERATION_TIME_MS);
    return START_DUTY_PERCENT +
           ((FORWARD_PEAK_DUTY_PERCENT - START_DUTY_PERCENT) * progress);
  }

  const unsigned long decelerationElapsed =
      elapsedMs - FORWARD_ACCELERATION_TIME_MS;
  const unsigned long decelerationTime =
      FORWARD_PROFILE_TIME_MS - FORWARD_ACCELERATION_TIME_MS;
  const float progress = static_cast<float>(decelerationElapsed) /
                         static_cast<float>(decelerationTime);
  return FORWARD_PEAK_DUTY_PERCENT +
         ((FORWARD_END_DUTY_PERCENT - FORWARD_PEAK_DUTY_PERCENT) * progress);
}

float computeReverseBaseDutyPercent(unsigned long elapsedMs) {
  const float progress = static_cast<float>(elapsedMs) /
                         static_cast<float>(REVERSE_PROFILE_TIME_MS);
  return REVERSE_START_DUTY_PERCENT +
         ((REVERSE_END_DUTY_PERCENT - REVERSE_START_DUTY_PERCENT) * progress);
}

unsigned long lastTelemetryMs = 0;

void maybePrintTelemetry(unsigned long nowMs, uint16_t raw1, uint16_t raw2,
                         float commandedDuty1, float commandedDuty2,
                         float correction1, float correction2) {
  if (nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = nowMs;

  Serial.print("M1 angle=");
  Serial.print(rawAngleToDegrees(raw1), 1);
  Serial.print(" rpm=");
  Serial.print(motor1Encoder.rpm, 1);
  Serial.print(" duty=");
  Serial.print(commandedDuty1, 2);
  Serial.print(" corr=");
  Serial.print(correction1, 2);

  Serial.print(" | M2 angle=");
  Serial.print(rawAngleToDegrees(raw2), 1);
  Serial.print(" rpm=");
  Serial.print(motor2Encoder.rpm, 1);
  Serial.print(" duty=");
  Serial.print(commandedDuty2, 2);
  Serial.print(" corr=");
  Serial.println(correction2, 2);
}

// Runs one speed profile segment (forward or reverse) for durationMs,
// using encoder feedback to correct each motor's commanded duty toward
// the two-motor average measured speed every RAMP_UPDATE_INTERVAL_MS.
// baseDutyFn supplies the time-based target duty percent (the setpoint
// shape); the encoders correct real-world deviation from it.
void runEncoderCorrectedProfile(bool forward, unsigned long durationMs,
                                float (*baseDutyFn)(unsigned long)) {
  const unsigned long profileStart = millis();

  while (true) {
    const unsigned long nowMs = millis();
    const unsigned long elapsed = nowMs - profileStart;

    if (elapsed >= durationMs) {
      break;
    }

    const float baseDuty = baseDutyFn(elapsed);

    uint16_t raw1 = motor1Encoder.lastRaw;
    uint16_t raw2 = motor2Encoder.lastRaw;
    mt6701ReadAngleRaw(Wire, raw1);
    mt6701ReadAngleRaw(Wire1, raw2);
    updateEncoderRpm(motor1Encoder, raw1, nowMs);
    updateEncoderRpm(motor2Encoder, raw2, nowMs);

    const float averageRpm = (motor1Encoder.rpm + motor2Encoder.rpm) / 2.0f;
    const float correction1 =
        SPEED_MATCH_KP_PERCENT_PER_RPM * (averageRpm - motor1Encoder.rpm);
    const float correction2 =
        SPEED_MATCH_KP_PERCENT_PER_RPM * (averageRpm - motor2Encoder.rpm);

    float commandedDuty1 = baseDuty + correction1;
    float commandedDuty2 = baseDuty + correction2;
    if (commandedDuty1 < 0.0f) commandedDuty1 = 0.0f;
    if (commandedDuty2 < 0.0f) commandedDuty2 = 0.0f;

    // pwmValueForPercent() applies the 66.7% rated-voltage ceiling; this
    // is the final clamp and is always applied after correction above.
    const uint16_t pwm1 = pwmValueForPercent(commandedDuty1);
    const uint16_t pwm2 = pwmValueForPercent(commandedDuty2);

    motorDriveIndividual(forward, pwm1, pwm2);
    maybePrintTelemetry(nowMs, raw1, raw2, commandedDuty1, commandedDuty2,
                       correction1, correction2);

    delay(RAMP_UPDATE_INTERVAL_MS);
  }

  // Ensure the final command matches the profile's end duty exactly,
  // same as the open-loop sketch, without further correction.
  const uint16_t endPwm = pwmValueForPercent(baseDutyFn(durationMs - 1));
  motorDriveIndividual(forward, endPwm, endPwm);
}

void motorForwardProfile() {
  runEncoderCorrectedProfile(true, FORWARD_PROFILE_TIME_MS,
                            computeForwardBaseDutyPercent);
}

void motorReverseProfile() {
  runEncoderCorrectedProfile(false, REVERSE_PROFILE_TIME_MS,
                            computeReverseBaseDutyPercent);
}

void runMotorTest() {
  // Fail loudly rather than silently running open-loop if either
  // encoder is not responding on its bus.
  const bool encoder1Present = mt6701CheckPresence(Wire);
  const bool encoder2Present = mt6701CheckPresence(Wire1);

  if (!encoder1Present || !encoder2Present) {
    Serial.println("ENCODER FAULT: cannot start test.");
    if (!encoder1Present) {
      Serial.println("  Motor 1 MT6701 (Wire, GP20/GP21) did not ACK.");
    }
    if (!encoder2Present) {
      Serial.println("  Motor 2 MT6701 (Wire1, ENCODER2_SDA_PIN/"
                     "ENCODER2_SCL_PIN) did not ACK.");
    }
    motorActiveBrakeBoth();
    return;
  }

  // Reset tracking so the first tick of each segment doesn't compute a
  // bogus RPM from a stale reading.
  motor1Encoder.hasLastReading = false;
  motor2Encoder.hasLastReading = false;

  Serial.println("Forward: 5% to 30%, then down to 3% over 5 seconds...");
  motorForwardProfile();

  motor1Encoder.hasLastReading = false;
  motor2Encoder.hasLastReading = false;

  Serial.println("Reverse: accelerating from 3% to 7% over 5 seconds...");
  motorReverseProfile();

  // Brake immediately at the end of the reverse interval.
  motorActiveBrakeBoth();
  Serial.println("Motor test complete; active brake applied.");
}

bool startButtonWasPressed() {
  if (digitalRead(START_BUTTON_PIN) != LOW) {
    return false;
  }

  delay(BUTTON_DEBOUNCE_TIME_MS);
  return digitalRead(START_BUTTON_PIN) == LOW;
}

void setup() {
  Serial.begin(115200);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // Philhower Arduino core supports selecting I2C pins before Wire.begin().
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  // Encoder 2 gets its own I2C1 bus since the MT6701 address is fixed.
  Wire1.setSDA(ENCODER2_SDA_PIN);
  Wire1.setSCL(ENCODER2_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  // Configure the PCA9685 in a safe inactive state first.
  pcaWriteRegister(PCA9685_MODE1, 0x00);
  pcaWriteRegister(PCA9685_MODE2, 0x04); // totem-pole outputs
  motorCoastBoth();
  pcaSetPwmFrequency(1000);

  Serial.println("Ready. Press the momentary start button on GP15.");
}

void loop() {
  if (!startButtonWasPressed()) {
    // Keep both motors stopped while waiting for a start command.
    motorActiveBrakeBoth();
    delay(50);
    return;
  }

  Serial.println("Start button pressed.");
  runMotorTest();

  // Do not retrigger while the button is still held down.
  while (digitalRead(START_BUTTON_PIN) == LOW) {
    delay(10);
  }

  delay(BUTTON_DEBOUNCE_TIME_MS);
}
