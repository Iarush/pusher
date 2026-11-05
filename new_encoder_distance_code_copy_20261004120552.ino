/*
  Pololu #998 motor test
  Raspberry Pi Pico 2 + Waveshare Pico-Motor-Driver

  Arduino core: Earle F. Philhower III RP2040/RP2350 core

  Sequence:
    - Drive forward until the encoder-measured distance is reached
    - Active brake and remain stopped

  Expected Waveshare Pico-Motor-Driver mapping:
    Motor 1 on C1/C2: PCA9685 channels 6 = PWM, 7 = IN1, 8 = IN2
    Motor 2 on D1/D2: PCA9685 channels 9 = PWM, 10 = IN1, 11 = IN2

  Confirm the mapping and I2C address against your board revision before
  connecting the motor.
*/

#include <Wire.h>

// Pico 2 I2C0 pins used by the Waveshare motor driver.
constexpr uint8_t SDA_PIN = 20;
constexpr uint8_t SCL_PIN = 21;
constexpr uint8_t START_BUTTON_PIN = 15;
constexpr unsigned long BUTTON_DEBOUNCE_TIME_MS = 30;

// Connect each MT6701's ABI quadrature outputs to these 3.3 V GPIO pins.
constexpr uint8_t LEFT_ENCODER_A_PIN = 2;
constexpr uint8_t LEFT_ENCODER_B_PIN = 3;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 4;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 5;

// Calibrate these for the actual wheel and MT6701 ABI configuration.
// Counts/rev means A-channel edges counted on both edges, per wheel revolution,
// including any motor gearbox ratio if the encoder is mounted on the motor.
constexpr float TARGET_DISTANCE_MM = 9000.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 60.325f;
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 16384.0f;
constexpr float DISTANCE_SLOWDOWN_MM = 200.0f;
constexpr unsigned long DISTANCE_MOVE_TIMEOUT_MS = 30000;

volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;

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

// PCA9685 registers.
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

void leftEncoderISR() {
  const bool channelA = digitalRead(LEFT_ENCODER_A_PIN);
  const bool channelB = digitalRead(LEFT_ENCODER_B_PIN);
  leftEncoderCount += (channelA == channelB) ? 1 : -1;
}

void rightEncoderISR() {
  const bool channelA = digitalRead(RIGHT_ENCODER_A_PIN);
  const bool channelB = digitalRead(RIGHT_ENCODER_B_PIN);
  rightEncoderCount += (channelA == channelB) ? 1 : -1;
}

int32_t readEncoderCount(volatile int32_t &encoderCount) {
  noInterrupts();
  const int32_t count = encoderCount;
  interrupts();
  return count;
}

int32_t absoluteCount(int32_t count) {
  return count < 0 ? -count : count;
}

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
// This keeps both motors on the same PWM value and minimizes timing
// differences when starting, reversing, or braking them.
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

void motorDriveBoth(bool forward, uint16_t pwmValue) {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  // Both motors receive the same PWM and direction values in one transaction.
  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOff(onValues[channel], offValues[channel]);
  }

  const uint8_t motor1PwmIndex = MOTOR_1_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In1Index = MOTOR_1_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In2Index = MOTOR_1_IN2_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2PwmIndex = MOTOR_2_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In1Index = MOTOR_2_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In2Index = MOTOR_2_IN2_CHANNEL - MOTOR_CHANNEL_START;

  setPwm(pwmValue, onValues[motor1PwmIndex], offValues[motor1PwmIndex]);
  setPwm(pwmValue, onValues[motor2PwmIndex], offValues[motor2PwmIndex]);

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

  // Apply active brake to both motors in one I2C transaction.
  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOn(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

bool runDistanceMove() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();

  const uint16_t startPwm = pwmValueForPercent(START_DUTY_PERCENT);
  const uint16_t peakPwm = pwmValueForPercent(FORWARD_PEAK_DUTY_PERCENT);
  const unsigned long moveStart = millis();
  float travelledMm = 0.0f;

  Serial.print("Driving to ");
  Serial.print(TARGET_DISTANCE_MM);
  Serial.println(" mm...");

  while (travelledMm < TARGET_DISTANCE_MM &&
         millis() - moveStart < DISTANCE_MOVE_TIMEOUT_MS) {
    const int32_t leftCount = absoluteCount(readEncoderCount(leftEncoderCount));
    const int32_t rightCount = absoluteCount(readEncoderCount(rightEncoderCount));
    const float averageCounts =
        (static_cast<float>(leftCount) + static_cast<float>(rightCount)) * 0.5f;
    travelledMm = averageCounts * WHEEL_CIRCUMFERENCE_MM /
                  ENCODER_COUNTS_PER_WHEEL_REV;

    float dutyPercent = FORWARD_PEAK_DUTY_PERCENT;
    const float remainingMm = TARGET_DISTANCE_MM - travelledMm;
    if (remainingMm < DISTANCE_SLOWDOWN_MM) {
      const float progress = remainingMm / DISTANCE_SLOWDOWN_MM;
      dutyPercent = START_DUTY_PERCENT +
                    ((FORWARD_PEAK_DUTY_PERCENT - START_DUTY_PERCENT) *
                     progress);
    }

    motorDriveBoth(true, pwmValueForPercent(dutyPercent));
    delay(10);
  }

  motorActiveBrakeBoth();
  if (travelledMm >= TARGET_DISTANCE_MM) {
    Serial.print("Target reached at approximately ");
    Serial.print(travelledMm);
    Serial.println(" mm; active brake applied.");
    return true;
  }

  Serial.print("Distance move timed out at approximately ");
  Serial.print(travelledMm);
  Serial.println(" mm; active brake applied.");
  return false;
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
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), rightEncoderISR,
                  CHANGE);

  // Philhower Arduino core supports selecting I2C pins before Wire.begin().
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  // Configure the PCA9685 in a safe inactive state first.
  pcaWriteRegister(PCA9685_MODE1, 0x00);
  pcaWriteRegister(PCA9685_MODE2, 0x04); // totem-pole outputs
  motorCoastBoth();
  pcaSetPwmFrequency(1000);

  Serial.println("Ready. Press the start button on GP15 to move by distance.");
}

void loop() {
  if (!startButtonWasPressed()) {
    // Keep both motors stopped while waiting for a start command.
    motorActiveBrakeBoth();
    delay(50);
    return;
  }

  Serial.println("Start button pressed.");
  runDistanceMove();

  // Do not retrigger while the button is still held down.
  while (digitalRead(START_BUTTON_PIN) == LOW) {
    delay(10);
  }

  delay(BUTTON_DEBOUNCE_TIME_MS);
}