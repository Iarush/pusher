/*
 * Pololu #998 motor test for Raspberry Pi Pico 2
 * Waveshare Pico-Motor-Driver
 *
 * Sequence:
 *   1. Forward for 3 seconds
 *   2. Immediately reverse for 5 seconds
 *   3. Apply an active brake and leave the motor stopped
 *
 * This targets the Waveshare Pico-Motor-Driver, which uses:
 *   - PCA9685 PWM controller over I2C
 *   - TB6612FNG dual H-bridge
 *
 * Important:
 *   The board mapping below follows the common Waveshare channel layout:
 *     Motor 1: PCA9685 channels 0 = PWM, 1 = IN1, 2 = IN2
 *
 *   Confirm the channel labels in the schematic for your exact board revision
 *   before connecting the motor. If forward/reverse is swapped, exchange
 *   MOTOR_IN1_CHANNEL and MOTOR_IN2_CHANNEL.
 */

#include <algorithm>
#include <cstdint>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace {

// Raspberry Pi Pico/Pico 2 I2C0 defaults commonly used by the Waveshare board.
constexpr i2c_inst_t* I2C_PORT = i2c0;
constexpr uint SDA_PIN = 4;
constexpr uint SCL_PIN = 5;
constexpr uint I2C_BAUD_HZ = 400'000;

// PCA9685 address with the address jumpers in their default positions.
// Change this if the board's address jumpers are changed.
constexpr uint8_t PCA9685_ADDRESS = 0x40;

// PCA9685 channels for motor 1 on the Waveshare Pico-Motor-Driver.
constexpr uint8_t MOTOR_PWM_CHANNEL = 0;
constexpr uint8_t MOTOR_IN1_CHANNEL = 1;
constexpr uint8_t MOTOR_IN2_CHANNEL = 2;

constexpr uint16_t PCA9685_FULL_SCALE = 4095;

// The motor is rated for 6 V and the source is 9 V.
constexpr float MOTOR_RATED_VOLTAGE = 6.0f;
constexpr float SUPPLY_VOLTAGE = 9.0f;

// 60% is intentionally below the mathematical 6/9 = 66.7% ceiling.
// This is the initial test setting; increase only after checking current,
// temperature, gearing, and mechanical load.
constexpr float TEST_DUTY_PERCENT = 60.0f;

constexpr uint32_t FORWARD_TIME_MS = 3'000;
constexpr uint32_t REVERSE_TIME_MS = 5'000;

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

// TB6612FNG control states. The PCA9685 channels connected to IN1/IN2 use
// full-on/full-off output, while the PWM channel controls motor speed.
enum class Direction : uint8_t {
  Forward,
  Reverse,
};

void pca9685_write_register(uint8_t reg, uint8_t value) {
  const uint8_t data[] = {reg, value};
  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

void pca9685_set_channel(uint8_t channel, uint16_t on, uint16_t off) {
  if (channel > 15) {
    return;
  }

  const uint8_t register_address =
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * channel));
  const uint8_t data[] = {
      register_address,
      static_cast<uint8_t>(on & 0xFF),
      static_cast<uint8_t>((on >> 8) & 0x0F),
      static_cast<uint8_t>(off & 0xFF),
      static_cast<uint8_t>((off >> 8) & 0x0F),
  };

  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

void pca9685_set_output(uint8_t channel, bool enabled) {
  // PCA9685 full-on and full-off bits live in the high byte of the channel.
  if (channel > 15) {
    return;
  }

  const uint8_t register_address =
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * channel));
  const uint8_t data[] = {
      register_address,
      0x00,
      enabled ? 0x10 : 0x00,
      0x00,
      enabled ? 0x00 : 0x10,
  };

  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

void pca9685_set_pwm_frequency(uint16_t frequency_hz) {
  // PCA9685 oscillator is nominally 25 MHz.
  const float prescale_value =
      (25'000'000.0f / (4096.0f * frequency_hz)) - 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(
      std::clamp(prescale_value, 3.0f, 255.0f));

  uint8_t mode1 = 0;
  const uint8_t mode1_register[] = {PCA9685_MODE1};
  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, mode1_register, 1, true);
  i2c_read_blocking(I2C_PORT, PCA9685_ADDRESS, &mode1, 1, false);

  // Sleep before changing PRE_SCALE, as required by the PCA9685.
  pca9685_write_register(PCA9685_MODE1, static_cast<uint8_t>(
      (mode1 & 0x7F) | 0x10));
  pca9685_write_register(PCA9685_PRESCALE, prescale);
  pca9685_write_register(PCA9685_MODE1, mode1);
  sleep_us(500);
  pca9685_write_register(PCA9685_MODE1, static_cast<uint8_t>(mode1 | 0xA0));
}

uint16_t rated_voltage_pwm_limit() {
  const float limit = (MOTOR_RATED_VOLTAGE / SUPPLY_VOLTAGE) *
                      static_cast<float>(PCA9685_FULL_SCALE);
  return static_cast<uint16_t>(std::clamp(
      limit, 0.0f, static_cast<float>(PCA9685_FULL_SCALE)));
}

uint16_t test_pwm_value() {
  const float requested = (TEST_DUTY_PERCENT / 100.0f) *
                          static_cast<float>(PCA9685_FULL_SCALE);
  return static_cast<uint16_t>(std::clamp(
      requested, 0.0f, static_cast<float>(rated_voltage_pwm_limit())));
}

void motor_coast() {
  pca9685_set_channel(MOTOR_PWM_CHANNEL, 0, 0);
  pca9685_set_output(MOTOR_IN1_CHANNEL, false);
  pca9685_set_output(MOTOR_IN2_CHANNEL, false);
}

void motor_drive(Direction direction, uint16_t pwm_value) {
  const bool forward = direction == Direction::Forward;

  // Direction inputs are set before PWM is applied, avoiding an enabled
  // bridge with an undefined direction.
  pca9685_set_output(MOTOR_IN1_CHANNEL, forward);
  pca9685_set_output(MOTOR_IN2_CHANNEL, !forward);
  pca9685_set_channel(MOTOR_PWM_CHANNEL, 0, pwm_value);
}

void motor_active_brake() {
  // TB6612FNG short-brake mode: IN1 = HIGH, IN2 = HIGH, PWM = HIGH.
  // This stops faster than simply disabling PWM/coasting.
  pca9685_set_output(MOTOR_IN1_CHANNEL, true);
  pca9685_set_output(MOTOR_IN2_CHANNEL, true);
  pca9685_set_output(MOTOR_PWM_CHANNEL, true);
}

void motor_test_sequence() {
  const uint16_t pwm = test_pwm_value();

  motor_drive(Direction::Forward, pwm);
  sleep_ms(FORWARD_TIME_MS);

  // No coast delay: this is the requested immediate direction change.
  motor_drive(Direction::Reverse, pwm);
  sleep_ms(REVERSE_TIME_MS);

  // Brake immediately when the reverse interval ends.
  motor_active_brake();
}

}  // namespace

int main() {
  stdio_init_all();

  i2c_init(I2C_PORT, I2C_BAUD_HZ);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  // Start with the bridge inactive while the PCA9685 is configured.
  motor_coast();

  pca9685_write_register(PCA9685_MODE1, 0x00);
  pca9685_write_register(PCA9685_MODE2, 0x04);  // totem-pole outputs
  pca9685_set_pwm_frequency(1'000);             // quiet motor PWM

  // Run once after boot. The final brake state is maintained indefinitely.
  motor_test_sequence();

  while (true) {
    tight_loop_contents();
  }
}