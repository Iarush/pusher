# Pololu #998 motor test

This example is for a Raspberry Pi Pico 2 plugged into the **Waveshare
Pico-Motor-Driver** and driving two Pololu #998 50:1 Micro Metal Gearmotor HP
6 V motors on the C and D output groups.

The program waits for a momentary start-button press, then runs the following
sequence:

1. Both motors run a 5-second forward profile: 5% to 30% over
   2.5 seconds, then down to 3% over 2.5 seconds.
2. Both motors reverse for 5 seconds while accelerating from 3% to 7%.
3. The TB6612FNG active brake is applied to both motors in one PCA9685
   transaction.

The Arduino uploadable code is in `pico_motor_test.ino` and uses the
Philhower core's built-in `Wire` library. A separate `main.cpp` Pico SDK
reference implementation is also included.

## 9 V source and 6 V motor

The code calculates a hard PWM ceiling of:

```text
6 V / 9 V = 66.7%
```

The forward profile starts at **5%**, reaches **30%**, then decelerates to
**3%** over 5 seconds. The reverse profile accelerates from **3%** to
**7%** over 5 seconds. Change the profile constants only after measuring
motor current and checking driver and motor temperature. PWM limits average
voltage; during each PWM high pulse the motor still sees the 9 V rail. It is
not a substitute for a regulated 6 V supply.

For a precise robot, use a 6 V buck converter between the 9 V source and the
driver's motor supply input if possible. Pololu #998 has a theoretical stall
current around 1.6 A at 6 V, while the TB6612FNG is typically rated for about
1.2 A continuous, so avoid a mechanical stall and add appropriate current
limiting or fuse protection.

## Wiring

- Pico 2 and Waveshare board: plug the Pico into the board header.
- Motor 1: connect to the board's **C1/C2** output.
- Motor 2: connect to the board's **D1/D2** output.
- Motor supply: connect the 9 V source to the driver's VIN and GND.
- Do not connect the 9 V source directly to Pico GPIO pins.
- Ensure the 9 V source ground, driver ground, and Pico ground are common.
- Start button: connect one side of a momentary button to Pico **GP15** and
  the other side to Pico **GND**. The sketch uses `INPUT_PULLUP`, so no 5 V
  connection is needed.
- For a 3-pin button module, connect VCC to Pico **3V3(OUT)**, GND to Pico
  **GND**, and SIG to **GP15**. Never connect a 5 V signal directly to GP15;
  Pico 2 GPIO pins are not 5 V tolerant.

The source assumes:

- I2C0 SDA = Pico GP20
- I2C0 SCL = Pico GP21
- PCA9685 address = `0x40`
- Motor 1 / C1-C2 mapping = PCA9685 channels 6 (PWM), 7 (IN1), 8 (IN2)
- Motor 2 / D1-D2 mapping = PCA9685 channels 9 (PWM), 10 (IN1), 11 (IN2)

Check the Waveshare schematic for your board revision. If the motor rotates
opposite to the desired robot direction, swap the IN1 and IN2 channel
assignments for that motor. If the I2C address jumpers or I2C pins differ,
update the constants at the top of `pico_motor_test.ino`.

Both motors receive the same PCA9685 PWM frequency and the same PWM count.
The sketch writes channels 6 through 11 together to minimize timing
difference. Identical PWM does not guarantee identical physical RPM under
different loads; encoder feedback is required for closed-loop velocity
matching.

## Arduino IDE with the Earle Philhower RP2350 core

Open `pico_motor_test.ino` in Arduino IDE. In **Tools**, select:

- A Raspberry Pi Pico 2 / RP2350 board provided by the Earle F. Philhower
  Arduino core
- The correct USB port

Upload the sketch. The motors remain stopped until the momentary button on
GP15 is pressed. Open Serial Monitor at `115200` baud to see the start and
completion messages.

The sketch uses only the built-in `Wire` library; no additional Arduino
library is required. It calls `Wire.setSDA(20)` and `Wire.setSCL(21)` before
`Wire.begin()`, which is supported by the Philhower RP2350 core.

If your board revision uses a different I2C address or channel mapping, change
the constants at the top of `pico_motor_test.ino`:

```cpp
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t MOTOR_1_PWM_CHANNEL = 6;
constexpr uint8_t MOTOR_1_IN1_CHANNEL = 7;
constexpr uint8_t MOTOR_1_IN2_CHANNEL = 8;
constexpr uint8_t MOTOR_2_PWM_CHANNEL = 9;
constexpr uint8_t MOTOR_2_IN1_CHANNEL = 10;
constexpr uint8_t MOTOR_2_IN2_CHANNEL = 11;
```

## Pico SDK build integration

Add the source to a Pico SDK project that already initializes the SDK, then
link these libraries:

```cmake
target_link_libraries(motor_test
    pico_stdlib
    hardware_i2c
)
```

Set the board for Pico 2 before configuring the build:

```bash
cmake -DPICO_BOARD=pico2 ..
cmake --build . -j
```

Do the first test with the motor unloaded, a current-limited supply if
available, and a way to disconnect power quickly. The final active-brake state
is intentional; remove power after confirming both motors have stopped.