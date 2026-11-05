# MT6701 Encoder Wiring — Raspberry Pi Pico 2

Two MT6701 magnetic encoders (I2C mode), one per motor. The MT6701's I2C
address is fixed at `0x06` and is not configurable, so the two encoders
cannot share a single I2C bus — each one gets its own hardware I2C
peripheral on the RP2350.

**Power both encoder boards from 3.3V, not 5V.** The MT6701 and its I2C
lines are 3.3V logic; the Pico 2's 3V3 pin is a regulated source
suitable for both.

## Encoder 1 (Motor 1) — shares the existing I2C0 bus

This bus already has the Waveshare Pico-Motor-Driver's PCA9685
(address `0x40`) on it. Since the PCA9685 and MT6701 use different
addresses, the encoder can share the bus with no new pins.

| MT6701 pin | Pico 2 pin | Notes |
|---|---|---|
| VCC | 3V3 (OUT) | **3.3V only** |
| GND | GND | |
| SDA | GP20 | `SDA_PIN`, shared with PCA9685 |
| SCL | GP21 | `SCL_PIN`, shared with PCA9685 |

## Encoder 2 (Motor 2) — dedicated I2C1 bus

Because the MT6701 address can't be changed, encoder 2 must run on a
separate hardware I2C bus.

| MT6701 pin | Pico 2 pin | Notes |
|---|---|---|
| VCC | 3V3 (OUT) | **3.3V only** |
| GND | GND | |
| SDA | GP18 | `ENCODER2_SDA_PIN` in firmware |
| SCL | GP19 | `ENCODER2_SCL_PIN` in firmware |

If you rewire encoder 2 to different pins, update
`ENCODER2_SDA_PIN` / `ENCODER2_SCL_PIN` at the top of
[pico_motor_test_encoder.ino](pico_motor_test_encoder.ino) to match —
those two constants are the only place the pin assignment is defined.

## Pull-ups

Most MT6701 breakout boards include their own I2C pull-up resistors.
If yours doesn't (or you're wiring the bare IC), add 2.2–4.7kΩ pull-ups
from SDA and SCL to 3.3V on **each** bus (GP20/GP21 and GP18/GP19
separately — pull-ups don't carry across buses).

## Sanity check before trusting the firmware

On power-up, the sketch checks that both encoders ACK on their
respective bus before it will start the motor test, and prints an
`ENCODER FAULT` message naming which one failed if not. If you get that
message, recheck VCC, GND, and that SDA/SCL aren't swapped — a swapped
SDA/SCL pair on the MT6701 will also fail to ACK.

## Full pin summary

| Signal | Pico 2 pin |
|---|---|
| PCA9685 + Encoder 1 SDA | GP20 |
| PCA9685 + Encoder 1 SCL | GP21 |
| Encoder 2 SDA | GP18 |
| Encoder 2 SCL | GP19 |
| Start button | GP15 (internal pull-up) |
