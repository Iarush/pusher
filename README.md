# Custom EV

<<<<<<< HEAD
## 1. Purpose
=======

[https://www.amazon.com/QCCAN-GY-6500-MPU-6500-Gyroscope-Accelerometer/dp/B0B74QXZS8/ref=sr_1_5_sspa?crid=2ARS825TM3OG2&dib=eyJ2IjoiMSJ9.DxBlHobOnDhJrtsCwX5sCMu4lSbi2lNWQJ1mwv0bV3-4eMQFu9_V180c-vNP9AK_pTCLGl25MUbwBqK2HZsd60hIyq_q7RLqoRLlIHLr7zLXU-EU9_D5AUeav4ODgFoED6W9zDXfM56GUzDfxH3cTLa_h7hVtwLyvBtvTQgBR9MvhL1RYFzQnAEsK1Sa_UJuXfWA9zJvXKhu2KRVhcusnl0OB6lXXaqPR7iTv2CgJn4.2WZ_oygO2dMwupOVsNDDpPX0Nwqr2vtgAMA53qvKCi8&dib_tag=se&keywords=MPU+6500&qid=1793845152&sprefix=mpu+650%2Caps%2C450&sr=8-5-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9tdGY&psc=1](MPU 6500)

This example is for a Raspberry Pi Pico 2 plugged into the **Waveshare
Pico-Motor-Driver** and driving two Pololu #998 50:1 Micro Metal Gearmotor HP
6 V motors on the C and D output groups.
>>>>>>> d809a803785cc1bd29adc0fe4e1bd5cccacf4a10

Custom EV is a small test platform for a two-wheel electric vehicle (EV) drive system. It uses a Raspberry Pi Pico 2 microcontroller, two brushed DC gear motors, and two magnetic encoders. The project checks that the motors, motor driver, and encoders work. Then it controls the car so it drives an exact distance.

The main goal is **accurate distance control**. The firmware drives the car forward to a target distance, stops it, and then drives it backward. The control loop uses encoder feedback. This feedback lets the car stop at the right place, even when the two wheels or the battery voltage change.

## 2. What the project does

- **Motor bring-up:** Checks that each motor turns in the correct direction and responds to duty cycle (the fraction of time the motor power is on).
- **Encoder test:** Reads the angle of each wheel and converts it to distance.
- **Distance drive:** Runs a forward leg, pauses, and then runs a reverse leg. Both legs use the same controller.
- **PID tuning:** Lets you change control gains and parameters from the Serial Monitor between runs. PID means proportional-integral-derivative, a standard feedback controller.
- **Status output:** Shows the result on the onboard LED (a blink code) and on the USB serial console (a text log).

## 3. Development workflow

The project follows a test-first sequence. Each stage proves one part of the system before the next stage uses it.

| Stage | Sketch or file | What it proves |
|---|---|---|
| 1. Power check | `Iarush_Code_WORKING/Test_Codes/Board_Power_Test.ino` | The Pico 2 runs and the onboard LED works. |
| 2. Motor check | `Iarush_Code_WORKING/Test_Codes/Motor_Test.ino`, `pico_motor_test/` | The motor driver responds, and both motors run and brake. |
| 3. Encoder check | `Iarush_Code_WORKING/Test_Codes/Encoder_Test.ino`, `pico_motor_test_encoder/` | Both MT6701 encoders answer on the I2C bus, and the angle changes as the wheel turns. |
| 4. Distance check | `pico_motor_test_encoder_distance/`, `correct_encoder_code1.cpp` | The car drives a set distance measured by the encoders. |
| 5. PID tuning | `Iarush_Code_WORKING/PID/PID_Tuner.ino` | You find gains that reach the target quickly with little overshoot. |
| 6. Drive run | `Iarush_Code_WORKING/PID/PID_Drive.ino` | The tuned gains drive a full forward-and-reverse cycle. |

### How to tune the controller

1. Upload `PID_Tuner.ino` to the Pico 2 with the Arduino IDE.
2. Open the Serial Monitor at **115200 baud**. Send `help` to see all commands.
3. Start a run with the button on GP15 or with the `g` command.
4. Change one gain at a time (`kp`, `kd`, `kh`, and so on) and run again. Use `history` to compare the last 10 runs.
5. Copy the final values into the constants at the top of `PID_Drive.ino`.
6. Record the final values in `Iarush_Code_WORKING/PID/PID_Constants.MD`.

The tuning order matters. Set the gains in this order: `kp`, then `kd`, then `kh`, then `mind`, then `ki`, then `cruise`. The tuning notes in `PID.md` and `PID_Constants.MD` explain each step.

### Record hardware changes

Log every hardware fault, rewire, or repair in `Iarush_Code_WORKING/Test_Codes/Hardware_Change_Log.MD`. Use the entry template at the end of that file. Each entry lists the issue, the actions taken, the result, and the follow-up items.

### Version control

- The repository is `main`-branch based. Commit each firmware change with a short message that names the change.
- Keep the working sketches in their folders. Do not overwrite a working sketch with an experiment. Create a new file or a new folder instead.

## 4. Hardware

### 4.1 Hardware list

| Part | Quantity | Role |
|---|---|---|
| Raspberry Pi Pico 2 (RP2350) | 1 | Main microcontroller. Runs the firmware, reads the encoders, and sends commands to the motor driver. |
| Waveshare Pico-Motor-Driver | 1 | Motor driver board. Holds a PCA9685 PWM controller and a TB6612FNG H-bridge (a circuit that reverses motor direction). |
| Pololu #998 50:1 Micro Metal Gearmotor HP, 6 V | 2 | Drive motors. The gearbox gives high torque at low speed. |
| MT6701 magnetic encoder (I2C mode) | 2 | Measures the angle of each wheel. One encoder per wheel. |
| Wheels, diameter 60.325 mm (circumference 189.52 mm) | 2 | Convert wheel rotation to distance. |
| 9 V supply | 1 | Powers the motor driver. |
| Momentary push button | 1 | Starts a run. |
| USB cable | 1 | Uploads firmware and provides the Serial Monitor. |

### 4.2 Why each part was chosen

The reasons below come from the specifications and from the design of the firmware. Where the repository does not record a reason, the text says so.

**Raspberry Pi Pico 2 (RP2350).**
The Pico 2 has two hardware I2C buses. The firmware needs two buses because the MT6701 encoder address is fixed at `0x06`. Two encoders cannot share one bus with the same address. The Pico 2 also has an Arduino core (the Earle Philhower RP2350 core) that supports the `Wire` and `Wire1` libraries. This core provides the I2C pin mapping that the firmware uses.

**Waveshare Pico-Motor-Driver (PCA9685 + TB6612FNG).**
The PCA9685 generates the PWM signals. It is a 16-channel PWM controller that uses I2C. It frees the Pico's GPIO pins and gives a stable PWM frequency (about 1 kHz in the firmware). The TB6612FNG switches the motor power and provides an active brake. The board has a regulated motor supply input. The repository notes that the TB6612FNG is rated for about 1.2 A continuous current.

**Pololu #998 gear motors, 6 V.**
These motors have a 50:1 gearbox. The gearbox trades speed for torque. The motors are rated for 6 V. The team's stiction tests (the force needed to start a stopped wheel moving) show that the car needs a minimum duty cycle before it moves. The firmware limits the maximum duty cycle to 6 V / 9 V = 66.7% (`MAX_DUTY`). This limit stops the motors from receiving more than their rated average voltage when the supply is 9 V.

**MT6701 magnetic encoders (I2C mode).**
The MT6701 is a 14-bit magnetic angle sensor. It gives 16,384 steps per revolution, which is fine enough to measure distance with good resolution. It uses I2C, so the Pico reads the angle with two wires and no extra interrupt pins. The encoder works at 3.3 V, which matches the Pico 2 logic level.

**9 V supply and 6 V motors.**
The motor supply is 9 V, and the motors are rated for 6 V. The firmware compensates for the higher voltage by limiting duty cycle. The repository does not record why a 9 V supply was chosen. The Pololu notes recommend a 6 V buck converter (a voltage reducer) for better accuracy. This change is not yet made.

**Wheel size.**
The wheel diameter of 60.325 mm gives a circumference of 189.52 mm. The firmware uses this value to convert encoder revolutions into centimeters.

### 4.3 Pin map

<<<<<<< HEAD
| Signal | Pico 2 pin | Used by |
|---|---|---|
| I2C0 SDA (PCA9685 and left encoder) | GP20 | All firmware |
| I2C0 SCL (PCA9685 and left encoder) | GP21 | All firmware |
| Right encoder SDA | GP2 (PID code) or GP18 (encoder test code) | See the open issue below |
| Right encoder SCL | GP3 (PID code) or GP19 (encoder test code) | See the open issue below |
| Start button | GP15 (internal pull-down in the PID firmware) | All firmware |
| Onboard LED | `LED_BUILTIN` | `PID_Drive.ino` status codes |

PCA9685 channel map (on the Waveshare board):

| Motor | PWM | IN1 | IN2 |
|---|---|---|---|
| Motor 1 (output C) | 6 | 7 | 8 |
| Motor 2 (output D) | 9 | 10 | 11 |

The PCA9685 uses I2C address `0x40`. The MT6701 encoders use I2C address `0x06`.

### 4.4 Power and wiring rules

- Power the MT6701 encoders from **3.3 V**, not 5 V.
- Do not connect the 9 V supply to a Pico GPIO pin.
- Connect the 9 V ground, the driver ground, and the Pico ground together.
- The Pico 2 GPIO pins are **not 5 V tolerant**. Use 3.3 V logic only.
- Add a current limit or a fuse to the motor supply. A stalled Pololu #998 can draw about 1.6 A at 6 V.

## 5. Open issues

- **Right encoder pins conflict.** The PID firmware (`PID_Drive.ino`, `PID_Tuner.ino`) and `Wiring.MD` use GP2/GP3 for the right encoder. `ENCODER_WIRING.md` and `pico_motor_test_encoder.ino` use GP18/GP19. Confirm which pins are wired on the board before you run the code. Then update the other files to match.
- **USB detection root cause is unknown.** The Pico 2 was not detected over USB in one wiring setup. The team changed the wiring, and the Pico works now. The root cause was not recorded. See the follow-up items in `Hardware_Change_Log.MD`.
- **Wiring diagram is missing.** `Wiring.MD` should show the current I2C wiring. Add photos or a diagram.
- **Motor voltage is not regulated.** The motors use a 9 V supply with PWM limiting. A 6 V buck converter would give more accurate speed.
- **Minimum duty values differ.** `Hardware_Change_Log.MD` records a minimum reverse duty of 0.055. `PID_Constants.MD` lists MIN DUTY 0.06. `PID_Drive.ino` uses `MIN_DUTY = 0.085`. Pick one value after a test on the car, and record it in the log.

## 6. Other folders

- `Assets/` contains a Replit web and API scaffold (pnpm workspace, Express API, Drizzle database). The repository does not use it for the car firmware. Its `replit.md` still has template placeholders.
- `Assets/attached_assets/` contains Arduino compile logs. They show the board package and build flags.
- `correct_encoder_code1.cpp` and `new_encoder_distance_code_copy_*.ino` are working copies of the distance-drive code.

## 7. Safety

- Test with the wheels off the ground or with the car unloaded first.
- Keep a way to cut the motor power quickly.
- The firmware brakes the motors when a run ends. Remove power after the car stops.
- If a wheel moves backward during a run, the firmware aborts the run. Check the sign of the encoder (`ENC_SIGN_L`, `ENC_SIGN_R`) and the motor direction (`LEFT_MOTOR_IS_M1`).
=======
Do the first test with the motor unloaded, a current-limited supply if
available, and a way to disconnect power quickly. The final active-brake state
is intentional; remove power after confirming both motors have stopped.
>>>>>>> d809a803785cc1bd29adc0fe4e1bd5cccacf4a10
