// Pico 2 + Waveshare Pico-Motor-Driver (PCA9685 @ 0x40) + I2C MT6701 encoders
// Press start -> wait 2 s -> drive forward 10 in at 5% -> brake.
// Left encoder: I2C0 (GP20/GP21, shared with PCA9685). Right encoder: I2C1 (GP2/GP3).
#include <Wire.h>

// ===== SETTINGS =====
const float CIRCUMFERENCE_MM = 189.52;    // wheel circumference
const float TARGET_CM = 25.4;             // 10 inches
const float POWER = 0.05;                 // 5% duty
const unsigned long START_DELAY_MS = 2000;
const unsigned long TIMEOUT_MS = 20000;   // safety stop after motion starts
const bool ACTIVE_LOW = false;            // false = button outputs HIGH when pressed
// ====================

const uint8_t PCA_ADDR = 0x40;
const uint8_t ENC_ADDR = 0x06;            // MT6701 default I2C address
const int BUTTON_PIN = 15;

const uint8_t M1_PWM = 6, M1_IN1 = 7, M1_IN2 = 8;
const uint8_t M2_PWM = 9, M2_IN1 = 10, M2_IN2 = 11;

enum State { IDLE, WAITING, RUNNING };
State state = IDLE;

bool lastPressed = false;
unsigned long lastChange = 0, pressTime = 0, startTime = 0;

int lPrev = 0, rPrev = 0;                 // previous raw angle (0..16383)
float lRevs = 0, rRevs = 0;               // unwrapped revolutions

// ---------- PCA9685 ----------
void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setChannel(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void setDuty(uint8_t ch, float duty) {
  if (duty <= 0) setChannel(ch, 0, 0x1000);
  else if (duty >= 1) setChannel(ch, 0x1000, 0);
  else setChannel(ch, 0, (uint16_t)(duty * 4095));
}

void setDigital(uint8_t ch, bool high) {
  if (high) setChannel(ch, 0x1000, 0);
  else setChannel(ch, 0, 0x1000);
}

void motorsRun() {
  setDigital(M1_IN1, true);  setDigital(M1_IN2, false);
  setDigital(M2_IN1, true);  setDigital(M2_IN2, false);
  setDuty(M1_PWM, POWER);
  setDuty(M2_PWM, POWER);
}

void motorsBrake() {
  setDigital(M1_IN1, true);  setDigital(M1_IN2, true);
  setDigital(M2_IN1, true);  setDigital(M2_IN2, true);
  setDuty(M1_PWM, 1.0);
  setDuty(M2_PWM, 1.0);
}

void motorsCoast() {
  setDuty(M1_PWM, 0);
  setDuty(M2_PWM, 0);
  setDigital(M1_IN1, false); setDigital(M1_IN2, false);
  setDigital(M2_IN1, false); setDigital(M2_IN2, false);
}

void pcaInit() {
  pcaWrite(0x00, 0x10);
  pcaWrite(0xFE, 5);        // ~1 kHz PWM
  pcaWrite(0x00, 0x20);
  delay(5);
  pcaWrite(0x01, 0x04);
}

// ---------- MT6701 over I2C ----------
// Returns 14-bit angle 0..16383, or -1 on error.
int readAngle(TwoWire &w) {
  w.beginTransmission(ENC_ADDR);
  w.write(0x03);                          // ANGLE_H register (bits 13:6), 0x04 holds bits 5:0 in top 6 bits
  if (w.endTransmission(false) != 0) return -1;
  if (w.requestFrom((uint8_t)ENC_ADDR, (size_t)2) < 2) return -1;
  int hi = w.read();
  int lo = w.read();
  return (hi << 6) | (lo >> 2);
}

// Change since last reading as a fraction of a rev, handling the wrap.
float deltaRevs(int &prev, int now) {
  int d = now - prev;
  if (d > 8192) d -= 16384;
  if (d < -8192) d += 16384;
  prev = now;
  return d / 16384.0;
}

void updateEncoders() {
  int l = readAngle(Wire);
  int r = readAngle(Wire1);
  if (l >= 0) lRevs += deltaRevs(lPrev, l);
  if (r >= 0) rRevs += deltaRevs(rPrev, r);
}

// Uses |each wheel| so a flipped encoder direction can't break the stop.
float distanceCM() {
  return ((fabs(lRevs) + fabs(rRevs)) / 2.0) * CIRCUMFERENCE_MM / 10.0;
}

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  return ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  Wire.setSDA(20);  Wire.setSCL(21);  Wire.begin();     // I2C0
  Wire1.setSDA(2);  Wire1.setSCL(3);  Wire1.begin();    // I2C1

  Wire.beginTransmission(PCA_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found" : "PCA9685 NOT found");

  int l = readAngle(Wire), r = readAngle(Wire1);
  Serial.println(l >= 0 ? "Left encoder OK" : "Left encoder NOT found");
  Serial.println(r >= 0 ? "Right encoder OK" : "Right encoder NOT found");
  lPrev = max(l, 0);
  rPrev = max(r, 0);

  pcaInit();
  motorsCoast();

  lastPressed = readPressed();
  lastChange = millis();
}

void loop() {
  updateEncoders();

  // Button: start a run
  bool pressed = readPressed();
  if (pressed != lastPressed && millis() - lastChange > 40) {
    lastChange = millis();
    lastPressed = pressed;
    if (pressed && state == IDLE) {
      state = WAITING;
      pressTime = millis();
      Serial.println("Button pressed, starting in 2 s");
    }
  }

  // 2 second delay, then go
  if (state == WAITING && millis() - pressTime >= START_DELAY_MS) {
    lRevs = 0; rRevs = 0;
    startTime = millis();
    motorsRun();
    state = RUNNING;
    Serial.println("RUN");
  }

  // Stop conditions
  if (state == RUNNING) {
    float d = distanceCM();
    if (d >= TARGET_CM) {
      motorsBrake();
      state = IDLE;
      Serial.print("DONE at "); Serial.print(d); Serial.println(" cm");
    } else if (millis() - startTime > TIMEOUT_MS) {
      motorsBrake();
      state = IDLE;
      Serial.print("TIMEOUT at "); Serial.print(d); Serial.println(" cm");
    }
  }
}



kh 0 run: see how far the car drifts on its own. Check max_head in the summary.
kp: raise it until the car reaches the target quickly. Once it overshoots by more than a couple of cm, back off.
kd: start at 0.0002 and raise it until the overshoot is small.
kh: start at 0.001 and raise it until max_head stays small. If max_head gets worse, flip LEFT_MOTOR_IS_M1.
mind: only if the car stops short or times out. Raise it until it just creeps into the target.
ki: only if a steady error of a few mm remains. Keep izone around 20–30 and ilimit around 0.05–0.1.
cruise: raise it last, to 0.25–0.3, then check that the gains still hold. Faster runs usually need a bit more kd.