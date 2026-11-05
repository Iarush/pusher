// Pico 2 + Waveshare Pico-Motor-Driver (PCA9685 @ 0x40) + I2C MT6701 encoders
// Press start (or send 'g') -> wait 2 s -> drive TARGET_CM using PID -> brake
// -> wait 2 s -> reverse REVERSE_CM using the same PID -> brake.
// TARGET_CM is how far FORWARD the car goes from the start (its farthest
// point), not where it ends up. REVERSE_CM is measured back from wherever the
// forward leg stopped, so the car ends about TARGET_CM - REVERSE_CM from the
// start.
// The car cruises at CRUISE duty (REV_CRUISE when reversing), drops to
// SLOW_DUTY (REV_SLOW_DUTY) inside the last SLOW_CM, then PID slows it down to
// stop on the target.
// A second (heading) loop trims left/right duty to keep both wheels even.
// No right-motor start boost: both motors start on the same PID/heading duty.
// Left encoder: I2C0 (GP20/GP21, shared with PCA9685). Right encoder: I2C1
// (GP2/GP3).
//
// Onboard LED: solid while running; after a run it blinks the forward result,
// pauses, then the reverse result (see updateLed for the codes).
//
// Serial (115200, newline): dist <cm>   rev <cm>   cruise <duty>
//   slow <cm>   slowduty <duty>   revcruise <duty>   revslowduty <duty>
//   g (go)   x (stop)   show
#include <Wire.h>

// ===== SETTINGS =====
// Max distance forward from the start (also: "dist <cm>")
float TARGET_CM = 1050.0;
// Distance to back up from where the forward leg stopped (also: "rev <cm>")
float REVERSE_CM = 200.0;
float CRUISE = 0.30; // default cruise duty (also: "cruise <duty>")
// Slow zone: inside the last SLOW_CM of a leg the duty is capped at SLOW_DUTY.
// A SLOW_DUTY that is too low may stall the car.
float SLOW_CM = 0.0;   // SLows down x distance from the target
float SLOW_DUTY = 0.3; // (also: "slowduty <duty>")
// Reverse leg duty (same SLOW_CM slow zone). Reverse may need different duty
// than forward, e.g. to push through stiction or to coast in more gently.
float REV_CRUISE = 0.30;    // reverse cruise (also: "revcruise <duty>")
float REV_SLOW_DUTY = 0.15; // reverse slow zone (also: "revslowduty <duty>")

const float CIRCUMFERENCE_MM = 189.52;     // wheel circumference
const int ENC_SIGN_L = -1, ENC_SIGN_R = 1; // flip so forward counts positive
// M1 = driver channel C, M2 = channel D. Right motor is on C, left on D.
const bool LEFT_MOTOR_IS_M1 = false;       // flip if steering makes drift worse

// Distance PID (copy tuned values from PID_Tuner)
const float KP = 0.005;       // duty per mm of error
const float KI = 0.0;         // duty per (mm*s)
const float KD = 0.0006;      // duty per (mm/s)
const float I_ZONE_MM = 30.0; // integrate only when this close
const float I_LIMIT = 0.10;   // clamp on the integral term

// Heading (keep left and right wheel distance equal)
const float KH = 0.0015;      // duty per mm of L-R mismatch
const float KHD = 0.00015;    // duty per (mm/s) of L-R mismatch rate
const float STEER_MAX = 0.08; // max left/right duty difference
const float TRIM =
    0.9806; // left-side duty multiplier (<1 slows left, fixes a right veer)

const float MIN_DUTY = 0.085;           // stiction floor while short of target
const float SLEW = 2.0;                 // max duty rise per second (soft start)
const float TOL_MM = 8.0;               // done when this close...
const float DONE_SPEED_MM_S = 20.0;     // ...and slower than this...
const unsigned long DONE_HOLD_MS = 100; // ...for this long
const float SPEED_FILTER = 0.3;         // speed low-pass (1 = no filtering)

const float MAX_DUTY = 6.0 / 7.8; // 6 V motors on a 7.8 V battery
const unsigned long START_DELAY_MS = 2000;
const unsigned long PAUSE_MS = 2000;  // wait at the target before reversing
const unsigned long BRAKE_MS = 300;   // brake time before coasting
const float BACKWARD_ABORT_MM = 50.0; // a wheel going backward = wrong ENC_SIGN
const unsigned long LOOP_MS = 10;     // 100 Hz control loop
const int ENC_FAIL_ABORT_TICKS = 10;  // abort after this many failed reads
const bool ACTIVE_LOW = false; // false = button outputs HIGH when pressed
// ====================

const uint8_t PCA_ADDR = 0x40;
const uint8_t ENC_ADDR = 0x06; // MT6701 default I2C address
const int BUTTON_PIN = 15;

const uint8_t M1_PWM = 6, M1_IN1 = 7, M1_IN2 = 8;
const uint8_t M2_PWM = 9, M2_IN1 = 10, M2_IN2 = 11;

enum State { IDLE, WAITING, RUNNING, BRAKING, PAUSED };
State state = IDLE;

bool lastPressed = false;
unsigned long lastChange = 0, pressTime = 0, startTime = 0, lastTick = 0;
unsigned long brakeTime = 0, doneSince = 0;

// Each leg (forward, then reverse) is measured from where it started.
int dir = 1;                // +1 forward, -1 reverse
float legTargetMM = 0;      // distance to travel this leg
float legL0 = 0, legR0 = 0; // wheel positions at the start of the leg
bool reverseNext = false; // forward leg finished OK -> reverse after the pause

int lPrev = 0, rPrev = 0;   // previous raw angle (0..16383)
float lRevs = 0, rRevs = 0; // unwrapped revolutions
float prevDist = 0, speed = 0, iTerm = 0, prevOut = 0, prevHeadErr = 0;
float lPeak = 0, rPeak = 0;
int lFail = 0, rFail = 0; // failed encoder reads in a row

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
  if (duty <= 0)
    setChannel(ch, 0, 0x1000);
  else if (duty >= 1)
    setChannel(ch, 0x1000, 0);
  else
    setChannel(ch, 0, (uint16_t)(duty * 4095));
}

void setDigital(uint8_t ch, bool high) {
  if (high)
    setChannel(ch, 0x1000, 0);
  else
    setChannel(ch, 0, 0x1000);
}

void motorsForwardMode() {
  setDigital(M1_IN1, true);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, true);
  setDigital(M2_IN2, false);
}

void motorsReverseMode() {
  setDigital(M1_IN1, false);
  setDigital(M1_IN2, true);
  setDigital(M2_IN1, false);
  setDigital(M2_IN2, true);
}

// Set left/right duty (direction pins must already be set for the leg).
void motorsDrive(float left, float right) {
  left = constrain(left, 0.0f, MAX_DUTY);
  right = constrain(right, 0.0f, MAX_DUTY);
  setDuty(M1_PWM, LEFT_MOTOR_IS_M1 ? left : right);
  setDuty(M2_PWM, LEFT_MOTOR_IS_M1 ? right : left);
}

void motorsBrake() {
  setDigital(M1_IN1, true);
  setDigital(M1_IN2, true);
  setDigital(M2_IN1, true);
  setDigital(M2_IN2, true);
  setDuty(M1_PWM, 1.0);
  setDuty(M2_PWM, 1.0);
}

void motorsCoast() {
  setDuty(M1_PWM, 0);
  setDuty(M2_PWM, 0);
  setDigital(M1_IN1, false);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, false);
  setDigital(M2_IN2, false);
}

void pcaInit() {
  pcaWrite(0x00, 0x10);
  pcaWrite(0xFE, 5); // ~1 kHz PWM
  pcaWrite(0x00, 0x20);
  delay(5);
  pcaWrite(0x01, 0x04);
}

// ---------- MT6701 over I2C ----------
// Returns 14-bit angle 0..16383, or -1 on error.
int readAngle(TwoWire &w) {
  w.beginTransmission(ENC_ADDR);
  w.write(
      0x03); // ANGLE_H register (bits 13:6), 0x04 holds bits 5:0 in top 6 bits
  if (w.endTransmission(false) != 0)
    return -1;
  if (w.requestFrom((uint8_t)ENC_ADDR, (size_t)2) < 2)
    return -1;
  int hi = w.read();
  int lo = w.read();
  return (hi << 6) | (lo >> 2);
}

// Change since last reading as a fraction of a rev, handling the wrap.
float deltaRevs(int &prev, int now) {
  int d = now - prev;
  if (d > 8192)
    d -= 16384;
  if (d < -8192)
    d += 16384;
  prev = now;
  return d / 16384.0;
}

void updateEncoders() {
  int l = readAngle(Wire);
  int r = readAngle(Wire1);
  if (l >= 0) {
    lRevs += deltaRevs(lPrev, l) * ENC_SIGN_L;
    lFail = 0;
  } else
    lFail++;
  if (r >= 0) {
    rRevs += deltaRevs(rPrev, r) * ENC_SIGN_R;
    rFail = 0;
  } else
    rFail++;
}

float leftMM() { return lRevs * CIRCUMFERENCE_MM; }
float rightMM() { return rRevs * CIRCUMFERENCE_MM; }

// Progress of each wheel along the current leg (positive = toward the leg's
// target, so these are positive in both directions).
float legLeftMM() { return dir * (leftMM() - legL0); }
float legRightMM() { return dir * (rightMM() - legR0); }
float legDistMM() { return (legLeftMM() + legRightMM()) / 2.0; }

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  return ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

// ---------- Status LED (readable without USB serial) ----------
// Solid on while a run is in progress. When idle after a run it blinks the
// forward result, pauses, blinks the reverse result (nothing if reverse never
// started), pauses longer, and repeats.
//   1 = DONE   2 = (unused)   3 = ABORT (wheel backward)   4 = STOPPED
//   5 = ABORT (encoder stopped responding on I2C)
int fwdCode = 0, revCode = 0;

void updateLed() {
  if (state != IDLE) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  const unsigned long BLINK_MS = 500, GAP_MS = 1500, END_MS = 3000;
  unsigned long aLen = fwdCode * BLINK_MS, bLen = revCode * BLINK_MS;
  unsigned long t = millis() % (aLen + GAP_MS + bLen + END_MS);
  bool on = false;
  if (t < aLen)
    on = t % BLINK_MS < BLINK_MS / 2;
  else if (t >= aLen + GAP_MS && t < aLen + GAP_MS + bLen)
    on = (t - aLen - GAP_MS) % BLINK_MS < BLINK_MS / 2;
  digitalWrite(LED_BUILTIN, on);
}

// ---------- Run control ----------
void startRun() {
  if (state != IDLE)
    return;
  state = WAITING;
  reverseNext = false;
  fwdCode = revCode = 0;
  pressTime = millis();
  Serial.print("Driving ");
  Serial.print(TARGET_CM);
  Serial.print(" cm at cruise ");
  Serial.print(CRUISE, 3);
  Serial.print(" (slow to ");
  Serial.print(SLOW_DUTY, 3);
  Serial.print(" in last ");
  Serial.print(SLOW_CM);
  Serial.print(" cm), then reverse ");
  Serial.print(REVERSE_CM);
  Serial.print(" cm at cruise ");
  Serial.print(REV_CRUISE, 3);
  Serial.print(" (slow to ");
  Serial.print(REV_SLOW_DUTY, 3);
  Serial.println(") - starting in 2 s");
}

// Start a leg of targetCm in direction (+1 forward, -1 reverse) from here.
void beginLeg(int direction, float targetCm) {
  updateEncoders();
  dir = direction;
  legTargetMM = targetCm * 10.0;
  legL0 = leftMM();
  legR0 = rightMM();
  prevDist = speed = iTerm = prevOut = prevHeadErr = 0;
  lPeak = rPeak = 0;
  lFail = rFail = 0;
  doneSince = 0;
  startTime = lastTick = millis();
  if (dir > 0)
    motorsForwardMode();
  else
    motorsReverseMode();
  motorsDrive(0, 0);
  state = RUNNING;
  Serial.println(dir > 0 ? "RUN" : "REVERSE");
}

void beginRun() {
  updateEncoders(); // sync prev angles, then zero
  lRevs = 0;
  rRevs = 0;
  beginLeg(1, TARGET_CM);
}

// ok = the leg reached its target (only then does the forward leg go on to
// reverse). code is the status LED blink count for this ending.
void stopRun(const char *why, int code, bool ok = false) {
  motorsBrake();
  brakeTime = millis();
  state = BRAKING;
  reverseNext = ok && dir > 0;
  if (dir > 0) {
    fwdCode = code;
    revCode = 0;
  } else
    revCode = code;
  float d = legDistMM();
  Serial.print(dir > 0 ? "[fwd] " : "[rev] ");
  Serial.print(why);
  Serial.print(" at ");
  Serial.print(d / 10.0);
  Serial.print(" cm (error ");
  Serial.print(legTargetMM - d, 1);
  Serial.print(" mm, L-R ");
  Serial.print(legLeftMM() - legRightMM(), 1);
  Serial.print(" mm, ");
  Serial.print(millis() - startTime);
  Serial.println(" ms)");
}

void controlTick() {
  unsigned long now = millis();
  float dt = (now - lastTick) / 1000.0;
  lastTick = now;
  if (dt <= 0 || dt > 0.1)
    dt = LOOP_MS / 1000.0;

  updateEncoders();
  // An encoder that stops answering freezes the measured distance while the
  // motors keep pushing, so the car would run on past the target.
  if (lFail >= ENC_FAIL_ABORT_TICKS || rFail >= ENC_FAIL_ABORT_TICKS) {
    stopRun("ABORT (encoder stopped responding on I2C)", 5);
    return;
  }
  float d = legDistMM();
  float err = legTargetMM - d;
  speed += SPEED_FILTER * ((d - prevDist) / dt - speed);
  prevDist = d;

  // Safety: a wheel running away from the target means a direction or encoder
  // sign is wrong.
  float lp = legLeftMM(), rp = legRightMM();
  lPeak = max(lPeak, lp);
  rPeak = max(rPeak, rp);
  if (lPeak - lp > BACKWARD_ABORT_MM || rPeak - rp > BACKWARD_ABORT_MM) {
    stopRun("ABORT (wheel backward - check ENC_SIGN)", 3);
    return;
  }

  // Distance PID. D acts on measured speed so it brakes as the car closes in.
  if (fabs(err) < I_ZONE_MM)
    iTerm = constrain(iTerm + KI * err * dt, -I_LIMIT, I_LIMIT);
  float cmd = KP * err + iTerm - KD * speed;
  float cap = min(dir > 0 ? CRUISE : REV_CRUISE, MAX_DUTY); // top speed...
  if (err <= SLOW_CM * 10.0)
    cap = min(cap, dir > 0 ? SLOW_DUTY : REV_SLOW_DUTY); // ...less in slow zone
  cmd = constrain(cmd, 0.0f, cap);
  cmd = min(cmd, prevOut + SLEW * dt); // soft start
  if (err > TOL_MM && cmd < MIN_DUTY)
    cmd = min(MIN_DUTY, cap); // push through stiction (never above the cap)
  prevOut = cmd;

  // Heading PD on the L-R mismatch: left ahead -> slow left, speed up right.
  float headErr = lp - rp;
  float headRate = (headErr - prevHeadErr) / dt;
  prevHeadErr = headErr;
  float steer =
      cmd > 0 ? constrain(KH * headErr + KHD * headRate, -STEER_MAX, STEER_MAX)
              : 0;
  float l = (cmd - steer) * TRIM, r = cmd + steer;
  float over =
      max(max(l, r) - MAX_DUTY, 0.0f); // keep the difference under the cap
  motorsDrive(l - over, r - over);

  // Done: at the target (or past it) and nearly stopped for DONE_HOLD_MS. The
  // duty never goes negative, so an overshoot can't be driven back; it must
  // count as done or the leg would sit there forever and never reverse.
  if (err < TOL_MM && fabs(speed) < DONE_SPEED_MM_S) {
    if (doneSince == 0)
      doneSince = now;
    if (now - doneSince >= DONE_HOLD_MS)
      stopRun("DONE", 1, true);
  } else {
    doneSince = 0;
  }
}

// ---------- Serial ----------
void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  float v = sp < 0 ? NAN : line.substring(sp + 1).toFloat();

  if (cmd == "g")
    startRun();
  else if (cmd == "x") {
    if (state == RUNNING)
      stopRun("STOPPED", 4);
    else {
      state = IDLE;
      reverseNext = false;
      motorsCoast();
      Serial.println("Stopped");
    }
  } else if (cmd == "dist" && v >= 1 && v <= 1000) {
    TARGET_CM = v;
    Serial.print("dist = ");
    Serial.print(v);
    Serial.println(" cm");
  } else if (cmd == "rev" && v >= 1 && v <= 1000) {
    REVERSE_CM = v;
    Serial.print("rev = ");
    Serial.print(v);
    Serial.println(" cm");
  } else if (cmd == "cruise" && v > 0 && v <= MAX_DUTY + 0.001) {
    CRUISE = v;
    Serial.print("cruise = ");
    Serial.println(v, 3);
  } else if (cmd == "slow" && v >= 0 && v <= 1000) {
    SLOW_CM = v;
    Serial.print("slow = ");
    Serial.print(v);
    Serial.println(" cm");
  } else if (cmd == "slowduty" && v > 0 && v <= MAX_DUTY + 0.001) {
    SLOW_DUTY = v;
    Serial.print("slowduty = ");
    Serial.println(v, 3);
  } else if (cmd == "revcruise" && v > 0 && v <= MAX_DUTY + 0.001) {
    REV_CRUISE = v;
    Serial.print("revcruise = ");
    Serial.println(v, 3);
  } else if (cmd == "revslowduty" && v > 0 && v <= MAX_DUTY + 0.001) {
    REV_SLOW_DUTY = v;
    Serial.print("revslowduty = ");
    Serial.println(v, 3);
  } else if (cmd == "show") {
    Serial.print("dist = ");
    Serial.print(TARGET_CM);
    Serial.print(" cm, rev = ");
    Serial.print(REVERSE_CM);
    Serial.print(" cm, cruise = ");
    Serial.print(CRUISE, 3);
    Serial.print(", slow = ");
    Serial.print(SLOW_CM);
    Serial.print(" cm @ ");
    Serial.print(SLOW_DUTY, 3);
    Serial.print(", revcruise = ");
    Serial.print(REV_CRUISE, 3);
    Serial.print(", revslowduty = ");
    Serial.println(REV_SLOW_DUTY, 3);
  } else if (cmd.length()) {
    Serial.print("Bad command. Use: dist <1-1000 cm>, rev <1-1000 cm>, "
                 "cruise <0-");
    Serial.print(MAX_DUTY, 3);
    Serial.print(">, slow <0-1000 cm>, slowduty <0-");
    Serial.print(MAX_DUTY, 3);
    Serial.print(">, revcruise <0-");
    Serial.print(MAX_DUTY, 3);
    Serial.print(">, revslowduty <0-");
    Serial.print(MAX_DUTY, 3);
    Serial.println(">, g, x, show");
  }
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(buf);
      buf = "";
    } else if (buf.length() < 40)
      buf += c;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin(); // I2C0
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin(); // I2C1

  Wire.beginTransmission(PCA_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found"
                                             : "PCA9685 NOT found");

  int l = readAngle(Wire), r = readAngle(Wire1);
  Serial.println(l >= 0 ? "Left encoder OK" : "Left encoder NOT found");
  Serial.println(r >= 0 ? "Right encoder OK" : "Right encoder NOT found");
  lPrev = max(l, 0);
  rPrev = max(r, 0);

  pcaInit();
  motorsCoast();

  lastPressed = readPressed();
  lastChange = millis();
  Serial.println("Ready. Press start or send 'g'. Commands: dist <cm>, rev "
                 "<cm>, cruise <duty>, slow <cm>, slowduty <duty>, revcruise "
                 "<duty>, revslowduty <duty>, show, x");
}

void loop() {
  pollSerial();
  updateLed();

  // Keep tracking the wheels whenever the control loop isn't (it reads them
  // itself) so the angle history never goes stale, even while the car rolls
  // to a stop before reversing.
  if (state != RUNNING)
    updateEncoders();

  // Button: start a run
  bool pressed = readPressed();
  if (pressed != lastPressed && millis() - lastChange > 40) {
    lastChange = millis();
    lastPressed = pressed;
    if (pressed)
      startRun();
  }

  if (state == WAITING && millis() - pressTime >= START_DELAY_MS)
    beginRun();

  if (state == RUNNING && millis() - lastTick >= LOOP_MS)
    controlTick();

  if (state == BRAKING && millis() - brakeTime >= BRAKE_MS) {
    motorsCoast();
    if (reverseNext) {
      state = PAUSED;
      Serial.println("Waiting 2 s before reversing");
    } else
      state = IDLE;
  }

  // The wait is timed from the moment the car stopped (brakeTime).
  if (state == PAUSED && millis() - brakeTime >= PAUSE_MS) {
    reverseNext = false;
    beginLeg(-1, REVERSE_CM);
  }
}
