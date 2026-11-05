// =============================================================================
// PID distance tuner  -  Pico 2 (Earle Philhower core) + Waveshare Pico-Motor-
// Driver (PCA9685 @ 0x40) + two I2C MT6701 encoders (@ 0x06).
// Left encoder: I2C0 (GP20/GP21, shared with PCA9685). Right encoder: I2C1
// (GP2/GP3).
//
// Press the start button (GP15) or send 'g' to drive a set distance. Every gain
// and parameter can be changed from the Serial Monitor between runs (115200
// baud, newline-terminated, case-insensitive). Changes made during a run apply
// to the NEXT run, so each summary matches the parameters it actually used.
//
// SERIAL COMMANDS
//   kp <v>  ki <v>  kd <v>   gains (duty/mm, duty/(mm*s), duty/(mm/s))
//   kh <v>  khd <v>          heading gains (duty/mm, duty/(mm/s)) on L-R
//   mismatch steermax <duty>          max steering differential trim <0.5-1.5>
//   left-side duty multiplier (0.97 = left 3% slower) dist <cm> run distance
//   cruise <duty>            max duty, 0 .. 0.667
//   mind <duty>              minimum-duty floor while error > tol (stiction)
//   slew <duty/s>            output slew limit (rising output only)
//   tol <mm>                 position tolerance
//   izone <mm>               integrate only when |error| < izone
//   ilimit <duty>            clamp on the integral term (+/-)
//   dfilt <0-1>              speed/D low-pass coefficient (1 = no filtering)
//   timeout <s>              run timeout
//   g                        start a run (same as the button)
//   x                        emergency stop (brake, abort run, then coast)
//   show / defaults          print / reset all parameters
//   cal                      toggle encoder monitor (raw angle + wheel mm)
//   csv                      toggle per-sample log (summary always prints)
//   history                  print the last 10 run summaries
//   help                     list commands
// =============================================================================
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================ TUNABLE CONSTANTS ==============================
// ---- Geometry ----
const float COUNTS_PER_REV = 16384.0; // MT6701 I2C angle is 14-bit
const float WHEEL_DIAMETER_MM = 60.325;
const float GEAR_RATIO = 1.0; // sensor revs per wheel rev (1:1)

// ---- Encoders (MT6701 over I2C, index 0 = left, 1 = right) ----
const bool ENC_MONITOR_MODE = false; // start in encoder monitor ('cal') mode
const int ENC_SIGN[2] = {-1, 1}; // flip to make forward drive count positive
const int ENC_FAIL_ABORT_TICKS =
    10; // abort after this many failed reads in a row

// ---- Motors ----
const bool M1_INVERT = false; // swap IN1/IN2 for motor 1
const bool M2_INVERT = false; // swap IN1/IN2 for motor 2
// Which motor drives the LEFT wheel (the one read by the left encoder). If
// this is wrong, steering pushes the car further off line: flip it.
const bool LEFT_MOTOR_IS_M1 = true;
const float SUPPLY_V = 9.0;
const float MOTOR_MAX_V = 6.0;
const float MAX_DUTY = MOTOR_MAX_V / SUPPLY_V; // 0.667 hard cap on any duty

// ---- Timing ----
const uint32_t LOOP_HZ = 100;        // control loop rate
const uint32_t LOG_INTERVAL_MS = 20; // CSV sample period
const uint32_t BRAKE_MS = 300;       // active brake time after a run
const uint32_t DONE_HOLD_MS = 100;   // in tolerance + slow for this long
const float DONE_SPEED_MM_S = 20.0;  // "slow" threshold for the done check
const uint32_t BUTTON_DEBOUNCE_MS = 40;
const uint32_t START_DELAY_MS = 2000; // wait after button/'g' before driving

// ---- Safety ----
const float BACKWARD_ABORT_MM = 50.0; // abort if a wheel backs up this far

// ---- Parameter defaults (all adjustable at runtime) ----
const float DEF_KP = 0.004;      // duty per mm of error
const float DEF_KI = 0.0;        // duty per (mm*s)
const float DEF_KD = 0.0002;     // duty per (mm/s)
const float DEF_DIST_CM = 100.0; // run distance (stored internally in mm)
const float DEF_CRUISE = 0.2;
const float DEF_MIND = 0.0;
const float DEF_SLEW = 2.0; // duty/s
const float DEF_TOL_MM = 8.0;
const float DEF_IZONE_MM = 30.0;
const float DEF_ILIMIT = 0.10; // duty
const float DEF_DFILT = 0.3;
const float DEF_TIMEOUT_S = 10.0;
const float DEF_KH = 0.0001;      // steering: duty per mm of L-R mismatch
const float DEF_KHD = 0.000001;   // steering: duty per (mm/s) of mismatch rate
const float DEF_STEER_MAX = 0.08; // max steering differential (duty)
const float DEF_TRIM =
    0.97; // left-side duty multiplier (<1 slows left, fixes a right veer)

// ---- Pins / bus ----
const uint8_t PCA_ADDR = 0x40;
const uint8_t ENC_ADDR = 0x06;        // MT6701 default I2C address
const int SDA_PIN = 20, SCL_PIN = 21; // I2C0: PCA9685 + left encoder
const int SDA1_PIN = 2, SCL1_PIN = 3; // I2C1: right encoder
const uint32_t I2C_CLOCK_HZ = 100000; // tested speed; try 400000 if needed
const int BUTTON_PIN = 15;
const bool BUTTON_ACTIVE_LOW =
    false; // false = button outputs HIGH when pressed
const uint8_t M1_PWM = 6, M1_IN1 = 7, M1_IN2 = 8;
const uint8_t M2_PWM = 9, M2_IN1 = 10, M2_IN2 = 11;

// ---- Buffers ----
const int LOG_MAX = 3200; // samples; covers 60 s timeout at 50 Hz
const int HISTORY_N = 10;
// ============================================================================

const uint32_t LOOP_US = 1000000UL / LOOP_HZ;
const float MM_PER_COUNT =
    (3.14159265f * WHEEL_DIAMETER_MM) / (COUNTS_PER_REV * GEAR_RATIO);

// ------------------------------- Parameters ---------------------------------
struct Params {
  float kp, ki, kd, dist, cruise, mind, slew, tol, izone, ilimit, dfilt,
      timeoutS, kh, khd, steerMax, trim;
};
Params P; // edited by serial commands
Params R; // snapshot used by the active run

void loadDefaults() {
  P = {DEF_KP,       DEF_KI,     DEF_KD,        DEF_DIST_CM * 10.0f,
       DEF_CRUISE,   DEF_MIND,   DEF_SLEW,      DEF_TOL_MM,
       DEF_IZONE_MM, DEF_ILIMIT, DEF_DFILT,     DEF_TIMEOUT_S,
       DEF_KH,       DEF_KHD,    DEF_STEER_MAX, DEF_TRIM};
}

// ------------------------------- Run state ----------------------------------
enum RunState { IDLE, WAIT, RUN, BRAKE };
RunState state = IDLE;
uint32_t waitStartMs;
bool calMode = ENC_MONITOR_MODE;
bool csvOn = true;

uint16_t runNumber = 0;
uint32_t runStartUs, lastTickUs, nextTickUs;
uint32_t brakeStartMs;
const char *runStatus = "OK";

// Onboard LED shows why the last run stopped (so you can tell without USB).
// Solid on = waiting / running. After the run it blinks N times, pauses,
// repeats:  1 = OK   2 = TIMEOUT   3 = ABORT-BACKWARD (check ENC_SIGN)
//           4 = ABORT-ENCODER (I2C read failures)   5 = ESTOP
int ledCode = 0;

int prevRaw[2], encFail[2]; // last raw angle, failed reads in a row
float unwrapCounts[2], wheelMM[2], peakMM[2];
float distMM, prevDist, speedMMs, iTerm, prevOut, maxDist;
float prevHeadingErr, maxHeadErr; // L-R wheel mismatch (mm)
int32_t doneStartMs;
int32_t tFirstMs, tEnterMs, tSettleMs;
uint32_t tTotalMs;
bool inBand;

// ------------------------------- Log / history ------------------------------
struct LogRow {
  uint32_t t;
  float target, dist, err, speed, out, p, i, d, heading, steer;
};
LogRow logBuf[LOG_MAX];
int logCount = 0;
int32_t lastLogMs;
bool logFull = false;

struct Summary {
  uint16_t run;
  float kp, ki, kd, dist, cruise, mind, kh;
  float finalErr, overshoot, headErr, maxHeadErr;
  int32_t tFirst, tSettle;
  uint32_t tTotal;
  const char *status;
};
Summary hist[HISTORY_N];
int histCount = 0, histNext = 0;

// ------------------------------- Helpers ------------------------------------
void pf(const char *fmt, ...) {
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ------------------------------- PCA9685 ------------------------------------
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
    setChannel(ch, 0, 0x1000); // full off
  else if (duty >= 1)
    setChannel(ch, 0x1000, 0); // full on
  else
    setChannel(ch, 0, (uint16_t)(duty * 4095));
}

void setDigital(uint8_t ch, bool high) {
  if (high)
    setChannel(ch, 0x1000, 0);
  else
    setChannel(ch, 0, 0x1000);
}

void pcaInit() {
  pcaWrite(0x00, 0x10); // sleep so prescale can be set
  pcaWrite(0xFE, 5);    // ~1 kHz PWM
  pcaWrite(0x00, 0x20); // wake, auto-increment
  delay(5);
  pcaWrite(0x01, 0x04); // totem-pole outputs
}

// ------------------------------- Motors -------------------------------------
enum DriveMode { M_COAST, M_FWD, M_BRAKE };
DriveMode driveMode = M_COAST;

// Forward with separate left/right duty (each clamped to MAX_DUTY). IN pins are
// only rewritten when the mode changes; the PWM channels are rewritten every
// call.
void motorsForward(float left, float right) {
  left = clampf(left, 0.0f, MAX_DUTY);
  right = clampf(right, 0.0f, MAX_DUTY);
  if (driveMode != M_FWD) {
    setDigital(M1_IN1, !M1_INVERT);
    setDigital(M1_IN2, M1_INVERT);
    setDigital(M2_IN1, !M2_INVERT);
    setDigital(M2_IN2, M2_INVERT);
    driveMode = M_FWD;
  }
  setDuty(M1_PWM, LEFT_MOTOR_IS_M1 ? left : right);
  setDuty(M2_PWM, LEFT_MOTOR_IS_M1 ? right : left);
}

// Active (short) brake: both IN pins high. Motor terminals are shorted, so PWM
// at full does not put supply voltage on the motor.
void motorsBrake() {
  setDigital(M1_IN1, true);
  setDigital(M1_IN2, true);
  setDigital(M2_IN1, true);
  setDigital(M2_IN2, true);
  setDuty(M1_PWM, 1.0);
  setDuty(M2_PWM, 1.0);
  driveMode = M_BRAKE;
}

void motorsCoast() {
  setDuty(M1_PWM, 0);
  setDuty(M2_PWM, 0);
  setDigital(M1_IN1, false);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, false);
  setDigital(M2_IN2, false);
  driveMode = M_COAST;
}

// ------------------------------- Encoders -----------------------------------
TwoWire &encBus(int idx) { return idx == 0 ? Wire : Wire1; }

// Returns 14-bit angle 0..16383, or -1 on error.
int readAngle(TwoWire &w) {
  w.beginTransmission(ENC_ADDR);
  w.write(0x03); // ANGLE_H (bits 13:6); 0x04 holds bits 5:0 in its top 6 bits
  if (w.endTransmission(false) != 0)
    return -1;
  if (w.requestFrom((uint8_t)ENC_ADDR, (size_t)2) < 2)
    return -1;
  int hi = w.read();
  int lo = w.read();
  return (hi << 6) | (lo >> 2);
}

// Zero both wheels. Returns false if an encoder does not answer.
bool zeroEncoders() {
  bool ok = true;
  for (int i = 0; i < 2; i++) {
    int a = readAngle(encBus(i));
    if (a < 0)
      ok = false;
    prevRaw[i] = a < 0 ? 0 : a;
    encFail[i] = 0;
    unwrapCounts[i] = 0;
    wheelMM[i] = 0;
    peakMM[i] = 0;
  }
  return ok;
}

// Accumulate the change since the last read (assumes < half a turn between
// samples), then convert to mm. A failed read keeps the last value.
void updateEncoders() {
  for (int i = 0; i < 2; i++) {
    int a = readAngle(encBus(i));
    if (a < 0) {
      encFail[i]++;
      continue;
    }
    encFail[i] = 0;
    int d = a - prevRaw[i];
    if (d > 8192)
      d -= 16384;
    if (d < -8192)
      d += 16384;
    prevRaw[i] = a;
    unwrapCounts[i] += d * ENC_SIGN[i];
    wheelMM[i] = unwrapCounts[i] * MM_PER_COUNT;
  }
  distMM = (wheelMM[0] + wheelMM[1]) * 0.5f;
}

// ------------------------------- Logging ------------------------------------
void logSample(uint32_t tMs, float err, float out, float p, float i, float d,
               float steer) {
  if (!csvOn)
    return;
  if (logCount >= LOG_MAX) {
    logFull = true;
    return;
  }
  logBuf[logCount++] = {tMs,  R.dist, distMM, err, speedMMs,
                        out,  p,      i,      d,   wheelMM[0] - wheelMM[1],
                        steer};
}

void flushLog() {
  for (int k = 0; k < logCount; k++) {
    const LogRow &r = logBuf[k];
    pf("%lu,%.1f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f\n",
       (unsigned long)r.t, r.target, r.dist, r.err, r.speed, r.out, r.p, r.i,
       r.d, r.heading, r.steer);
  }
  if (logFull)
    Serial.println("WARNING: log buffer filled, tail not recorded");
  logCount = 0;
}

// Takes a history slot (not a Summary&) because the Arduino auto-prototype
// generator would place a Summary& prototype before the struct definition.
void printSummary(int slot) {
  const Summary &s = hist[slot];
  pf("RUN %u | kp=%.5f ki=%.5f kd=%.5f kh=%.4f dist=%.1fcm cruise=%.3f "
     "mind=%.3f | final_err=%.1f overshoot=%.1f head_err=%.1f "
     "max_head=%.1f t_first=",
     s.run, s.kp, s.ki, s.kd, s.kh, s.dist / 10.0f, s.cruise, s.mind,
     s.finalErr, s.overshoot, s.headErr, s.maxHeadErr);
  if (s.tFirst >= 0)
    pf("%ldms", (long)s.tFirst);
  else
    Serial.print("never");
  Serial.print(" settle=");
  if (s.tSettle >= 0)
    pf("%ldms", (long)s.tSettle);
  else
    Serial.print("never");
  pf(" total=%lums | %s\n", (unsigned long)s.tTotal, s.status);
}

void printHistory() {
  if (histCount == 0) {
    Serial.println("No runs yet.");
    return;
  }
  int oldest = (histNext - histCount + HISTORY_N) % HISTORY_N;
  for (int k = 0; k < histCount; k++)
    printSummary((oldest + k) % HISTORY_N);
}

void printParams() {
  pf("kp=%.5f ki=%.5f kd=%.5f\n", P.kp, P.ki, P.kd);
  pf("dist=%.1f cm  cruise=%.3f  mind=%.3f  slew=%.2f duty/s\n", P.dist / 10.0f,
     P.cruise, P.mind, P.slew);
  pf("tol=%.1f mm  izone=%.1f mm  ilimit=%.3f  dfilt=%.3f  timeout=%.1f s\n",
     P.tol, P.izone, P.ilimit, P.dfilt, P.timeoutS);
  pf("kh=%.4f  khd=%.5f  steermax=%.3f  trim=%.3f\n", P.kh, P.khd, P.steerMax,
     P.trim);
  pf("csv=%s  cal=%s  MAX_DUTY=%.3f  loop=%lu Hz  mm/count=%.4f\n",
     csvOn ? "on" : "off", calMode ? "on" : "off", MAX_DUTY,
     (unsigned long)LOOP_HZ, MM_PER_COUNT);
}

// ------------------------------- Run control --------------------------------
void startRun() {
  if (state != IDLE) {
    Serial.println("Run already active - ignoring.");
    return;
  }
  if (calMode) {
    Serial.println("Encoder monitor is on - send 'cal' to leave it first.");
    return;
  }
  state = WAIT;
  waitStartMs = millis();
  pf("Starting in %lu ms ('x' cancels)\n", (unsigned long)START_DELAY_MS);
}

// Called once START_DELAY_MS has passed: zero, snapshot parameters and drive.
void beginRun() {
  if (!zeroEncoders()) {
    Serial.println(
        "REJECTED: an encoder is not responding on I2C - check wiring.");
    state = IDLE;
    return;
  }
  R = P; // freeze parameters for this run
  runNumber++;
  runStatus = "OK";
  distMM = prevDist = speedMMs = 0;
  iTerm = prevOut = maxDist = 0;
  prevHeadingErr = maxHeadErr = 0;
  doneStartMs = -1;
  tFirstMs = tEnterMs = tSettleMs = -1;
  tTotalMs = 0;
  inBand = false;
  logCount = 0;
  logFull = false;
  lastLogMs = -(int32_t)LOG_INTERVAL_MS;

  pf("# RUN %u START kp=%.5f ki=%.5f kd=%.5f dist=%.1fcm\n", runNumber, R.kp,
     R.ki, R.kd, R.dist / 10.0f);
  if (csvOn)
    Serial.println(
        "t_ms,target_mm,dist_mm,error_mm,speed_mm_s,output_duty,p_term,i_term,"
        "d_term,heading_mm,steer");

  uint32_t now = micros();
  runStartUs = lastTickUs = now;
  nextTickUs = now + LOOP_US;
  state = RUN;
  motorsForward(0, 0);
}

// Stop driving: active brake now, finish the BRAKE phase in controlTick().
void finishRun(const char *status, uint32_t tMs) {
  motorsBrake();
  runStatus = status;
  tTotalMs = tMs;
  tSettleMs = inBand ? tEnterMs : -1;
  brakeStartMs = tMs;
  state = BRAKE;
}

void endRun() {
  motorsCoast();
  state = IDLE;
  if (!strcmp(runStatus, "OK"))
    ledCode = 1;
  else if (!strcmp(runStatus, "TIMEOUT"))
    ledCode = 2;
  else if (!strcmp(runStatus, "ABORT-BACKWARD"))
    ledCode = 3;
  else if (!strcmp(runStatus, "ABORT-ENCODER"))
    ledCode = 4;
  else
    ledCode = 5;
  Summary &s = hist[histNext];
  s.run = runNumber;
  s.kp = R.kp;
  s.ki = R.ki;
  s.kd = R.kd;
  s.kh = R.kh;
  s.headErr = wheelMM[0] - wheelMM[1];
  s.maxHeadErr = maxHeadErr;
  s.dist = R.dist;
  s.cruise = R.cruise;
  s.mind = R.mind;
  s.finalErr = R.dist - distMM;
  s.overshoot = maxDist > R.dist ? maxDist - R.dist : 0;
  s.tFirst = tFirstMs;
  s.tSettle = tSettleMs;
  s.tTotal = tTotalMs;
  s.status = runStatus;
  int slot = histNext;
  histNext = (histNext + 1) % HISTORY_N;
  if (histCount < HISTORY_N)
    histCount++;

  flushLog();
  Serial.print("SUMMARY ");
  printSummary(slot);
}

void emergencyStop() {
  if (state == RUN) {
    Serial.println("EMERGENCY STOP");
    finishRun("ESTOP", (micros() - runStartUs) / 1000);
  } else if (state == WAIT) {
    state = IDLE;
    Serial.println("Start cancelled.");
  } else if (state == IDLE) {
    motorsCoast();
    Serial.println("Motors coasting (no run active).");
  }
}

// One control iteration. Called at LOOP_HZ while state != IDLE.
void controlTick(uint32_t nowUs) {
  float dt = (nowUs - lastTickUs) * 1e-6f;
  lastTickUs = nowUs;
  if (dt < 0.0005f || dt > 0.1f)
    dt = 1.0f / LOOP_HZ;

  updateEncoders();
  uint32_t tMs = (nowUs - runStartUs) / 1000;
  float err = R.dist - distMM;

  // Speed: derivative of measured distance, low-passed.
  float raw = (distMM - prevDist) / dt;
  prevDist = distMM;
  speedMMs += R.dfilt * (raw - speedMMs);
  if (distMM > maxDist)
    maxDist = distMM;

  if (state == BRAKE) {
    logSample(tMs, err, 0, 0, 0, 0, 0);
    if (tMs - brakeStartMs >= BRAKE_MS)
      endRun();
    return;
  }

  // ---- RUN ----
  for (int i = 0; i < 2; i++) {
    if (encFail[i] >= ENC_FAIL_ABORT_TICKS) {
      pf("ABORT: %s encoder stopped responding on I2C\n",
         i == 0 ? "left" : "right");
      logSample(tMs, err, 0, 0, 0, 0, 0);
      finishRun("ABORT-ENCODER", tMs);
      return;
    }
    if (wheelMM[i] > peakMM[i])
      peakMM[i] = wheelMM[i];
    if (peakMM[i] - wheelMM[i] > BACKWARD_ABORT_MM) {
      pf("ABORT: %s encoder moved backward %.0f mm (check ENC_SIGN)\n",
         i == 0 ? "left" : "right", peakMM[i] - wheelMM[i]);
      logSample(tMs, err, 0, 0, 0, 0, 0);
      finishRun("ABORT-BACKWARD", tMs);
      return;
    }
  }

  // Tolerance-band bookkeeping for the summary.
  bool nowIn = fabsf(err) < R.tol;
  if (nowIn && !inBand) {
    tEnterMs = tMs;
    if (tFirstMs < 0)
      tFirstMs = tMs;
  }
  inBand = nowIn;

  // PID. D acts on the measurement: d_term = -kd * speed.
  float pTerm = R.kp * err;
  float dTerm = -R.kd * speedMMs;
  if (fabsf(err) < R.izone) {
    iTerm += R.ki * err * dt;
    iTerm = clampf(iTerm, -R.ilimit, R.ilimit);
  }

  float cmd = clampf(pTerm + iTerm + dTerm, 0.0f, R.cruise);
  float maxUp = prevOut + R.slew * dt; // slew limits rising output only
  if (cmd > maxUp)
    cmd = maxUp;
  if (err > R.tol && cmd < R.mind)
    cmd = R.mind; // stiction floor
  cmd = clampf(cmd, 0.0f, MAX_DUTY);
  prevOut = cmd;

  // Steering: PD on the left-right wheel distance mismatch. Left ahead ->
  // steer > 0 -> left slows, right speeds up.
  float headingErr = wheelMM[0] - wheelMM[1];
  float headingRate = (headingErr - prevHeadingErr) / dt;
  prevHeadingErr = headingErr;
  if (fabsf(headingErr) > fabsf(maxHeadErr))
    maxHeadErr = headingErr;

  float steer = R.kh * headingErr + R.khd * headingRate;
  steer = clampf(steer, -R.steerMax, R.steerMax);
  if (cmd <= 0)
    steer = 0; // never spin one wheel while the car is stopped

  float l = (cmd - steer) * R.trim; // trim: fixed left/right bias correction
  float r = cmd + steer;
  // Don't exceed the 6 V cap: shift both down so the differential survives.
  float over = fmaxf(fmaxf(l, r) - MAX_DUTY, 0.0f);
  l = fmaxf(l - over, 0.0f);
  r = fmaxf(r - over, 0.0f);
  motorsForward(l, r);

  if ((int32_t)(tMs - lastLogMs) >= (int32_t)LOG_INTERVAL_MS) {
    lastLogMs = tMs;
    logSample(tMs, err, cmd, pTerm, iTerm, dTerm, steer);
  }

  // Done: inside tolerance and slow for DONE_HOLD_MS.
  if (nowIn && fabsf(speedMMs) < DONE_SPEED_MM_S) {
    if (doneStartMs < 0)
      doneStartMs = tMs;
    if ((int32_t)tMs - doneStartMs >= (int32_t)DONE_HOLD_MS) {
      finishRun("OK", tMs);
      return;
    }
  } else {
    doneStartMs = -1;
  }

  if (tMs >= (uint32_t)(R.timeoutS * 1000.0f)) {
    finishRun("TIMEOUT", tMs);
  }
}

// ------------------------------- Encoder monitor ----------------------------
// Push the car forward by hand: both wheel distances should go positive.
// If one goes negative, flip its ENC_SIGN.
void calTick() {
  static uint32_t lastRead = 0, lastPrint = 0;
  if (millis() - lastRead < 10)
    return; // sample fast so wraps are tracked
  lastRead = millis();
  updateEncoders();
  if (millis() - lastPrint < 100)
    return;
  lastPrint = millis();
  pf("L raw=%5d %s mm=%8.1f | R raw=%5d %s mm=%8.1f\n", prevRaw[0],
     encFail[0] ? "FAIL" : "ok  ", wheelMM[0], prevRaw[1],
     encFail[1] ? "FAIL" : "ok  ", wheelMM[1]);
}

// ------------------------------- Serial commands ----------------------------
// Parse `arg` as a float within [lo, hi] and store it in `dst`. Prints a clear
// message and returns false on rejection. Tiny slack lets "0.667" pass for the
// 0.6667 duty cap (the value is then clamped back into range).
bool setRange(const char *name, const char *arg, float lo, float hi,
              const char *unit, float &dst) {
  char *end;
  float v = strtof(arg, &end);
  if (*arg == '\0' || *end != '\0' || !isfinite(v)) {
    pf("REJECTED: '%s' is not a number for %s\n", arg, name);
    return false;
  }
  if (v < lo - 0.0006f || v > hi + 0.0006f) {
    pf("REJECTED: %s must be %g .. %g %s (got %g)\n", name, lo, hi, unit, v);
    return false;
  }
  dst = clampf(v, lo, hi);
  pf("%s = %g %s%s\n", name, dst, unit,
     state != IDLE ? "  (applies to next run)" : "");
  return true;
}

void handleCommand(char *line) {
  for (char *c = line; *c; c++)
    *c = tolower((unsigned char)*c);
  while (*line == ' ' || *line == '\t')
    line++;
  char *arg = line;
  while (*arg && *arg != ' ' && *arg != '\t')
    arg++;
  if (*arg)
    *arg++ = '\0';
  while (*arg == ' ' || *arg == '\t')
    arg++;
  char *e = arg + strlen(arg); // trim trailing blanks
  while (e > arg && (e[-1] == ' ' || e[-1] == '\t'))
    *--e = '\0';
  const char *cmd = line;
  if (*cmd == '\0')
    return;

  if (!strcmp(cmd, "kp"))
    setRange("kp", arg, 0, 1, "", P.kp);
  else if (!strcmp(cmd, "ki"))
    setRange("ki", arg, 0, 1, "", P.ki);
  else if (!strcmp(cmd, "kd"))
    setRange("kd", arg, 0, 1, "", P.kd);
  else if (!strcmp(cmd, "kh"))
    setRange("kh", arg, 0, 0.1, "", P.kh);
  else if (!strcmp(cmd, "khd"))
    setRange("khd", arg, 0, 0.01, "", P.khd);
  else if (!strcmp(cmd, "steermax"))
    setRange("steermax", arg, 0, 0.3, "duty", P.steerMax);
  else if (!strcmp(cmd, "trim"))
    setRange("trim", arg, 0.5, 1.5, "", P.trim);
  else if (!strcmp(cmd, "dist")) {
    float cm;
    if (setRange("dist", arg, 1, 500, "cm", cm))
      P.dist = cm * 10.0f;
  } else if (!strcmp(cmd, "cruise")) {
    float v;
    if (setRange("cruise", arg, 0, MAX_DUTY, "duty", v)) {
      if (v < P.mind)
        pf("REJECTED: cruise %g is below mind %g - lower mind first\n", v,
           P.mind);
      else
        P.cruise = v;
    }
  } else if (!strcmp(cmd, "mind")) {
    float v;
    if (setRange("mind", arg, 0, MAX_DUTY, "duty", v)) {
      if (v > P.cruise)
        pf("REJECTED: mind %g is above cruise %g - raise cruise first\n", v,
           P.cruise);
      else
        P.mind = v;
    }
  } else if (!strcmp(cmd, "slew"))
    setRange("slew", arg, 0.1, 100, "duty/s", P.slew);
  else if (!strcmp(cmd, "tol"))
    setRange("tol", arg, 0.5, 100, "mm", P.tol);
  else if (!strcmp(cmd, "izone"))
    setRange("izone", arg, 0, 1000, "mm", P.izone);
  else if (!strcmp(cmd, "ilimit"))
    setRange("ilimit", arg, 0, 0.5, "duty", P.ilimit);
  else if (!strcmp(cmd, "dfilt"))
    setRange("dfilt", arg, 0.01, 1, "", P.dfilt);
  else if (!strcmp(cmd, "timeout"))
    setRange("timeout", arg, 1, 60, "s", P.timeoutS);
  else if (!strcmp(cmd, "g"))
    startRun();
  else if (!strcmp(cmd, "x"))
    emergencyStop();
  else if (!strcmp(cmd, "show"))
    printParams();
  else if (!strcmp(cmd, "defaults")) {
    loadDefaults();
    Serial.println(state != IDLE ? "Defaults restored (apply to next run)."
                                 : "Defaults restored.");
    printParams();
  } else if (!strcmp(cmd, "cal")) {
    if (state != IDLE) {
      Serial.println("REJECTED: cannot toggle encoder monitor during a run.");
    } else {
      calMode = !calMode;
      if (calMode) {
        motorsCoast();
        zeroEncoders();
        Serial.println("ENCODER MONITOR ON - push the car forward by hand; "
                       "send 'cal' to exit.");
      } else {
        Serial.println("ENCODER MONITOR OFF");
      }
    }
  } else if (!strcmp(cmd, "csv")) {
    csvOn = !csvOn;
    Serial.println(csvOn ? "csv log ON" : "csv log OFF (summary only)");
  } else if (!strcmp(cmd, "history"))
    printHistory();
  else if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    Serial.println(
        "kp ki kd kh khd steermax trim dist cruise mind slew tol izone "
        "ilimit dfilt "
        "timeout <value> | g x show defaults cal csv history");
  } else {
    pf("Unknown command '%s' (try 'help')\n", cmd);
  }
}

void pollSerial() {
  static char buf[64];
  static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = '\0';
      n = 0;
      handleCommand(buf);
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ------------------------------- Button -------------------------------------
// Debounced. Disarmed at boot if held; re-arms after one release.
bool btnPressed = false, btnRaw = false, btnArmed = true;
uint32_t btnChangeMs = 0;

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  return BUTTON_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void pollButton() {
  bool raw = readPressed();
  uint32_t now = millis();
  if (raw != btnRaw) {
    btnRaw = raw;
    btnChangeMs = now;
  }
  if (raw != btnPressed && now - btnChangeMs >= BUTTON_DEBOUNCE_MS) {
    btnPressed = raw;
    if (!btnPressed) {
      btnArmed = true;
    } else if (btnArmed) {
      if (state == IDLE)
        startRun();
      // while a run is active the press is simply ignored
    }
  }
}

// ------------------------------- Arduino entry ------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_HZ);

  uint32_t t0 = millis(); // give the USB serial a moment so the banner shows
  while (!Serial && millis() - t0 < 2000)
    delay(10);

  Wire.beginTransmission(PCA_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found"
                                             : "PCA9685 NOT found");
  pcaInit();
  motorsCoast();

  Serial.println(readAngle(Wire) >= 0 ? "Left encoder OK"
                                      : "Left encoder NOT found");
  Serial.println(readAngle(Wire1) >= 0 ? "Right encoder OK"
                                       : "Right encoder NOT found");
  zeroEncoders();

  loadDefaults();

  btnRaw = btnPressed = readPressed();
  btnChangeMs = millis();
  if (btnPressed) {
    btnArmed = false;
    Serial.println("Start button held at boot - ignored until released.");
  }

  Serial.println(
      "PID tuner ready. Type 'help' or 'show'. Start: button or 'g'.");
  if (calMode)
    Serial.println("ENCODER MONITOR ON - push the car forward by hand.");
}

void loop() {
  pollSerial();
  pollButton();

  if (calMode) {
    calTick();
    return;
  }

  if (state == WAIT) {
    if (millis() - waitStartMs >= START_DELAY_MS)
      beginRun();
  } else if (state != IDLE) {
    uint32_t now = micros();
    if ((int32_t)(now - nextTickUs) >= 0) {
      nextTickUs += LOOP_US;
      if ((int32_t)(now - nextTickUs) >= 0)
        nextTickUs = now + LOOP_US; // overrun
      controlTick(now);
    }
  }
}
