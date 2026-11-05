// Pico 2 + Waveshare Pico-Motor-Driver (PCA9685 @ 0x40) + analog MT6701
// encoders Press start -> drive forward 6 inches (averaged encoder distance) ->
// brake.
#include <Wire.h>

// ===== SET THESE =====
const float WHEEL_DIAMETER_MM = 65.0; // <-- measure your wheel and change this
const float TARGET_MM = 152.4;        // 6 inches
const float POWER = 0.05;             // 5% duty
const bool ACTIVE_LOW = false; // false = button outputs HIGH when pressed

// Encoder ADC range (run with CALIBRATION_MODE = true, spin each wheel by hand)
const bool CALIBRATION_MODE = false;
const int L_ADC_MIN = 0, L_ADC_MAX = 4095;
const int R_ADC_MIN = 0, R_ADC_MAX = 4095;

// Flip to -1 if an encoder counts backward while driving forward
const int L_SIGN = 1;
const int R_SIGN =
    -1; // wheels are usually mirrored; flip if distance goes negative

const unsigned long TIMEOUT_MS = 10000; // safety stop
// =====================

const uint8_t PCA_ADDR = 0x40;
const int SDA_PIN = 20, SCL_PIN = 21;
const int BUTTON_PIN = 15;
const int L_ENC_PIN = 26, R_ENC_PIN = 27;

const uint8_t M1_PWM = 6, M1_IN1 = 7, M1_IN2 = 8;
const uint8_t M2_PWM = 9, M2_IN1 = 10, M2_IN2 = 11;

bool running = false;
bool lastPressed = false;
unsigned long lastChange = 0, startTime = 0;

float lPrev, rPrev;         // previous angle fraction 0..1
float lRevs = 0, rRevs = 0; // unwrapped revolutions

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

void motorsRun() {
  setDigital(M1_IN1, true);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, true);
  setDigital(M2_IN2, false);
  setDuty(M1_PWM, POWER);
  setDuty(M2_PWM, POWER);
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
  pcaWrite(0xFE, 5); // ~1 kHz
  pcaWrite(0x00, 0x20);
  delay(5);
  pcaWrite(0x01, 0x04);
}

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  return ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

int readAdcAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++)
    sum += analogRead(pin);
  return sum / 8;
}

float adcToFrac(int adc, int mn, int mx) {
  float f = (float)(adc - mn) / (float)(mx - mn);
  if (f < 0)
    f = 0;
  if (f > 1)
    f = 1;
  return f;
}

// Adds the change since last read, handling the wrap at 0/1
float updateRevs(float &prev, float now, int sign) {
  float d = now - prev;
  if (d > 0.5)
    d -= 1.0;
  if (d < -0.5)
    d += 1.0;
  prev = now;
  return d * sign;
}

void updateEncoders() {
  float l = adcToFrac(readAdcAvg(L_ENC_PIN), L_ADC_MIN, L_ADC_MAX);
  float r = adcToFrac(readAdcAvg(R_ENC_PIN), R_ADC_MIN, R_ADC_MAX);
  lRevs += updateRevs(lPrev, l, L_SIGN);
  rRevs += updateRevs(rPrev, r, R_SIGN);
}

float distanceMM() {
  float circ = 3.14159265 * WHEEL_DIAMETER_MM;
  return ((lRevs + rRevs) / 2.0) * circ;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(BUTTON_PIN, ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.beginTransmission(PCA_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found"
                                             : "PCA9685 NOT found");

  pcaInit();
  motorsCoast();

  lPrev = adcToFrac(readAdcAvg(L_ENC_PIN), L_ADC_MIN, L_ADC_MAX);
  rPrev = adcToFrac(readAdcAvg(R_ENC_PIN), R_ADC_MIN, R_ADC_MAX);

  lastPressed = readPressed();
  lastChange = millis();
}

void loop() {
  if (CALIBRATION_MODE) {
    Serial.print("L ADC: ");
    Serial.print(readAdcAvg(L_ENC_PIN));
    Serial.print("  R ADC: ");
    Serial.println(readAdcAvg(R_ENC_PIN));
    delay(100);
    return;
  }

  updateEncoders();

  // Button start
  bool pressed = readPressed();
  if (pressed != lastPressed && millis() - lastChange > 40) {
    lastChange = millis();
    lastPressed = pressed;
    if (pressed && !running) {
      Serial.println("BUTTON PRESSED - starting in 2s");
      delay(2000);
      lRevs = 0;
      rRevs = 0;
      startTime = millis();
      running = true;
      motorsRun();
      Serial.println("RUN");
    }
  }

  // Stop conditions
  if (running) {
    float d = distanceMM();
    if (d >= TARGET_MM) {
      motorsBrake();
      running = false;
      Serial.print("DONE at ");
      Serial.print(d);
      Serial.println(" mm");
    } else if (millis() - startTime > TIMEOUT_MS) {
      motorsBrake();
      running = false;
      Serial.println("TIMEOUT");
    }
  }
}