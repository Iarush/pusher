// Pico 2 + Waveshare Pico-Motor-Driver (PCA9685 @ 0x40)
// Press start button -> both motors run at 25%. Press again -> stop.

// Use this code to see if the motors work, click button to start and stop
#include <Wire.h>

const uint8_t PCA_ADDR = 0x40;
const int SDA_PIN = 20;
const int SCL_PIN = 21;
const int BUTTON_PIN = 15;

// false = module outputs HIGH when pressed, true = outputs LOW when pressed
const bool ACTIVE_LOW = false;

// PCA9685 channels (from wiring notes)
const uint8_t M1_PWM = 6, M1_IN1 = 7, M1_IN2 = 8;
const uint8_t M2_PWM = 9, M2_IN1 = 10, M2_IN2 = 11;

const float POWER = 0.25; // 25% duty

bool running = false;
bool lastPressed = false;
unsigned long lastChange = 0;

void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setChannel(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t base = 0x06 + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(base);
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

void motorsRun() {
  setDigital(M1_IN1, true);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, true);
  setDigital(M2_IN2, false);
  setDuty(M1_PWM, POWER);
  setDuty(M2_PWM, POWER);
}

void motorsStop() {
  setDuty(M1_PWM, 0);
  setDuty(M2_PWM, 0);
  setDigital(M1_IN1, false);
  setDigital(M1_IN2, false);
  setDigital(M2_IN1, false);
  setDigital(M2_IN2, false);
}

void pcaInit() {
  pcaWrite(0x00, 0x10); // sleep so prescale can be set
  pcaWrite(0xFE, 5);    // ~1 kHz PWM
  pcaWrite(0x00, 0x20); // wake, auto-increment
  delay(5);
  pcaWrite(0x01, 0x04); // totem-pole outputs
}

bool readPressed() {
  bool level = digitalRead(BUTTON_PIN);
  return ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  Wire.beginTransmission(PCA_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found"
                                             : "PCA9685 NOT found");

  pcaInit();
  motorsStop();
  lastPressed = readPressed(); // ignore whatever state the button is in at boot
  lastChange = millis();
}

void loop() {
  bool pressed = readPressed();

  if (pressed != lastPressed && millis() - lastChange > 40) { // debounce
    lastChange = millis();
    lastPressed = pressed;
    if (pressed) { // on press edge
      running = !running;
      if (running)
        motorsRun();
      else
        motorsStop();
      Serial.println(running ? "RUN 25%" : "STOP");
    }
  }
}