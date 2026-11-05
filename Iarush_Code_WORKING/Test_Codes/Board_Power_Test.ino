/*
  Pico 2 onboard LED blink test
  Arduino core: Earle F. Philhower III RP2040/RP2350 core

 Blinks the LED Light on the PICO, run code and place on driver to see if there
 are any issues

*/

void setup() { pinMode(LED_BUILTIN, OUTPUT); }

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
