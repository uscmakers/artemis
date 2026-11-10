// VL53L4CX quick check: is the sensor on I2C and responding?
// No extra libraries, fits on an Arduino Uno.
// Expected output: "Sensor found at 0x29", Model ID 0xEB, Module type 0xAA

#include <Wire.h>

#define ADDR 0x29   // sensor's default I2C address

// read one byte from a 16-bit register
uint8_t readReg(uint16_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // check if anything answers at 0x29
  Wire.beginTransmission(ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("Sensor NOT found. Check wiring.");
    return;
  }
  Serial.println("Sensor found at 0x29");

  // read chip ID registers
  Serial.print("Model ID: 0x");
  Serial.println(readReg(0x010F), HEX);
  Serial.print("Module type: 0x");
  Serial.println(readReg(0x0110), HEX);
}

void loop() {}
