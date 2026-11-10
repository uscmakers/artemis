// Adafruit VL53L4CX - print current distance (mm) to Serial
// Library: "STM32duino VL53L4CX" (install via Arduino Library Manager)

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define XSHUT_PIN A1   // change to whatever pin XSHUT is wired to

VL53L4CX tof(&Wire, XSHUT_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();           // ESP32: Wire.begin(SDA_PIN, SCL_PIN) if not using defaults

  tof.begin();
  tof.VL53L4CX_Off();     // reset via XSHUT
  tof.InitSensor(0x12);   // default I2C address (0x29 in 7-bit)
  tof.VL53L4CX_StartMeasurement();

  Serial.println("VL53L4CX ready");
}

void loop() {
  VL53L4CX_MultiRangingData_t data;
  uint8_t ready = 0;
  int status;

  // wait for a new reading
  do {
    status = tof.VL53L4CX_GetMeasurementDataReady(&ready);
  } while (!ready);

  if (status == 0) {
    status = tof.VL53L4CX_GetMultiRangingData(&data);

    // sensor can see multiple objects; print the closest valid one
    int closest = -1;
    for (int i = 0; i < data.NumberOfObjectsFound; i++) {
      if (data.RangeData[i].RangeStatus == 0) {   // 0 = valid reading
        int d = data.RangeData[i].RangeMilliMeter;
        if (closest < 0 || d < closest) closest = d;
      }
    }

    if (closest >= 0) {
      Serial.print("Distance: ");
      Serial.print(closest);
      Serial.println(" mm");
    } else {
      Serial.println("Out of range");
    }

    tof.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}
