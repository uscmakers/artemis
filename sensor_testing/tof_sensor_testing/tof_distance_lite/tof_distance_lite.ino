// VL53L4CX quick distance test using the lightweight VL53L4CD driver.
// The L4CD shares the L4CX's chip ID, and its driver fits on an Arduino Uno.
// Not the official driver for this sensor: fine for "does it measure",
// not for accurate ARTEMIS readings. Use tof_distance_full for that.
// Library: "STM32duino VL53L4CD" (Arduino Library Manager)

#include <Wire.h>
#include <vl53l4cd_class.h>

VL53L4CD tof(&Wire, A1);   // A1 = XSHUT, not wired, that's fine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  tof.begin();
  tof.InitSensor();
  tof.VL53L4CD_SetRangeTiming(100, 0);   // 100 ms per reading
  tof.VL53L4CD_StartRanging();
  Serial.println("Ranging...");
}

void loop() {
  uint8_t ready = 0;
  VL53L4CD_Result_t r;

  do { tof.VL53L4CD_CheckForDataReady(&ready); } while (!ready);
  tof.VL53L4CD_ClearInterrupt();
  tof.VL53L4CD_GetResult(&r);

  Serial.print(r.distance_mm);
  Serial.print(" mm  status=");
  Serial.println(r.range_status);   // 0 = good reading
}
