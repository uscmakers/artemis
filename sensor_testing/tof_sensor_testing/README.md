# ToF sensor testing (Adafruit VL53L4CX)

Arduino sketches for checking the VL53L4CX time-of-flight sensor. Open each folder in Arduino IDE and use Serial Monitor at **115200 baud**.

| Sketch | What it does | Board | Library |
|---|---|---|---|
| `tof_i2c_check` | Confirms the sensor is on I2C and reads its chip ID | Uno or anything | none |
| `tof_distance_lite` | Quick distance readings using the smaller VL53L4CD driver | Uno or anything | STM32duino VL53L4CD |
| `tof_distance_full` | Proper distance readings with the official driver | ESP32, Uno R4, Pico, etc. | STM32duino VL53L4CX |

## Wiring

| Sensor pin | Arduino Uno | ESP32 DevKit |
|---|---|---|
| VIN | 5V | 3.3V |
| GND | GND | GND |
| SDA | A4 | GPIO 21 |
| SCL | A5 | GPIO 22 |
| XSHUT | optional (A1 in code) | optional (set `XSHUT_PIN`) |
| GPIO, 3Vo | not connected | not connected |

## Notes

- **The official VL53L4CX driver does not fit on an Arduino Uno.** It needs about 53 KB of flash and 9.6 KB of RAM; the Uno has 32 KB and 2 KB. A Mega (8 KB RAM) won't fit it either.
- `tof_i2c_check` passed on an Uno: sensor found at 0x29, Model ID 0xEB, Module type 0xAA.
- `tof_distance_lite` is a workaround, not the official driver for this chip. Use it to confirm the sensor measures, not for real accuracy.
- Remove the protective film over the sensor before testing.
