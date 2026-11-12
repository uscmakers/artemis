"""VL53L4CX distance readout for the Raspberry Pi.

Prints the sensor's current distance in mm, about 10 times a second.

Uses Adafruit's VL53L4CD driver. The VL53L4CX reports the same chip ID
(0xEB / 0xAA) and runs fine on it, but in this mode max range is about
1.3 m, not the 6 m the L4CX can do with ST's full C driver.

Setup (once, on the Pi):
    sudo raspi-config nonint do_i2c 0
    python3 -m venv ~/artemis-venv
    source ~/artemis-venv/bin/activate
    pip install adafruit-blinka adafruit-circuitpython-vl53l4cd

Run:
    source ~/artemis-venv/bin/activate
    python sensor_testing/tof_sensor_testing/tof_distance_pi.py

Wiring (matches the team pin table): VIN -> pin 1 (3.3V), GND -> pin 9,
SDA -> pin 3, SCL -> pin 5. GPIO (pin 13) and XSHUT (pin 15) are optional
and not used here. SDA/SCL are shared with the BNO08x and TFS20-L.
Check with `i2cdetect -y 1`: the ToF shows up at 0x29.
"""

import board
import adafruit_vl53l4cd

TIMING_BUDGET_MS = 100  # time per reading; longer = steadier, slower


def main():
    i2c = board.I2C()  # pins 3 (SDA) and 5 (SCL)
    tof = adafruit_vl53l4cd.VL53L4CD(i2c)

    model_id, module_type = tof.model_info
    print(f"Sensor found: model 0x{model_id:02X}, type 0x{module_type:02X}")

    tof.inter_measurement = 0  # back-to-back readings
    tof.timing_budget = TIMING_BUDGET_MS
    tof.start_ranging()
    print("Ranging... (Ctrl+C to stop)")

    try:
        while True:
            while not tof.data_ready:
                pass
            tof.clear_interrupt()
            distance_mm = round(tof.distance * 10)  # driver reports cm
            print(f"Distance: {distance_mm} mm")
    except KeyboardInterrupt:
        pass
    finally:
        tof.stop_ranging()
        print("\nStopped.")


if __name__ == "__main__":
    main()
