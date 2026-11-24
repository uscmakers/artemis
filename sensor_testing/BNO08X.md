# GY-BNO08X on Raspberry Pi over I²C

Use the silkscreen labels on the purple GY-BNO08X board. Turn the Pi off before wiring. The Pi column uses **physical header pin numbers**.

| GY-BNO08X | Raspberry Pi | Purpose |
| --- | --- | --- |
| VCC | Pin 1 (3.3 V) | Power and logic supply |
| GND | Pin 6 (GND) | Common ground |
| SCL | Pin 18 (BCM GPIO24) | I²C clock |
| SDA | Pin 16 (BCM GPIO23) | I²C data |
| AD0 | GND | Address `0x4A`. Use 3.3 V for `0x4B`. |
| PS1, PS0 | GND | Select I²C at power-up |
| INT | Pin 11 (BCM GPIO17) | Active-low data ready |
| CS, RST, BOOT pad | Leave open | The reader requests a soft reset over I²C |

Use 3.3 V for VCC and AD0 high. The photo shows the board labels. The wiring follows CEVA’s pin functions and assumes the module populates the reset and boot pull-ups. If the board never starts, verify RST and BOOT idle high at 3.3 V. SDA and SCL each need a pull-up to 3.3 V. Verify the module’s pull-ups before adding external ones. [CEVA BNO08x datasheet, §1.2.2](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)

## Set up Raspberry Pi OS

The wiring uses a software I²C controller so it can honor clock stretching and leave the Pi’s standard I²C bus available. Add the following line under `[all]` in `/boot/firmware/config.txt` and reboot. On older images the file may be `/boot/config.txt`. [Raspberry Pi overlay reference](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README) · [CEVA I²C transaction details](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)

```ini
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2
```

On the Pi, from the repository root:

```bash
sudo apt install python3-smbus2 python3-gpiozero python3-lgpio
ls -l /dev/i2c-3
/usr/bin/python3 sensor_testing/bno08x.py
```

If AD0 is high, append `--address 0x4B`. Use `--help` to see the bus, interrupt pin and rate options. If opening the bus or GPIO fails with a permission error, check that your account has access to the Pi’s `i2c` and `gpio` devices.

## Read the display

The terminal lists firmware identification, raw IMU counts, raw gyroscope temperature counts, calibrated measurements, uncalibrated readings with bias, gravity and linear acceleration, rotation quaternions, and motion classifications and events. Each row shows the age of the latest report received by the Pi. For standard reports, accuracy is the sensor’s status and time offset is relative to that packet’s interrupt reference. The gyro-integrated stream uses a different report layout and carries quaternion and angular velocity values.

The script requests the reports at 5 Hz by default. Firmware can clamp report rates and omit optional features. Tap or shake the board to check event rows. An empty event row can mean the event has yet to occur, so use the firmware information and received data to establish what your unit delivers. If the screen updates but readings grow old, check the INT connection. If your terminal is shorter than the report list, use scrollback by redirecting output to a file and stop it with Ctrl+C.

Ambient temperature, pressure, light, humidity and proximity require separate environmental sensors. The raw gyro temperature field is a device count. [CEVA SH-2 reference manual, §6.5](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf)
