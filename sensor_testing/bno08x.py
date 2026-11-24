"""Live SH-2 diagnostics for the GY-BNO08X on Raspberry Pi I²C."""

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Report:
    name: str
    layout: str
    fields: tuple[str, ...]
    scales: tuple[int, ...] = ()
    unit: str = ""
    event: bool = False

    @property
    def size(self) -> int:
        return struct.calcsize("<" + self.layout)

    def describe(self, payload: bytes) -> str:
        values = struct.unpack("<" + self.layout, payload)
        if self.scales:
            return "  ".join(
                f"{field}={value / (1 << scale):+.4f}"
                for field, value, scale in zip(self.fields, values, self.scales)
            ) + (f"  [{self.unit}]" if self.unit else "")
        return "  ".join(
            f"{field}={value}" for field, value in zip(self.fields, values)
        )


XYZ = ("x", "y", "z")
BIAS = ("bias x", "bias y", "bias z")
QUAT = ("qx", "qy", "qz", "qw")
ANGLE_ERROR = ("heading error (rad)",)
ACCURACY = ("unreliable", "low", "medium", "high")
STABILITY = ("unknown", "on table", "stationary", "stable", "in motion")
ACTIVITY = (
    "unknown", "in vehicle", "on bicycle", "on foot", "still",
    "tilting", "walking", "running", "on stairs",
)
REPORTS = {
    0x01: Report("Accelerometer", "3h", XYZ, (8,) * 3, "m/s²"),
    0x02: Report("Calibrated gyroscope", "3h", XYZ, (9,) * 3, "rad/s"),
    0x03: Report("Calibrated magnetometer", "3h", XYZ, (4,) * 3, "µT"),
    0x04: Report("Linear acceleration", "3h", XYZ, (8,) * 3, "m/s²"),
    0x05: Report("Rotation vector", "4hH", QUAT + ANGLE_ERROR, (14,) * 4 + (12,)),
    0x06: Report("Gravity", "3h", XYZ, (8,) * 3, "m/s²"),
    0x07: Report("Uncalibrated gyroscope", "6h", XYZ + BIAS, (9,) * 6, "rad/s"),
    0x08: Report("Game rotation vector", "4h", QUAT, (14,) * 4),
    0x09: Report("Geomagnetic rotation vector", "4hH", QUAT + ANGLE_ERROR, (14,) * 4 + (12,)),
    0x0F: Report("Uncalibrated magnetometer", "6h", XYZ + BIAS, (4,) * 6, "µT"),
    0x10: Report("Tap detector", "B", ("flags",), event=True),
    0x11: Report("Step counter", "II", ("latency (µs)", "steps"), event=True),
    0x12: Report("Significant motion", "H", ("motion",), event=True),
    0x13: Report("Stability classifier", "Bx", ("class",)),
    0x14: Report("Raw accelerometer", "3h2xI", XYZ + ("sensor timestamp",)),
    0x15: Report("Raw gyroscope", "4hI", XYZ + ("temperature (raw)", "sensor timestamp")),
    0x16: Report("Raw magnetometer", "3h2xI", XYZ + ("sensor timestamp",)),
    0x18: Report("Step detector", "I", ("latency (µs)",), event=True),
    0x19: Report("Shake detector", "H", ("axes",), event=True),
    0x1A: Report("Flip detector", "H", ("flip",), event=True),
    0x1B: Report("Pickup detector", "H2x", ("pickup",), event=True),
    0x1C: Report("Stability detector", "H", ("stability",), event=True),
    0x1E: Report("Personal activity", "12B", ("page", "class"), event=True),
    0x1F: Report("Sleep detector", "Bx", ("state",), event=True),
    0x20: Report("Tilt detector", "H", ("tilt",), event=True),
    0x21: Report("Pocket detector", "H", ("pocket",), event=True),
    0x22: Report("Circle detector", "H", ("circle",), event=True),
    0x28: Report("AR/VR rotation vector", "4hH", QUAT + ANGLE_ERROR, (14,) * 4 + (12,)),
    0x29: Report("AR/VR game rotation vector", "4h", QUAT, (14,) * 4),
    0x2A: Report(
        "Gyro-integrated rotation vector", "7h",
        QUAT + ("ωx (rad/s)", "ωy (rad/s)", "ωz (rad/s)"),
        (14,) * 4 + (10,) * 3,
    ),
}


def label(names: tuple[str, ...], value: int) -> str:
    return names[value] if value < len(names) else f"code {value}"


def describe(report_id: int, data: bytes) -> str:
    if report_id == 0x10:
        flags = data[0]
        axes = [
            ("+" if flags & (2 << bit) else "-") + axis
            for bit, axis in zip((0, 2, 4), "XYZ")
            if flags & (1 << bit)
        ]
        tap = "double tap" if flags & 0x40 else "single tap"
        return f"{tap}  axes={', '.join(axes) or 'unspecified'}  flags=0x{flags:02X}"
    if report_id == 0x13:
        return label(STABILITY, data[0])
    if report_id == 0x19:
        flags, = struct.unpack("<H", data)
        axes = [axis for bit, axis in enumerate("XYZ") if flags & (1 << bit)]
        return f"shake  axes={', '.join(axes) or 'unspecified'}  flags=0x{flags:04X}"
    if report_id == 0x1E:
        page, state, *confidence = data
        page &= 0x7F
        scores = "  ".join(
            f"{label(ACTIVITY, page * 10 + index)}={score}%"
            for index, score in enumerate(confidence)
            if page * 10 + index < len(ACTIVITY) or score
        )
        return f"{label(ACTIVITY, state)}  page={page}  {scores}"
    return REPORTS[report_id].describe(data)


def decode_reports(channel: int, payload: bytes):
    """Yield report ID, formatted data and a page number for activity reports."""
    if channel == 5:
        size = REPORTS[0x2A].size
        for offset in range(0, len(payload) - size + 1, size):
            yield 0x2A, describe(0x2A, payload[offset:offset + size]), 0
        return
    if channel not in (3, 4):
        return

    cursor = 0
    reference = 0
    while cursor < len(payload):
        report_id = payload[cursor]
        report = REPORTS.get(report_id)
        if report_id in (0xFA, 0xFB):  # Timestamp rebase / base timestamp
            if cursor + 5 > len(payload):
                return
            delta, = struct.unpack_from("<i", payload, cursor + 1)
            reference = -delta if report_id == 0xFB else reference + delta
            cursor += 5
            continue
        if report_id == 0xEF:  # Flush completed
            cursor += 2
            continue
        if report is None or report_id == 0x2A or cursor + 4 + report.size > len(payload):
            # The next boundary cannot be determined without this report's length.
            return
        _, sequence, status, delay_low = payload[cursor:cursor + 4]
        data = payload[cursor + 4:cursor + 4 + report.size]
        delay = ((status & 0xFC) << 6) | delay_low
        offset_us = (reference + delay) * 100
        value = describe(report_id, data)
        yield (
            report_id,
            f"{value}  |  accuracy={ACCURACY[status & 3]}  seq={sequence}  time offset={offset_us:+d} µs",
            data[0] & 0x7F if report_id == 0x1E else 0,
        )
        cursor += 4 + report.size


class BNO08x:
    def __init__(self, bus, address, interrupt, message):
        self.bus = bus
        self.address = address
        self.interrupt = interrupt
        self.message = message
        self.sequence = [0] * 6

    def send(self, channel: int, payload: bytes) -> None:
        packet = struct.pack("<HBB", len(payload) + 4, channel, self.sequence[channel])
        self.bus.i2c_rdwr(self.message.write(self.address, packet + payload))
        self.sequence[channel] = (self.sequence[channel] + 1) % 256

    def read(self, length: int) -> bytes:
        message = self.message.read(self.address, length)
        self.bus.i2c_rdwr(message)
        return bytes(message)

    def receive(self) -> tuple[int, bytes] | None:
        # BNO085 can hold SCL low if polled when there is no data.
        if not self.interrupt.is_active:
            return None
        header = self.read(4)
        wire_length, channel, _ = struct.unpack("<HBB", header)
        length = wire_length & 0x7FFF
        if length == 0:
            return None
        if not 4 <= length <= 4096:
            raise OSError(f"Unexpected SHTP length {length}")
        if length == 4:
            return None
        packet = self.read(length)
        # A header-only read leaves the payload pending. The next transaction
        # can mark it as a continuation and can carry a different sequence.
        packet_length, packet_channel, _ = struct.unpack_from("<HBB", packet)
        if (packet_length & 0x7FFF) != length or packet_channel != channel:
            raise OSError(
                f"Inconsistent SHTP length or channel: header [{header.hex(' ')}], "
                f"payload header [{packet[:4].hex(' ')}]"
            )
        # Normal BNO08x sensor cargoes fit in one I²C payload read.
        if wire_length & 0x8000:
            return None
        return channel, packet[4:]

    def packets_for(self, seconds: float):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            packet = self.receive()
            if packet is None:
                time.sleep(0.001)
            else:
                yield packet

    def initialize(self, interval_us: int) -> list[str]:
        # Drain any startup advertisement or reports from a previous session.
        for _ in self.packets_for(0.1):
            pass
        self.send(1, b"\x01")  # Executable channel: soft reset
        for _ in self.packets_for(0.6):
            pass
        self.send(2, b"\xf9\x00")
        products = []
        for channel, data in self.packets_for(0.5):
            if channel == 2 and len(data) >= 16 and data[0] == 0xF8:
                _, cause, major, minor, part, build, patch = struct.unpack_from("<4BIIH", data)
                products.append(
                    f"Firmware {major}.{minor}.{patch}  part {part}  build {build}  reset cause {cause}"
                )
        if not products:
            raise OSError("No product-ID response. Check INT, SDA/SCL, PS0/PS1 and AD0 wiring.")
        for report_id in REPORTS:
            # Enable all nine standard activity classes.
            specific = 0x1FF if report_id == 0x1E else 0
            self.send(2, struct.pack("<BBBHIII", 0xFD, report_id, 0, 0,
                                     interval_us, 0, specific))
        return products


def display(sensor: BNO08x, addr: int, bus: int, interval: float, products: list[str]) -> None:
    latest: dict[int, dict[int, tuple[str, float]]] = {}
    while True:
        for channel, payload in sensor.packets_for(interval):
            if channel == 1 and payload == b"\x01":
                raise OSError("Sensor reset while streaming. Restart the reader.")
            for report_id, value, page in decode_reports(channel, payload):
                latest.setdefault(report_id, {})[page] = (value, time.monotonic())
        now = time.monotonic()
        lines = [f"GY-BNO08X  |  I²C bus {bus}  |  address 0x{addr:02X}  |  Ctrl+C to exit",
                 *products, ""]
        for report_id, report in REPORTS.items():
            pages = latest.get(report_id)
            if pages:
                values = [f"{value}  ({now - received:.1f}s ago)"
                          for _, (value, received) in sorted(pages.items())]
                status = " / ".join(values)
            else:
                status = "awaiting event or firmware support" if report.event else "no report received"
            lines.append(f"0x{report_id:02X}  {report.name:31} {status}")
        prefix = "\033[H\033[2J" if sys.stdout.isatty() else ""
        print(prefix + "\n".join(lines), flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=3, help="Linux I²C bus (default: 3)")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x4A,
                        choices=(0x4A, 0x4B), metavar="{0x4A,0x4B}")
    parser.add_argument("--int-gpio", type=int, default=17, help="BCM GPIO for INT (default: 17)")
    parser.add_argument("--sample-hz", type=float, default=5, help="Requested report rate (default: 5)")
    parser.add_argument("--refresh-hz", type=float, default=2, help="Display updates/sec (default: 2)")
    args = parser.parse_args(argv)
    if args.bus < 0 or args.int_gpio < 0:
        parser.error("bus and GPIO must be nonnegative")
    for name in ("sample_hz", "refresh_hz"):
        rate = getattr(args, name)
        if not math.isfinite(rate) or not 0.1 <= rate <= 20:
            parser.error(f"--{name.replace('_', '-')} must be between 0.1 and 20")

    try:
        from gpiozero import DigitalInputDevice
        from gpiozero.exc import GPIOZeroError
        from smbus2 import SMBus, i2c_msg
    except ImportError:
        print("Install python3-smbus2, python3-gpiozero and python3-lgpio on the Pi.", file=sys.stderr)
        return 1

    try:
        with SMBus(args.bus) as bus, DigitalInputDevice(args.int_gpio, pull_up=True) as interrupt:
            sensor = BNO08x(bus, args.address, interrupt, i2c_msg)
            products = sensor.initialize(round(1_000_000 / args.sample_hz))
            display(sensor, args.address, args.bus, 1 / args.refresh_hz, products)
    except KeyboardInterrupt:
        return 0
    except (OSError, GPIOZeroError) as exc:
        print(f"BNO08x on I²C bus {args.bus}, address 0x{args.address:02X}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
