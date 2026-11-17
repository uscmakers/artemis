"""Read distance from the VL53L4CX time-of-flight sensor.

The team's sensor scripts print one reading per line (see sensor_testing/tof_sensor_testing):
    "Distance: 1234 mm"        tof_distance_full.ino, tof_distance_pi.py
    "1234 mm  status=0"        tof_distance_lite.ino (status 0 = good)
    "Out of range"             no valid target
A bare number ("1234") is also accepted as millimeters.

`ToFReader` reads those lines in a background thread from a USB serial port or any text
stream (e.g. stdin piped from `ssh pi python3 -u tof_distance_pi.py`) and keeps a median-
smoothed latest value. Needs `pyserial` only for serial ports.
"""

from __future__ import annotations

import glob
import re
import statistics
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass

_MM_RE = re.compile(r"(-?\d+(?:\.\d+)?)\s*mm\b", re.IGNORECASE)
_BARE_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*$")
_STATUS_RE = re.compile(r"status\s*=\s*(-?\d+)", re.IGNORECASE)
SERIAL_PORT_PATTERNS = ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*")
RATE_WINDOW_S = 2.0


def parse_tof_line(line: str) -> float | None:
    """Distance in meters from one line of sensor output, or None if it holds no valid reading."""
    status = _STATUS_RE.search(line)
    if status and int(status.group(1)) != 0:
        return None
    match = _MM_RE.search(line) or _BARE_RE.match(line)
    if not match:
        return None
    mm = float(match.group(1))
    return mm / 1000.0 if mm > 0 else None


def find_serial_port() -> str | None:
    """First USB serial port that looks like a microcontroller, or None."""
    for pattern in SERIAL_PORT_PATTERNS:
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    return None


def serial_lines(port: str, baud: int = 115200) -> Iterator[str]:
    import serial  # optional dependency: pip install pyserial

    with serial.Serial(port, baud, timeout=1.0) as ser:
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("utf-8", errors="replace")


@dataclass(frozen=True)
class ToFReading:
    distance_m: float  # median of the recent readings, as the sensor reports it (no offset applied)
    raw_m: float  # the most recent single reading
    age_s: float  # time since the most recent valid reading
    readings_per_s: float
    out_of_range: bool  # the sensor's latest line said it had no valid target


class ToFReader:
    """Collects sensor lines (from a thread, or by calling `feed`) into a smoothed latest reading."""

    def __init__(self, smoothing_samples: int = 5, clock: Callable[[], float] = time.monotonic, label: str = ""):
        self.label = label
        self._clock = clock
        self._recent: deque[float] = deque(maxlen=max(1, smoothing_samples))
        self._times: deque[float] = deque()
        self._last_t: float | None = None
        self._out_of_range = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None

    def feed(self, line: str) -> None:
        distance_m = parse_tof_line(line)
        now = self._clock()
        with self._lock:
            if distance_m is None:
                if "out of range" in line.lower() or _STATUS_RE.search(line):
                    self._out_of_range = True
                return
            self._out_of_range = False
            self._recent.append(distance_m)
            self._times.append(now)
            self._last_t = now
            while self._times and now - self._times[0] > RATE_WINDOW_S:
                self._times.popleft()

    def latest(self) -> ToFReading | None:
        with self._lock:
            if self._last_t is None:
                return None
            now = self._clock()
            recent_count = sum(1 for t in self._times if now - t <= RATE_WINDOW_S)
            return ToFReading(
                distance_m=statistics.median(self._recent),
                raw_m=self._recent[-1],
                age_s=now - self._last_t,
                readings_per_s=recent_count / RATE_WINDOW_S,
                out_of_range=self._out_of_range,
            )

    def start(self, lines: Iterable[str]) -> ToFReader:
        """Read `lines` in a daemon thread until they run out or `stop()` is called."""

        def run() -> None:
            try:
                for line in lines:
                    if self._stop.is_set():
                        break
                    self.feed(line)
            except Exception as exc:  # serial unplugged, port busy, ...
                self.error = str(exc)

        self._thread = threading.Thread(target=run, name="tof-reader", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
