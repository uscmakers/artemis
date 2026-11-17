import io
import time

import pytest

from artemis.io.tof import ToFReader, parse_tof_line


@pytest.mark.parametrize(
    "line, expected_m",
    [
        ("Distance: 1234 mm\n", 1.234),  # tof_distance_full.ino, tof_distance_pi.py
        ("850 mm  status=0\r\n", 0.850),  # tof_distance_lite.ino, good reading
        ("612", 0.612),  # bare number = mm
        ("Distance: 45.5 mm", 0.0455),
    ],
)
def test_parse_valid_lines(line, expected_m):
    assert parse_tof_line(line) == pytest.approx(expected_m)


@pytest.mark.parametrize(
    "line",
    ["Out of range", "850 mm  status=2", "VL53L4CX ready", "Ranging...", "Sensor found: model 0xEB, type 0xAA", ""],
)
def test_parse_rejects_non_readings(line):
    assert parse_tof_line(line) is None


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_reader_median_smooths_a_spike():
    clock = FakeClock()
    reader = ToFReader(smoothing_samples=5, clock=clock)
    assert reader.latest() is None
    for mm in (1000, 1002, 1950, 998, 1001):  # one bad spike
        reader.feed(f"Distance: {mm} mm")
        clock.t += 0.1
    r = reader.latest()
    assert r.distance_m == pytest.approx(1.001)
    assert r.raw_m == pytest.approx(1.001)
    assert r.age_s == pytest.approx(0.1)
    assert r.readings_per_s == pytest.approx(2.5)


def test_reader_out_of_range_and_staleness():
    clock = FakeClock()
    reader = ToFReader(clock=clock)
    reader.feed("Distance: 500 mm")
    reader.feed("Out of range")
    clock.t += 3.0
    r = reader.latest()
    assert r.out_of_range
    assert r.distance_m == pytest.approx(0.5)  # keeps the last good value
    assert r.age_s == pytest.approx(3.0)
    assert r.readings_per_s == 0.0


def test_reader_thread_reads_a_stream():
    reader = ToFReader().start(io.StringIO("VL53L4CX ready\nDistance: 700 mm\nDistance: 710 mm\n"))
    deadline = time.monotonic() + 2.0
    while reader.running and time.monotonic() < deadline:
        time.sleep(0.01)
    assert reader.latest().distance_m == pytest.approx(0.705)
    assert reader.error is None
