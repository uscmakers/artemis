import pytest

from artemis.config import M_PER_IN, N_PER_LBF, load_config, preset_names


def test_active_preset_loads_in_si():
    params = load_config()
    assert params.preset_name == "recurve_30lb"
    assert params.bow.curve.brace_m == pytest.approx(7.5 * M_PER_IN)
    assert params.bow.curve.force_n[-1] == pytest.approx(30.0 * N_PER_LBF)
    assert params.bow.nominal_draw_m == pytest.approx(28 * M_PER_IN)
    assert params.flight.wind_mps == (0.0, 0.0, 0.0)


def test_preset_override_by_name():
    assert "toy_suction" in preset_names()
    params = load_config(preset="toy_suction")
    assert params.preset_name == "toy_suction"
    assert params.flight.arrow_diameter_m == pytest.approx(0.025)


def test_unknown_preset_raises():
    with pytest.raises(KeyError):
        load_config(preset="longbow_9000")


def test_with_flight_copies():
    params = load_config()
    windy = params.with_flight(wind_mps=[0, 2, 0], drag=False)
    assert windy.flight.wind_mps == (0.0, 2.0, 0.0)
    assert windy.flight.drag_k_per_m == 0.0
    assert params.flight.drag_k_per_m > 0.0
