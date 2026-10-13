# Artemis engine v0

Ballistic engine for the Artemis aim-assist bow: bow model (draw -> launch speed),
3D point-mass arrow flight (gravity, quadratic drag, wind), and an aim solver that
reports pitch/yaw error from the current aim. See `SPEC.md` for the full design.

## Install

Python 3.11+.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy matplotlib pyyaml pytest scipy
```

`scipy` is only used by the tests as a cross-check; the engine itself needs only
`numpy` and `pyyaml` (plus `matplotlib` for the explorations).

## Run the tests

```bash
python -m pytest
```

## Run the explorations

Run from the repo root. Every script takes `--preset <name>` (default: `active_preset`)
and `--no-show` (save the PNG without opening a window). Plots are saved to `explore/out/`.
Key results are written up in [explore/FINDINGS.md](explore/FINDINGS.md).

```bash
python -m explore.trajectories        # side view of arcs, drag vs vacuum, 15 m target
python -m explore.sight_tape          # required pitch vs range, 3-40 m, plus table
python -m explore.sensitivity         # impact shift per error source, and an error budget
python -m explore.monte_carlo         # 2000 noisy-sensor shots on a target face (--range, --shots)
python -m explore.explorer            # interactive sliders: draw, pitch, yaw, range, wind, Cd
python -m explore.solver_timing       # solve_aim cold vs warm-start timing (--n)
```

For example, compare the suction-cup setup:

```bash
python -m explore.sight_tape --preset toy_suction
```

Sensor noise levels for the Monte Carlo are constants at the top of `explore/monte_carlo.py`.

## Layout

```
artemis/config.py          config.yaml -> dataclasses, inches/pounds -> SI
artemis/physics/bow.py     force-draw curve -> stored energy -> launch speed
artemis/physics/flight.py  RK4 integrator, trajectory to the target plane
artemis/solver/aim.py      solve required pitch/yaw, compute aim error
explore/                   exploration scripts (see below)
tests/                     pytest suite
```

## Conventions

- SI units everywhere, units in names (`range_m`, `pitch_deg`, `v_launch_mps`).
- World frame: origin at the nock, **x** downrange, **y** to the archer's left, **z** up.
- **pitch** + = up, **yaw** + = left. Errors are `required - current`:
  `pitch_err_deg > 0` means raise, `yaw_err_deg > 0` means turn left.
- All bow/arrow numbers live in `config.yaml` as named presets. They are placeholders
  until the real bow is measured.

## Quick use

```python
from artemis import load_config
from artemis.physics.bow import launch_speed_mps
from artemis.solver.aim import Target, solve_aim, aim_error

params = load_config(preset="recurve_30lb")
v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
sol = solve_aim(Target(range_m=15.0), v, params)          # pitch ~1.56 deg
err = aim_error(0.9, 0.0, params.bow.nominal_draw_m, Target(15.0), params, warm_start=sol)
```
