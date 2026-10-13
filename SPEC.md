# Artemis Engine v0: Spec for Claude Code

## How to use this file

Put this file in an empty folder as `SPEC.md`, open Claude Code there, and say:

> Read SPEC.md. Build Phase 1 step by step, running the tests after each step. Then build the explorations in Phase 2 and tell me what they show. Don't start Phase 3 unless I ask.

---

## 1. Project context

Read this whole section before writing code. It explains what the engine is for and the decisions already made, so the code fits where the project is headed.

### What Artemis is

Artemis is a student team project: a physical bow-and-arrow set with a built-in aim assistant. While the archer is at full draw, the system tells them through audio or visual cues how to adjust three things to hit the bullseye:

- **Pitch**: the bow's angle up or down
- **Yaw**: the bow's left/right orientation
- **Draw**: how far back the arrow is pulled, which sets how hard the shot is

The software's job, in the team's words: *develop the physics simulation for the arrow's path, and the algorithm for working out corrections to the user's state, as recorded by on-bow sensors and environmental target factors.*

### Team

Three subteams:

- **Electrical / sensing**: picks and implements the sensors (camera, gyroscope, tension/draw sensing) and integrates them with the microcontroller and power
- **Software**: physics simulation and the correction algorithm (this repo)
- **Mechanical**: bow-and-arrow design, either built from wood or bought and modified, and mounting the electronics ergonomically on the frame

### Roadmap

**Fall semester (now)**
- Finished physical bow and arrow frame
- Sensor configuration chosen and sensor hardware developed
- **Software physics simulations developed** (this spec is the start of that)

**Spring semester**
- Sensors integrated with the physical frame
- Sensor fusion developed, sensors integrated with software
- Higher precision in the physics simulation and arrow path predictions

**Stretch goal**: accurately shoot a moving target.

### Open decisions that affect the software

- **Build vs buy the bow.** Not decided yet. The team is weighing more control (build) against reliability and time saved (buy). Either way, software only needs the bow's measured force-draw curve, so the code must treat the bow as data in `config.yaml`, never as hardcoded physics.
- **Bow type.** The pitch deck shows a compound bow. Compound bows have a "let-off" curve: draw force rises to a peak and then drops near full draw. Recurves rise steadily. The force-draw table approach handles both, as long as interpolation doesn't assume the curve is monotonic.
- **Arrow type.** The deck shows both real arrows and suction-cup arrows. These fly completely differently (roughly 50 m/s vs 15 m/s, with very different drag), so the config has a preset for each.

### Design decisions already made

These came out of earlier design work. The engine doesn't implement most of them yet, but don't contradict them.

**Full system pipeline** (most of this is NOT in scope for v0):

```
IMU + draw sensor (200 Hz, microcontroller) ─┐
                                             ├─> bow state ───┐
camera + rangefinder (30 Hz, Raspberry Pi) ──┴─> target state ┴─> ballistic solver (50 Hz) ─> aim error ─> audio cues
                                                                                                         └─> shot log
```

- **Hardware split.** A microcontroller reads the IMU and draw sensor at about 200 Hz and streams packets over UART to a Raspberry Pi. The Pi runs the camera, estimation, solver, and cues in a loop at about 50 Hz. Core engine code should stay plain Python + numpy so it runs on the Pi.
- **The camera is mounted boresighted**, bolted parallel to the arrow. Image center is where the arrow points, so the target's pixel offset from center gives aim error in yaw and pitch directly, with no absolute heading needed.
- **Pitch and roll** come from the IMU's accelerometer and gyro through a complementary filter (gravity is the absolute reference).
- **Yaw** comes from the gyro, corrected by the camera's target offset each frame. **The magnetometer is not used for aiming.** It's only good to ±2–5° near the metal and electronics on a bow, while aiming needs about 0.3°, and absolute heading isn't needed anyway.
- **Gyro bias** gets calibrated at startup by holding the bow still for about 2 seconds.
- **Range** comes from a ToF/lidar rangefinder, with the target's apparent ring size as a cross-check.
- **Target detection** uses classic OpenCV: color threshold on the target rings, then an ellipse fit. No ML at first.
- **Draw** is measured as a length (distance sensor from riser to string), which is easier to mount than a load cell.
- **The solver** works backwards from the target. It simulates the arrow and bisects on pitch until it hits, then reports `required - current` as the error.
- **Cues** are mainly audio, since the archer's eyes are on the target. They use a deadband (about 0.3°) so noise doesn't make them flicker, and switch to a "hold" tone when every error is inside it.
- **Every shot gets logged** (state at release plus where it actually hit). The model starts out wrong; drag coefficient, bow efficiency, and camera alignment get tuned against these logs. That calibration loop *is* the spring "higher precision" goal.
- **For the moving-target goal**, track the target's velocity with a Kalman filter and lead it by the arrow's time of flight, which this engine already computes.

### Example of one full frame of data

For intuition about what the later system passes around (numbers from the `recurve_30lb` preset):

```
IMU packet:     accel_mps2 [0.82, -0.36, 9.77], gyro_dps [0.4, -1.1, 3.2], draw_mm 712
Vision:         target_px [332, 231] in a 640x480 frame, range_m 15.1
Bow state:      pitch 4.8°, roll -2.1°, draw 0.712 m, v_launch ~53 m/s
Target state:   target 1.2° right and 0.9° above the arrow axis, 15.0 m away
Solver:         needs ~1.5° of holdover above target, currently 0.9° below it
                -> raise 2.4°, turn right 1.2°, predicted miss 63 cm low / 31 cm right
                   (in this repo's sign convention: pitch_err_deg = +2.4, yaw_err_deg = -1.2)
Cue:            "raise" tone + "right" beeps, until all errors < 0.3° -> "hold"
```

### What this version is

**v0 is only the ballistic engine plus tools to explore it.** No hardware, no camera, no real sensors. The goal is to start experimenting: a physics model and solver the team can play with, test, and build intuition from. It doesn't need to be the final architecture, but it should be clean enough that the later stages in section 7 can plug into it.

---

## 2. Scope

### In scope (v0)
- Bow model: draw length -> stored energy -> arrow launch speed
- Flight model: 3D point-mass arrow with gravity, quadratic drag, and optional wind
- Solver: given a target, find the pitch and yaw that hit it
- Aim error: given where the bow is currently pointing, report pitch/yaw error and predicted impact point
- Exploration scripts and an interactive explorer
- Tests that check the physics against hand-derivable answers

### Out of scope (v0)
- Real sensor I/O, serial, camera, OpenCV
- Attitude filtering, sensor fusion
- Audio output
- Arrow flex, fletching spin, arrow-rest geometry
- Effect of bow roll (cant) on the trajectory. Roll can be accepted as an input and ignored, with a `TODO` comment
- Moving targets

---

## 3. Tech and conventions

- Python 3.11+
- Dependencies: `numpy`, `matplotlib`, `pyyaml`, `pytest`. `scipy` is allowed **only in tests** as a cross-check. Core code should run without it (it may eventually run on a Raspberry Pi).
- Use `dataclasses` and type hints. Keep physics functions pure (no globals, no hidden state).
- **SI units everywhere inside the code.** Put units in variable names: `range_m`, `pitch_deg`, `v_launch_mps`, `mass_kg`. Angles are degrees at the API boundary and radians internally; convert in one place.
- All bow and arrow constants live in `config.yaml`. No magic numbers in physics code.
- Include a `README.md` that says how to install, run tests, and run each exploration.
- If the folder is a git repo, commit after each step with a clear message.

### Coordinate frame (source of truth)

Right-handed world frame, origin at the arrow's nock at launch:

- **x**: downrange, toward the target
- **y**: to the archer's left
- **z**: up

Angles:

- **pitch**: rotation above horizontal, positive = arrow points up
- **yaw**: rotation about +z, positive = arrow points **left**
- **roll**: rotation about the arrow axis, positive = top of bow tilts right (accepted, ignored in v0)

Errors are always `required - current`. So `pitch_err_deg > 0` means "raise the bow" and `yaw_err_deg > 0` means "turn left". The cue layer (later) translates signs into words.

---

## 4. Repo layout

```
artemis/
  __init__.py
  config.py           # load config.yaml into dataclasses, convert units to SI
  physics/
    __init__.py
    bow.py            # force-draw curve, stored energy, launch speed
    flight.py         # RK4 integrator, trajectory to target plane
  solver/
    __init__.py
    aim.py            # solve required pitch/yaw, compute aim error
explore/
  trajectories.py
  sight_tape.py
  sensitivity.py
  monte_carlo.py
  explorer.py         # interactive sliders
  solver_timing.py
tests/
  test_bow.py
  test_flight.py
  test_solver.py
config.yaml
README.md
SPEC.md
```

---

## 5. Phase 1: the engine

### Step 1. Config

`config.yaml` holds named presets so we can compare setups. Start with these two. **All values are placeholders** until we measure the real bow; that's expected.

```yaml
active_preset: recurve_30lb

environment:
  gravity_mps2: 9.81
  air_density_kgpm3: 1.2
  wind_mps: [0.0, 0.0, 0.0]     # world frame [x, y, z]; +y blows toward the archer's left

presets:
  recurve_30lb:
    # Force-draw curve: what a luggage scale reads at each draw length.
    # draw_in = distance from grip pivot to string. First point is brace height (string at rest).
    force_draw:
      draw_in:  [7.5, 12, 16, 20, 24, 28]
      force_lb: [0.0, 6.6, 12.4, 18.3, 24.1, 30.0]
    efficiency: 0.80              # fraction of stored energy that goes into the arrow
    arrow_mass_kg: 0.020          # ~310 grains
    arrow_diameter_m: 0.0075      # shaft diameter, used for drag area
    drag_coefficient: 2.0         # placeholder, tune from real shots later
    nominal_draw_in: 28
    launch_height_m: 1.5          # nock height above ground at launch

  toy_suction:
    # Rough guess for a toy bow with suction-cup arrows. Very uncertain.
    force_draw:
      draw_in:  [6, 10, 14, 18, 20]
      force_lb: [0.0, 2.3, 4.6, 6.9, 8.0]
    efficiency: 0.40
    arrow_mass_kg: 0.025
    arrow_diameter_m: 0.025       # suction cup dominates drag
    drag_coefficient: 1.0
    nominal_draw_in: 20
    launch_height_m: 1.5

solver:
  dt_s: 0.001                     # integration step
  pitch_bracket_deg: [-15.0, 45.0]  # low-arc search range
  tolerance_m: 0.0005             # stop when vertical miss < 0.5 mm
  max_iterations: 50
```

`config.py` loads this into dataclasses, converts inches and pounds to meters and newtons, and exposes the active preset. Allow overriding the preset by name.

### Step 2. Bow model (`physics/bow.py`)

- `stored_energy_j(draw_m, curve) -> float`: integrate force over draw from brace height to `draw_m` using the trapezoid rule on the force-draw table, with linear interpolation for draw lengths between table points. Draw beyond the last point: clamp and warn, don't extrapolate. Don't assume force increases with draw; a compound bow's curve peaks and then drops (let-off). Add a test with a peaked curve.
- `launch_speed_mps(draw_m, bow) -> float`: `v = sqrt(2 * efficiency * E / arrow_mass)`.

Sanity numbers for `recurve_30lb` at 28 in: about **34.7 J** stored, about **52.7 m/s** launch.

### Step 3. Flight model (`physics/flight.py`)

State is `[x, y, z, vx, vy, vz]`. Forces on the arrow:

- gravity: `(0, 0, -g)`
- drag: `a_drag = -(0.5 * rho * Cd * A / m) * |v_rel| * v_rel`, where `v_rel = v - wind` and `A = pi * (d/2)^2`

Functions:

- `launch_velocity(speed_mps, pitch_deg, yaw_deg) -> np.ndarray` using the frame in section 3.
- `simulate(v0, params, x_target_m, dt_s) -> Trajectory`: fixed-step RK4 (write it by hand, no scipy). Stop when x crosses `x_target_m` and **linearly interpolate** the exact crossing point. Also stop if the arrow falls more than `launch_height_m` below launch (hit the ground) or x stops increasing.

`Trajectory` dataclass holds: arrays of `t, x, y, z, vx, vy, vz`, plus `hit_target_plane: bool`, `impact_y_m`, `impact_z_m`, `time_of_flight_s`, `impact_speed_mps`.

Provide a `drag=False` switch (or just Cd = 0) so tests can compare against vacuum physics.

### Step 4. Solver (`solver/aim.py`)

A target is a point in the world frame: `Target(range_m, lateral_m, height_m)`, where `height_m` is relative to launch height (target center 0.2 m below the nock means `height_m = -0.2`).

**Required aim**

`solve_aim(target, v_launch_mps, params) -> AimSolution`

1. Start yaw at `atan2(lateral, range)`.
2. Bisect on pitch inside `pitch_bracket_deg` until the vertical miss at the target plane is under `tolerance_m`. Use the low arc only.
3. Correct yaw: simulate, measure lateral miss, adjust yaw by `atan(miss / range)`, repeat pitch solve. Two or three outer passes is enough (yaw only matters with wind).
4. If the target is out of reach (even the top of the bracket falls short), return `converged=False` with a reason. Never return garbage.

`AimSolution`: `pitch_deg, yaw_deg, time_of_flight_s, impact_speed_mps, converged, iterations, reason`.

Use the previous solution as a warm start when one is passed in (this will matter at 50 Hz later).

**Aim error**

`aim_error(current_pitch_deg, current_yaw_deg, draw_m, target, params, warm_start=None) -> AimError`

- Compute launch speed from `draw_m`.
- Solve the required aim.
- Simulate the **current** aim to get the predicted impact point on the target plane.

`AimError`: `pitch_err_deg, yaw_err_deg, predicted_impact_y_m, predicted_impact_z_m` (relative to target center), `predicted_miss_m`, `solution: AimSolution`.

Sanity numbers for `recurve_30lb`, 28 in draw, target at 15 m, level with the nock, no wind:

| Quantity | Expected |
|---|---|
| Required pitch (with drag) | ~1.56° |
| Required pitch (no drag, analytic) | ~1.52° |
| Time of flight | ~0.29 s |
| Drop if shot level (0°) | ~0.41 m |
| Impact speed | ~50.7 m/s |

At 30 m: ~3.21° with drag vs ~3.04° without. Drag matters more as range grows.

### Step 5. Tests (definition of done for Phase 1)

All must pass with `pytest`:

**Bow**
- Linear force-draw curve: energy equals `0.5 * F_max * (draw - brace)` within 0.1%.
- Energy is zero at brace height and increases monotonically with draw.

**Flight**
- With drag off, impact height matches the vacuum formula `z = x*tan(θ) - g*x² / (2 v² cos²θ)` within 1 mm, for several angles and ranges.
- Halving `dt_s` changes the impact point by less than 1 mm (step size is small enough).
- With drag on, the arrow lands lower than with drag off.
- Cross-check: one case against `scipy.integrate.solve_ivp` agrees within 1 mm.
- Wind toward +y pushes impact toward +y.

**Solver**
- With drag off, required pitch matches the analytic low-arc solution within 0.01°:
  `tan θ = (v² - sqrt(v⁴ - g(g x² + 2 h v²))) / (g x)`, where h is target height relative to launch.
- Converges for ranges 3 m to 40 m on `recurve_30lb`.
- Required pitch increases monotonically with range.
- An unreachable target returns `converged=False`, not a number.
- Target to the left with no wind: required yaw is positive.
- `aim_error` at exactly the solved aim gives `predicted_miss_m < 0.001`.
- Warm start reaches the same answer in fewer iterations.

---

## 6. Phase 2: explorations

Each one is a standalone script in `explore/` that uses the engine, reads `config.yaml`, and either shows a plot or prints a table. Accept `--preset` as a CLI flag. Save plots to `explore/out/` as PNGs as well as showing them.

1. **`trajectories.py`**: side view (x vs z) of arcs for several pitches, drag vs no drag overlaid, with the target drawn at 15 m. Builds intuition for how flat the arc is.

2. **`sight_tape.py`**: required pitch vs range, 3 m to 40 m, with and without drag. Archers call this a sight tape. Print a table at 5 m intervals too.

3. **`sensitivity.py`**: the most useful one. For ranges 5, 10, 15, 20, 30 m, print how far the impact point moves (cm) for:
   - +1° pitch error
   - +1° yaw error
   - 1 cm short draw
   - 0.5 m error in the range estimate
   - 2 m/s crosswind

   This tells us how accurate each sensor needs to be. Example question it answers: "If yaw is off by 0.5°, do we still hit an 80 cm face at 15 m?"

4. **`monte_carlo.py`**: simulate 2000 shots where the archer follows the cue perfectly but sensors have noise. Sample errors from normal distributions with sigmas set at the top of the script (start with pitch 0.3°, yaw 0.3°, draw 5 mm, range 0.3 m). Plot impact points on a drawn target face (80 cm, 10 rings) and print the mean radius and the percent that land inside each ring. Lets us trade off sensor accuracy against grouping size.

5. **`explorer.py`**: interactive matplotlib window with sliders for draw (in), pitch (°), yaw (°), target range (m), crosswind (m/s), and Cd. Show side view, top view, and the impact point on the target face, all updating live. Show the current aim error as text. This is the one the team will play with the most, so make it responsive.

6. **`solver_timing.py`**: time `solve_aim` cold and warm-started over 200 random targets. Print mean and worst case in ms. We eventually want it to fit in a 20 ms loop budget on a Pi, so this just measures where we are. Don't optimize yet.

When Phase 2 is done, write a short `explore/FINDINGS.md` with the key numbers from each script (sensitivity table, Monte Carlo grouping, timing) and anything surprising.

---

## 7. Phase 3: next steps (do NOT build unless asked)

Listed so the code is structured to allow them later:

- `packets.py`: dataclasses for the sensor packet (`t_ms, accel_mps2[3], gyro_dps[3], draw_mm`), vision result, aim error, shot record
- `io/sources.py`: `FakeStream` producing synthetic packets (a bow slowly wandering around the target with hand tremor), later `ReplayFile` and `SerialPort` with the same interface
- `estimation/attitude.py`: complementary filter, pitch/roll from accel + gyro, yaw from gyro corrected by camera offset. Gyro bias calibration at startup. No magnetometer in the aim math.
- `feedback/cues.py`: deadband (about 0.3°), map errors to cue states (`adjust` / `hold`)
- `main.py`: 50 Hz loop wiring FakeStream -> estimation -> solver -> cues, printing to the terminal
- `io/shot_log.py`: CSV writer, one row per shot with state at release plus measured hit
- Calibration script: fit `efficiency` and `drag_coefficient` to logged shots
- Model bow roll (cant)

---

## 8. Open questions for the team (don't block on these)

- **Build or buy the bow, and recurve or compound?** The full team decides. Software only needs the measured force-draw curve either way.
- **Real arrows or suction cups?** The two presets cover both; the explorations will show how different they are.
- **The real force-draw curve.** Measure with a luggage scale at 4 in steps and drop the numbers into `config.yaml`.
- **Real arrow mass and diameter.** Weigh one on a kitchen scale.
- **Target size and distance** for the fall demo.
