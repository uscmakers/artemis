# Phase 2 findings (engine v0)

All numbers come from the placeholder presets in `config.yaml`. The bow and arrows
haven't been measured yet, so treat these as orders of magnitude, not specs.
Target face: 80 cm, 10 rings (4 cm per ring). "Gold" means the 9 and 10 rings (8 cm radius).
Plots are in `explore/out/` after running the scripts.

## 1. Trajectories (`trajectories.py`)

- **recurve_30lb** (52.7 m/s): the arc is very flat. A level shot drops 4.5 cm at 5 m,
  18 cm at 10 m, 41 cm at 15 m, and 73 cm at 20 m. The solved 15 m shot (1.56°) rises
  only 11 cm at its peak. Drag barely changes the shape inside 40 m.
- **toy_suction** (14.3 m/s): a real lob. Drag matters a lot. With drag, the furthest it
  can reach on level ground is **~17.5 m** (at ~42°), versus ~20.7 m in vacuum.

## 2. Sight tape (`sight_tape.py`)

recurve_30lb, target level with the nock:

| range | pitch (drag) | pitch (vacuum) | drag adds | time of flight | impact speed |
|---|---|---|---|---|---|
| 5 m  | 0.51° | 0.51° | 0.01° | 0.096 s | 52.0 m/s |
| 10 m | 1.03° | 1.01° | 0.02° | 0.192 s | 51.3 m/s |
| 15 m | 1.56° | 1.52° | 0.04° | 0.290 s | 50.6 m/s |
| 20 m | 2.10° | 2.03° | 0.07° | 0.390 s | 50.0 m/s |
| 30 m | 3.21° | 3.04° | 0.17° | 0.593 s | 48.7 m/s |
| 40 m | 4.37° | 4.06° | 0.31° | 0.803 s | 47.4 m/s |

toy_suction: 7.3° at 5 m, 15.8° at 10 m, 27.6° at 15 m; drag adds 0.3°, 1.4°, and 4.5°.
Out of reach past ~17.5 m.

**Takeaway:** on the recurve, the whole 5–40 m sight tape spans only 4°. For the toy bow,
drag isn't a correction, it *is* the trajectory, so the drag coefficient has to be tuned
from real shots before the toy predictions mean anything.

## 3. Sensitivity (`sensitivity.py`)

How far the impact moves (cm) from each error on its own, recurve_30lb:

| range | +1° pitch | +1° yaw | 1 cm short draw | +0.5 m range error | 2 m/s crosswind |
|---|---|---|---|---|---|
| 5 m  | 8.7  | 8.7  | 0.2 | 0.4 | 0.1 |
| 10 m | 17.4 | 17.5 | 0.7 | 0.9 | 0.5 |
| 15 m | 26.2 | 26.2 | 1.6 | 1.4 | 1.1 |
| 20 m | 34.9 | 34.9 | 2.9 | 1.9 | 2.1 |
| 30 m | 52.2 | 52.4 | 6.7 | 3.0 | 4.7 |

Error budget: how large each error can be, on its own, before the arrow moves one ring (4 cm):

| range | pitch / yaw | draw | range estimate | crosswind |
|---|---|---|---|---|
| 15 m | 0.15° | 2.5 cm | 1.4 m | 7 m/s |
| 30 m | 0.08° | 6 mm  | 0.7 m | 1.7 m/s |

**Answering the spec's example:** if yaw is off by 0.5° at 15 m, the arrow moves 13 cm. It
stays on the 80 cm face easily (the limit is 1.5°) but leaves the gold.

**Takeaways for the electrical team (recurve):**
- **Attitude (pitch/yaw) accuracy is the only hard requirement.** At 15 m every 0.1° is
  2.6 cm. The ~0.3° figure from the design notes keeps the arrow on the face but not in the gold.
- **Draw sensing is easy.** Millimeter-level isn't needed; ±5 mm is plenty out to 30 m.
- **Range sensing is easy.** ±0.5 m costs about 1.4 cm at 15 m, so a cheap ToF sensor or the
  ring-size estimate is fine.
- **Wind** is minor inside 20 m with real arrows.
- **The toy bow is the opposite.** Range and draw errors matter as much as attitude
  (+0.5 m of range error moves the impact 15 cm at 10 m and 31 cm at 15 m).

## 4. Monte Carlo (`monte_carlo.py`)

2000 shots; sensor sigmas: pitch 0.3°, yaw 0.3°, draw 5 mm, range 0.3 m. The archer
follows the cue perfectly.

| case | mean radius | in 10 | in gold | inside 6-ring (20 cm) | on face |
|---|---|---|---|---|---|
| recurve, 15 m | 9.9 cm  | 12% | 40% | 96% | 100% |
| recurve, 30 m | 19.9 cm | 4%  | 13% | 54% | 96% |
| toy, 10 m     | 11.1 cm | 13% | 39% | 89% | 99.8% |

Each noise source on its own (recurve, 15 m): pitch alone gives a 6.2 cm mean radius,
yaw alone 6.1 cm, draw alone 0.3 cm, range alone 0.7 cm. **Essentially all of the group
size comes from the attitude sensors.** For the toy bow at 10 m, range noise is the biggest
single source (7.4 cm), ahead of pitch, yaw, and draw (3–4 cm each).

From the 2D normal distribution, at 15 m you'd need a per-axis attitude sigma of about
0.2° for 70% gold, or about 0.14° for 90% gold. At 30 m, halve those.

## 5. Explorer (`explorer.py`)

Each slider update takes about 9 ms mean and 22 ms worst on the recurve, or about 35 ms
mean on the toy bow (its flights are longer). Redraw comes on top of that. It feels live.

## 6. Solver timing (`solver_timing.py`)

200 random targets (3–40 m, ±2 m lateral, ±1 m height) on an Apple M4 Pro laptop,
pure Python:

| | mean | p95 | worst | sims per solve |
|---|---|---|---|---|
| cold                              | 12.3 ms | 23.4 ms | 25.8 ms | 15.5 |
| warm start (target moved slightly) | 7.6 ms  | 14.9 ms | 17.0 ms | 9.2 |
| full `aim_error`, warm            | 8.4 ms  | 16.3 ms | 18.4 ms | 10.2 |
| cold, 3 m/s crosswind             | 15.1 ms | 28.8 ms | 43.1 ms | 19.1 |

On the laptop, a warm `aim_error` fits the 20 ms budget. **A Raspberry Pi running pure
Python is typically several times slower** (not measured yet), so expect this to be
**over budget on the Pi** as it stands. Cheap fixes, in order of payoff, for when we get there:
1. Replace bisection with a secant step from the warm start. With a good guess it should
   converge in 2–3 simulations instead of ~9.
2. Loosen `dt_s` to 2–5 ms. RK4 error is tiny here (halving dt moves the impact by far less than 1 mm).
3. Loosen `tolerance_m`. 0.5 mm is ~100× finer than the sensors can support.
4. Run the solver less often than 50 Hz, e.g. only when range or draw changes. Required aim
   depends only on range, draw, and wind, not on where the bow is pointing.

## Surprises / things to act on

1. **Measure the force-draw curve past full draw.** Both presets have nominal draw equal to
   the last table point, so any draw reading above it gets clamped (half of all noisy
   readings). The solver then thinks the arrow is slower than it is and aims high:
   +3.5 cm bias for the toy bow at 10 m, +1.2 cm for the recurve at 30 m. Take luggage-scale
   readings 1–2 in past the expected full draw.
2. **Warm start initially did nothing.** It reused the previous yaw, which is wrong as soon
   as the target slides sideways, forcing an extra yaw pass. Fixed: it now carries over only
   the wind-drift correction (`AimSolution.yaw_drift_deg`) and recomputes the bearing.
   Warm solves dropped from 12.8 to 9.2 simulations (recurve) and from 14.2 to 8.2 (toy).
3. **The 0.3° cue deadband matters as much as the sensor noise.** If the archer releases
   anywhere inside ±0.3°, that is about ±8 cm at 15 m, the same size as the whole Monte Carlo
   group. Consider a tighter deadband for the "hold" tone, or a graded tone near center.
4. **Real arrows vs suction cups is a big decision for the demo.** The toy bow can't
   reach 20 m at all, needs 15–28° of pitch at 10–15 m, and is dominated by drag, which is the
   least-known parameter.
5. **Near max range the toy solver needs a scan.** With drag, 45° (the top of the bracket)
   can fall short even when a lower angle reaches. The solver handles this, but unreachable
   targets cost ~25 simulations to reject.
