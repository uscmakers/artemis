# Mapping a real bow into the simulator

A hands-on guide for the day we have the actual bow and arrows. Goal: replace the placeholder
numbers in `config.yaml` with measured ones, so the simulator predicts where *our* arrows land.

Every test below has a step-by-step procedure, the tools it needs, and a data sheet. Most of it is
an afternoon with a luggage scale, a kitchen scale, and a borrowed chronograph.

---

## Short answer: what does the simulator need to know?

**Do we need the string's spring constant?** No, and it wouldn't be enough anyway. The string barely
stretches; the *limbs* are the spring. And a bow isn't a simple spring:

- A recurve is *roughly* linear. Our placeholder works out to about 1.46 lb per inch. But the force
  usually climbs faster near full draw ("stacking"), so a single number gets the stored energy wrong.
- A compound is nowhere near linear: force rises to a peak, then drops by 65–85% near full draw (let-off).

So instead of a spring constant, we measure the **whole force-draw curve**: a table of force at each
draw length. That table *contains* the spring constant (it's the slope) and everything a single number
would miss. The simulator integrates the area under it to get stored energy.

Everything the simulator needs, per bow + arrow setup:

| Parameter in `config.yaml` | What it is | How we get it | Tool | Test |
|---|---|---|---|---|
| `force_draw` (table) | String force at each draw length | Pull the string back in steps and read the force | Digital luggage scale + ruler | [Test 1](#test-1-force-draw-curve) |
| `arrow_mass_kg` | Mass of one complete arrow | Weigh it | 0.1 g scale | [Test 2](#test-2-arrows) |
| `arrow_diameter_m` | Shaft (or suction cup) diameter | Measure it | Calipers | [Test 2](#test-2-arrows) |
| `efficiency` | Fraction of stored energy that becomes arrow speed | Measure arrow speed, compare to stored energy | Chronograph (or a phone mic) | [Test 3](#test-3-arrow-speed--efficiency) |
| `drag_coefficient` | How quickly the arrow slows down in flight | Measure speed near the bow and near the target | Chronograph | [Test 4](#test-4-drag-coefficient) |
| `nominal_draw_in` | The archer's normal full draw | Measure at full draw | Ruler or the draw sensor | [Test 1](#test-1-force-draw-curve) |
| `launch_height_m` | Nock height above the ground at full draw | Tape measure | Tape | 1 minute |
| *(not in config yet)* pitch/yaw offsets | Angle between where the sensors say the bow points and where the arrow actually goes | Shoot at short range, measure the group's offset | Sensors on the bow | [Test 5](#test-5-sensor-alignment-after-sensors-are-mounted) |

---

## How precisely do we need each number?

Computed with the engine on the `recurve_30lb` placeholder. The solver aims using the config
numbers, the "real" arrow flies with slightly different ones, and the table shows how far
**low** the arrow lands as a result:

| If this is wrong by… | miss at 10 m | 15 m | 20 m | 30 m |
|---|---|---|---|---|
| Efficiency 5% too high (0.80 vs 0.76) | 0.9 cm | 2.1 cm | 3.9 cm | 8.9 cm |
| Whole force curve 5% too high | 0.9 cm | 2.1 cm | 3.9 cm | 8.9 cm |
| Arrow mass 1 g too light (20 g vs 21 g) | 0.8 cm | 2.0 cm | 3.6 cm | 8.0 cm |
| **Draw reference 0.5 in off** (see below) | 0.9 cm | 2.1 cm | 3.7 cm | 8.5 cm |
| Drag coefficient 25% too low (2.0 vs 2.5) | 0.0 cm | 0.3 cm | 0.7 cm | 2.3 cm |
| Drag coefficient 2× too low (2.0 vs 4.0) | 0.3 cm | 1.1 cm | 2.7 cm | 9.6 cm |

One ring on an 80 cm face is 4 cm. What this means:

- **Speed-related numbers (force curve, efficiency, arrow mass, draw reference) matter most.**
  Aim for ~2% accuracy on each, and they stay under a ring out to 20 m.
- **Drag barely matters for real arrows inside 20 m.** Even being off by 2× costs less than a ring.
  Don't spend a day on it; a rough value now and a fit from logged shots later is fine.
- **For the suction-cup toy bow it's the opposite.** At 10 m, Cd 25% off costs 6 cm and efficiency
  5% off costs 15 cm. Drag *is* the trajectory, so Test 4 matters a lot.

Even with a perfect calibration, these errors are small compared with the aim-sensor noise
(1° of pitch = 26 cm at 15 m; see [explore/FINDINGS.md](../explore/FINDINGS.md)).
Calibration removes the *consistent* offset; sensor noise sets the *group size*.

---

## Before you start

### Decide the draw reference point first (important)

"Draw length" can be measured from different places, and **the force-draw table and the draw sensor
must use the same one.** The table above shows a 0.5 in mismatch costs as much as a 5% efficiency error.

- The config's convention is **grip pivot point → string** (the deepest point of the grip, where
  your hand pushes).
- Bow shops and spec sheets quote **AMO draw length = pivot → string + 1.75 in**. A "28 in AMO" bow
  is 26.25 in in our convention. Don't mix them.
- The draw sensor will measure **riser → string** from wherever it's mounted, which is yet another offset.

**Best practice:** once the draw sensor is mounted, **record its reading at every step of Test 1**,
alongside the tape-measure value. Then we can build the table directly in the sensor's own
units and the offset question disappears. Until then, measure from the pivot point and write it down.

### Shopping / borrowing list

| Item | Approx. cost | Used for |
|---|---|---|
| Digital luggage scale, 0.1 lb resolution, rated ≥ 1.5× the draw weight | $10–15 | Test 1 |
| Yardstick or tape measure; masking tape and marker | — | Test 1 |
| Digital scale with 0.01–0.1 g resolution ("jewelry" or "reloading" scale) | $15 | Test 2 |
| Digital calipers | $15 | Test 2 |
| **Chronograph** (e.g. Caldwell, Competition Electronics) | $100–150, or **borrow one from an archery shop or club**, since most have one | Tests 3–4 |
| Phone with slow-motion video and a voice recorder | — | Backups for Test 3 |
| Target butt / backstop rated for the bow's draw weight | — | Tests 3–5 |
| Bow vise or a padded clamp, or a second person | — | Test 1 |

### Safety

- **Never dry-fire** (release the string without an arrow). It can break the limbs and hurt someone.
  When measuring the force-draw curve, **let the string down slowly** by hand. Never let go.
- During Test 1, keep the bow pointed at a backstop with nothing in front of it.
- Don't pull a recurve far past its rated maximum draw length.
- Chronograph: shoot through the middle of the sensor window from a few feet back, at a
  backstop. Arrows that hit the chronograph break it.

### Let a new string settle

New strings stretch over the first ~50–100 shots, which changes brace height and the force curve.
Shoot the bow in before calibrating, and **re-run Test 1 whenever the string, limbs, or brace
height change.**

---

## Test 1: Force-draw curve

**Gives:** `force_draw` table, `nominal_draw_in`, brace height (first point of the table).
**Time:** 30–45 min. **People:** 2 (one holds the bow, one pulls and reads).

### Setup

1. Mark the draw positions. Tape a yardstick along the arrow line, or tape a ruler to an old arrow
   shaft, with **0 at the grip pivot point**. Mark every **1 in** for a compound, every **2 in**
   for a recurve.
2. Clamp the bow by the grip (bow vise or padded clamp), or have a person hold it at arm's length,
   **vertical and steady**.
3. Measure the **brace height**: pivot point to the string at rest. That's the first table row,
   with force 0.

### Procedure

1. Hook the luggage scale on the string **at the nocking point** (where the arrow would sit).
2. Pull **straight back along the arrow line**, not up or down, until the string reaches the next mark.
3. Hold still for a second and record the force.
   *If the draw sensor is mounted, record its reading too.*
4. Continue to full draw:
   - **Recurve:** continue **1–2 in past** the archer's normal full draw, staying within the bow's
     rated limit. This matters: if full draw is the *last* point in the table, every noisy sensor
     reading above it gets clamped, which biases the aim high (see FINDINGS, "Surprises" #1).
   - **Compound:** you'll hit the "wall" and can't go further. That's the end of the table. Take
     extra points (every 0.5 in) around the peak and the let-off valley, where the curve bends sharply.
5. **Let down slowly.** Never release.
6. Repeat the whole sweep **3 times** and average each row. If one sweep is way off, redo it.
7. Have the archer come to their real full draw a few times and measure where the string ends up.
   Average it; that's `nominal_draw_in`.

> **Use the pull-back readings, not the let-down ones.** Force is a bit lower on the way down.
> That lost energy never reaches the arrow, and the `efficiency` number accounts for it.

### Data sheet

| Mark (in from pivot) | Draw-sensor reading | Force sweep 1 (lb) | Sweep 2 | Sweep 3 | **Average** |
|---|---|---|---|---|---|
| brace = ____ | | 0 | 0 | 0 | 0 |
| | | | | | |
| | | | | | |
| … | | | | | |

### Sanity checks

- Recurve: the force at full draw should match the bow's labeled draw weight within ~10%. Recurves are
  usually labeled at 28 in AMO, which is 26.25 in in our convention.
- The plot of force against draw should be smooth. A jump usually means a misread or the scale slipped.
- Check the energy (after adding the preset, see [Putting it into the config](#putting-it-into-the-config)):
  ```bash
  python3 -c "from artemis import load_config; from artemis.physics.bow import stored_energy_j; p=load_config(preset='our_bow'); print(stored_energy_j(p.bow.nominal_draw_m, p.bow.curve), 'J')"
  ```
  A 30 lb recurve stores roughly 30–40 J; a 50 lb compound noticeably more (often 60–90 J), because its curve stays high for longer.

---

## Test 2: Arrows

**Gives:** `arrow_mass_kg`, `arrow_diameter_m`. **Time:** 10 min.

1. Weigh **every arrow complete**: point, nock, fletching, and suction cup if it has one. Use 0.1 g
   resolution or better.
2. Use the **average** for `arrow_mass_kg`. Arrows more than ~1 g from the average will hit
   differently; set them aside or label them. (Arrow shops quote mass in grains: 1 grain = 0.0648 g,
   so 310 grains = 20.1 g.)
3. Measure the shaft diameter with calipers in 3 places and average. For suction-cup arrows, use the
   **cup's** diameter, since the cup causes most of the drag.

> If you change arrows later, **re-run Test 3**. Heavier arrows absorb more of the bow's energy,
> so efficiency depends on the arrow, not just the bow.

| Arrow # | Mass (g) | Diameter (mm) | Notes |
|---|---|---|---|
| 1 | | | |
| … | | | |
| **Average** | | | |

---

## Test 3: Arrow speed → efficiency

**Gives:** `efficiency`. **Time:** 30–60 min.

The bow model says `speed = sqrt(2 · efficiency · stored_energy / arrow_mass)`. We know the stored
energy (Test 1) and the mass (Test 2), so measuring the speed gives us the efficiency.

### Procedure (chronograph)

1. Set the chronograph **1.5–3 m in front of the archer**, at arrow height, in front of a backstop.
2. Shoot **5 arrows at full draw**. Record each speed. Most chronographs read ft/s; multiply by
   0.3048 for m/s.
3. Repeat at **two or three shorter draws**, e.g. 2 and 4 inches short of full. Holding a partial
   draw is hard; with the draw sensor mounted, just record its reading for each shot.
4. Average each draw length's speeds.

### Compute

Add the new preset to `config.yaml` (see [Putting it into the config](#putting-it-into-the-config)),
then:

```python
from artemis import load_config
from artemis.config import M_PER_IN
from artemis.physics.bow import stored_energy_j

params = load_config(preset="our_bow")
arrow_mass_kg = params.bow.arrow_mass_kg

# (draw_in, average speed in m/s). From ft/s: multiply by 0.3048.
readings = [(28.0, 52.1), (26.0, 47.9), (24.0, 43.2)]
for draw_in, v_mps in readings:
    energy_j = stored_energy_j(draw_in * M_PER_IN, params.bow.curve)
    efficiency = arrow_mass_kg * v_mps**2 / (2 * energy_j)
    print(f"{draw_in:4.1f} in: {energy_j:5.1f} J stored -> efficiency {efficiency:.3f}")
```

Put the full-draw efficiency in the config.

**Read the result:**
- Typical: recurve 0.70–0.85, compound similar or a bit higher, toy bows much lower.
- **If efficiency changes by more than ~5% across draw lengths**, the single-number model isn't
  enough. Tell software, and we'll switch to an efficiency-vs-draw table (a small code change). For
  the made-up numbers above, it goes 0.78 → 0.83, which would be borderline.
- Way outside 0.5–0.95: re-check the draw reference point and the units (inches vs cm, lb vs kg).

### No chronograph? Use sound

A phone can time the flight. Put the phone **next to the archer**, record, and shoot at a target a
measured distance away (15–20 m). Open the recording in Audacity (free) and measure the time from
the release "twang" to the impact "thwack". Then:

```
time_of_flight = measured_gap − (distance / 343)        # 343 m/s: sound travels back from the target
```

Average 5+ shots. The engine can then find the launch speed that gives that flight time. Ask
software, or use `solve_aim` and vary the speed until `time_of_flight_s` matches. This is good to
~1–2%. Slow-motion video works too, but at 240 fps a 50 m/s arrow moves 21 cm per frame, so you
need a long, measured stretch (3+ m) in view.

---

## Test 4: Drag coefficient

**Gives:** `drag_coefficient`. **Time:** 30 min.

For real arrows, a rough value is fine (see the precision table). For suction-cup arrows, do this carefully.

### Real arrows: speed loss over distance

Drag shows up as the arrow slowing down. Over 15 m a target arrow loses roughly 4% of its speed.

1. Shoot 5 arrows through the chronograph **near the bow** (~2 m). Average the speeds.
2. Move the chronograph **near the target** (~15 m), with the target right behind it, and shoot 5
   more. Shoot level at chronograph height. This is the risky part, so go slowly and use spare
   arrows.
3. Fit Cd with the engine:

```python
import numpy as np
from artemis import load_config
from artemis.physics.flight import launch_velocity, simulate

params = load_config(preset="our_bow")
v_near_mps, v_far_mps = 52.1, 50.3     # averaged chronograph readings
gap_m = 13.0                           # distance between the two chronograph positions

def speed_after_gap(cd):
    flight = params.with_flight(drag_coefficient=cd).flight
    return simulate(launch_velocity(v_near_mps, 0.0, 0.0), flight, gap_m, params.solver.dt_s).impact_speed_mps

cds = np.arange(0.5, 6.0, 0.05)
best = cds[np.argmin([abs(speed_after_gap(cd) - v_far_mps) for cd in cds])]
print(f"best-fit drag_coefficient: {best:.2f}")
```

Note that `Cd` here is relative to the **shaft cross-section**. Most of an arrow's drag is skin
friction along its length plus the fletching, so the number comes out larger than you'd expect for a
sphere or a car. Values around 1.5–3 are plausible. Don't be alarmed.

### Suction-cup / slow arrows: range at a fixed angle

For slow arrows, drag changes where they land a lot, which makes it easy to measure:

1. On flat ground, set the bow to a known pitch (e.g. 15° and 30°) using a digital angle gauge or
   a phone inclinometer app held against the arrow.
2. Shoot 5 arrows at each angle and measure where each **first touches the ground**.
3. Fit Cd: try values in the explorer (`python3 -m explore.explorer --preset our_bow`, Cd slider) or
   ask software for a fitting script. With efficiency already known from Test 3, only Cd is left to adjust.

---

## Putting it into the config

Add a new preset to [config.yaml](../config.yaml). Keep the old ones for comparison. Example:

```yaml
active_preset: our_bow

presets:
  our_bow:
    # Measured 2026-__-__ by ____. Bow: ______, string: ______, brace ____ in.
    # draw_in = grip pivot point -> string (NOT AMO). Pull-back readings, 3-sweep average.
    force_draw:
      draw_in:  [7.75, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]
      force_lb: [0.0, 3.1, 6.0, 8.9, 11.8, 14.6, 17.5, 20.4, 23.4, 26.5, 29.8, 33.4]
    efficiency: 0.78              # Test 3, chronograph at full draw, 5 shots
    arrow_mass_kg: 0.0201         # Test 2, average of 12 arrows
    arrow_diameter_m: 0.0072      # Test 2, calipers
    drag_coefficient: 2.1         # Test 4, speed loss over 13 m
    nominal_draw_in: 28
    launch_height_m: 1.45         # nock height at full draw, archer standing
```

(The numbers are illustrative. Note the table goes 2 in past the 28 in full draw.)

**Write the date, who measured it, and the bow/string in the comment.** When results look off in
three months, this is the first thing we'll want.

Then check it:

```bash
python3 -m pytest                                   # nothing should break
python3 -m explore.sight_tape --preset our_bow      # does the sight tape look sensible?
python3 -m explore.sensitivity --preset our_bow     # updated error budget for the electrical team
```

---

## Test 5: Sensor alignment (after sensors are mounted)

> **Needs a small code addition** (Phase 3): `pitch_offset_deg` and `yaw_offset_deg` per preset.
> v0 assumes the arrow leaves exactly along the direction the sensors report.

The IMU and camera will never be perfectly lined up with the arrow, and the arrow doesn't leave
perfectly straight (arrow rest height, nocking point, arrow flex). Together that's a **fixed angle
offset** between "where the sensors say we're aiming" and "where the arrow actually goes."

### Procedure

1. Shoot a group of **5–10 arrows at 5 m** with the system cueing "hold" (or with a fixed, logged aim).
2. Measure the **center of the group** relative to where the simulator predicted (cm up/down, left/right).
3. Convert to angle: `offset_deg = atan(offset_m / range_m)`. At 5 m, 1 cm ≈ 0.11°.
4. Repeat at 10 m and 20 m.

### Which number is wrong? Use the distance pattern

How a consistent miss *changes with distance* tells you which number to fix:

| Group center vs prediction | Grows with distance like… | Most likely cause | Fix |
|---|---|---|---|
| High or low by the **same angle** at every distance (cm grows in proportion to range) | range | Pitch offset: sensor mounting, nocking point | `pitch_offset_deg` |
| Left or right by the **same angle** at every distance | range | Yaw offset: camera boresight, arrow rest, arrow flex | `yaw_offset_deg` |
| Low, and the miss grows **much faster** than range (small at 10 m, big at 30 m) | ~ range² or faster | Arrow slower than the model thinks | Re-check efficiency, force curve, draw reference, arrow mass |
| High, growing much faster than range | ~ range² | Arrow faster than modeled | Same as above |
| Fine up close, low only at long range | mostly beyond 20 m | Drag coefficient too low | Increase Cd |
| Big vertical spread, small horizontal spread | — | Inconsistent draw, or pitch sensor noise | Check the draw-sensor readings shot to shot |

Short-range shots (5–10 m) isolate the angle offsets, because speed and drag barely matter there.
Long-range shots (20–30 m) expose speed and drag errors. **Fix the offsets first, then speed and drag.**

---

## The long-term plan: let the shot log do it

Once the bow streams sensor data, **every shot gets logged**: draw, pitch, yaw, range, and where it
actually hit (scored by hand or from the camera). With a few dozen logged shots at mixed distances,
a calibration script (Phase 3 in `SPEC.md`) can fit `efficiency`, `drag_coefficient`, and the pitch/yaw
offsets all at once, by adjusting them until the simulator's predicted hits best match the real ones.

That turns calibration from a one-time afternoon into something that keeps improving the model
as we shoot. It's the spring "higher precision" goal. The tests above give it a good starting point,
so the fit starts close to the right answer.

---

## When to recalibrate

| Change | Re-run |
|---|---|
| New string, brace height changed, limbs swapped, draw weight adjusted | Test 1, then 3 |
| Different arrows (mass, spine, points, fletching) | Tests 2, 3, 4 |
| Sensors or camera remounted or bumped | Test 5 |
| Big temperature change (cold limbs are a bit stiffer) | Quick Test 3 check |
| Hits drift over a session | Check the string hasn't stretched; quick Test 5 |

---

## One-page checklist

- [ ] Decide and write down the draw reference point (pivot → string)
- [ ] Shoot in the string (~50–100 shots)
- [ ] **Test 1:** force-draw curve, 3 sweeps, extending past full draw (recurve), plus brace height and nominal draw
- [ ] **Test 2:** weigh and measure every arrow; set aside outliers
- [ ] **Test 3:** chronograph at full draw and 2 shorter draws → efficiency
- [ ] **Test 4:** chronograph near bow + near target → Cd (careful version for suction cups)
- [ ] Measure nock height at full draw → `launch_height_m`
- [ ] Add the `our_bow` preset with date and notes; run pytest, sight_tape, sensitivity
- [ ] *(after sensors)* **Test 5:** groups at 5, 10, 20 m → pitch/yaw offsets
- [ ] *(Phase 3)* turn on shot logging and let the fit refine everything
