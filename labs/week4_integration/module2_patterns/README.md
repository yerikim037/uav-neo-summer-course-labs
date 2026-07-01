# Week 4 · Module 2 — Flight Patterns

A flight path is a list of waypoints visited in order. This module flies a square by
reusing the Module 1 waypoint controller, one corner at a time. It tests whether your
position controller actually composes into a longer route.

## What you'll learn

- Sequencing waypoints into a path
- Advancing to the next target when the current one is reached
- Reusing a single-point controller across a whole route

## Key terms

- **Path / trajectory** — an ordered list of waypoints the drone visits in turn.
- **Leg** — one segment of the path, from one waypoint to the next.
- **Waypoint advance** — the rule for deciding you've "arrived" at a corner and should switch to the next (here: within `WP_TOL` meters on both axes).
- **Composability** — whether a controller that works for one target still works when you chain many. Drift and overshoot that were harmless on one leg can accumulate over four.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week4_integration/module2_patterns/main.py            # all steps, your code
drone sim course/week4_integration/module2_patterns/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_fly_square.py`** — visit the four corners of a square, then land

## What to expect

The drone flies corner to corner, printing each corner as it reaches it, and lands
after the last one.

## You're done when

The drone visits all four corners in order (you'll see a "reached corner" message for
each), returns near the start, and lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Skips a corner | `WP_TOL` may be too large, or you advance `_wp` before actually arriving. |
| Stalls between corners | `WP_TOL` too tight for the dead-reckoning drift — loosen it. |
| Drifts farther off each leg | Position error accumulates; this is expected. Note how far off the final corner is from the start. |
| Loses height while turning corners | Keep the throttle altitude term running every frame, not just on straight legs. |

## Going further (optional)

- Replace the square with a **figure-8** or a circle by generating waypoints from a parametric equation (`x = R·sin(t)`, `z = R·sin(2t)`).
- Yaw to face the next corner before flying to it, so the drone "drives" the path instead of strafing.
- Measure total drift: how far from `(0, 0)` does the drone end up after a full loop? What would reduce it?

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
