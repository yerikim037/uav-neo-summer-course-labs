# Week 2 · Module 7 — Optical Flow

Estimate how fast the drone is moving by watching the ground slide past the
downward camera. This is how many real drones hold position indoors without GPS.

## What you'll learn

- What optical flow is, and sparse feature tracking (`cv2.goodFeaturesToTrack` + `cv2.calcOpticalFlowPyrLK`)
- Why downward-camera flow encodes ground speed
- Converting pixel motion to a metric velocity using altitude

## Key terms

- **Optical flow** — the apparent motion of image patterns between two frames, as a `(dx, dy)` displacement.
- **Sparse flow** — track a small set of distinctive corner points frame-to-frame (Lucas-Kanade). Far cheaper than **dense** flow (a vector for every pixel), so it keeps the simulator real-time.
- **Feature / corner** — a locally distinctive pixel patch that is easy to find again next frame; `goodFeaturesToTrack` picks them.
- **Flow magnitude** — the length `sqrt(dx² + dy²)` of a point's displacement; larger means faster apparent motion.
- **Ground footprint / meters-per-pixel** — how much real ground one pixel covers. It grows with altitude: `2 · height · tan(½ FOV) / image_width`.
- **Frame rate (dt)** — seconds between frames. Pixels/frame ÷ dt gives pixels/second, which the footprint converts to meters/second.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module7_optical_flow/main.py            # all steps, your code
drone sim course/week2_vision/module7_optical_flow/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_flow_magnitude.py`** — compute the average optical-flow magnitude while drifting
2. **`step2_velocity_estimate.py`** — convert flow to a velocity estimate and compare to the true velocity

## What to expect

The drone drifts gently forward so the ground moves under it. Step 1 prints a mean
flow magnitude; Step 2 prints an estimated velocity beside the drone's true velocity.

> **Run this lab in the FlightDemo world.** Optical flow tracks moving *features*, so a
> plain, untextured floor gives near-zero flow even while the drone moves. The default race
> world has a near-flat floor; select the **FlightDemo** world from the sim's startup menu —
> its textured floor produces clear flow. Flow ≈ 0 over bare ground is the scene, not your code.

## You're done when

- Step 1 prints a mean flow magnitude greater than 0 while drifting.
- Step 2 prints an estimated `(x, z)` velocity whose **sign and rough size** track the true velocity from `get_linear_velocity()`. The scale is approximate — exact match is not expected.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Flow is ~0 | Run in the **FlightDemo** world (textured floor); the race world is too plain. Also confirm the drone is drifting (`PROBE_PITCH`). |
| First frame errors | Sparse flow needs a previous frame *and* points — find features and store `_prev_gray`/`_prev_pts` before tracking. |
| Lab lags the sim | Don't run dense flow per pixel; sparse tracking (this lab) stays real-time. Lower `maxCorners` if needed. |
| Estimate has the wrong sign | The camera moves opposite to the scene flow; negate the mean flow. |
| Estimate is way too big/small | Check `meters_per_pixel` uses the current `height` and that you divided by `dt`. |

## Going further (optional)

- Compare against **dense** flow (`cv2.calcOpticalFlowFarneback`) and measure how much slower it runs — that's why this lab tracks sparse features.
- Use the flow velocity to **hold position**: command roll/pitch to drive the estimated velocity to zero.
- Optical flow drifts over time. Combine it with the Module 6 distance estimate to bound the error (a tiny taste of sensor fusion).

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
