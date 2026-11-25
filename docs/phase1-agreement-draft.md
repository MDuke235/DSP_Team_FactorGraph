# Phase-1 agreement — Project 5: Factor-Graph INS/GPS Fusion (DRAFT)

Teams: Team-XX, Team-YY, Team-ZZ · Status: **proposal for discussion** · Approved by instructor: ☐

The course requires the three teams on the same project to agree on three things before Phase 2
(Introduction slides, p. 10): the **baseline data model**, the **interface**, and the **evaluator**.
This draft is a starting point; every item marked ❓ needs a decision in the joint meeting.

## 1. Baseline data model

| Item | Proposal | Decision |
|---|---|---|
| Motion | Planar (2D) figure-eight, A = 200 m, B = 100 m, period 120 s | ❓ 2D vs 3D for baseline |
| Duration | 120 s | ❓ |
| IMU | 100 Hz; 2-axis accel + yaw gyro; white noise σ_a = 0.02 m/s², σ_g = 0.002 rad/s | ❓ |
| IMU bias | Constant; b_a ~ N(0, 0.05²), b_g ~ N(0, 0.005²) | ❓ |
| GPS | 1 Hz, position only, σ = 2 m, Gaussian | ❓ |
| Outage | One window [50, 70) s | ❓ |
| Not in baseline | Outliers, lever arm, time offset, bias random walk, 3D | (Phase 2 advanced) |

Full description: [data-model.md](data-model.md). Reference generator: `generate_baseline.m`.

## 2. Interface

- One `.mat` file with `meta`, `imu`, `gps`, optional `gt`; SI units; local ENU frame.
- Algorithm entry point `est = run_algorithm(data)`; output at every GPS epoch: `est.t`, `est.pos` (required), `est.yaw`, `est.vel`, `est.bias`, `est.pos_std` (optional).
- Challenge data given to other teams may omit `gt` only if the owner team scores it.

❓ Decide: field names, whether `gps.pos` uses `NaN` during outages, MATLAB version.

Full specification: [interface.md](interface.md).

## 3. Evaluator

- Primary metric: horizontal position RMSE over all output epochs (m), lower is better.
- Secondary: RMSE during outages, max error during outages, final error, heading RMSE, bias error.
- Validity conditions (speed ≤ 30 m/s, MEMS noise ranges, GPS σ 0.3–10 m, outage ≤ 60 s, …) —
  data violating them is rejected. This answers "what if another team generates unrealistic data?"
- Execution: `run_challenge(file)` → validate → run → score; runtime ≤ 120 s.

❓ Decide: primary metric, tie-break rules, how many challenge datasets per team, Monte-Carlo seeds.

## 4. Organisation

| Item | Proposal | Decision |
|---|---|---|
| Shared folder | Google Drive `Projects/Project-5/Team-xx/` | ❓ confirm path with instructor |
| Challenge data deadline | 48 h before defence (course rule) | fixed |
| Communication | Zalo group of the three teams | ❓ |
| Next joint meeting |  | ❓ |
