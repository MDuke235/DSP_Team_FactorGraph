# Interface specification

> Status: **DRAFT v0.1.0**. To be agreed by the three Project-5 teams and approved by the instructor (see [phase1-agreement-draft.md](phase1-agreement-draft.md)).

## 1. Problem definition

Estimate the planar trajectory (position, heading, velocity) of a small UAV from
a 100 Hz IMU (2-axis accelerometer + yaw gyro) and a 1 Hz GPS receiver.
GPS may be unavailable during outage windows. The estimate is requested at every
GPS epoch, including epochs inside outages.

Scope of v0.1.0 (baseline): planar motion, synchronized clocks, no lever arm,
constant IMU biases, Gaussian GPS noise without outliers.

## 2. Input data

One MATLAB `.mat` file containing the variables `meta`, `imu`, `gps` and (optionally) `gt`.
Load with `data = load(file)`. Frame: local ENU (x = East, y = North); body frame x forward, y left.

| Field | Type/shape | Unit | Required | Description |
|---|---|---|---|---|
| `imu.t` | N×1 double, uniform | s | Yes | IMU timestamps, step `1/meta.imu_fs` |
| `imu.acc` | N×2 double | m/s² | Yes | Body-frame specific force [forward, left] |
| `imu.gyro` | N×1 double | rad/s | Yes | Yaw rate (counter-clockwise positive) |
| `gps.t` | M×1 double | s | Yes | GPS epochs; each coincides with an IMU sample |
| `gps.pos` | M×2 double | m | Yes | [East, North] fix; `NaN` when not valid |
| `gps.valid` | M×1 logical | – | Yes | `false` during outages |
| `meta.interface_version` | char | – | Yes | `'0.1.0'` |
| `meta.imu_fs`, `meta.gps_fs` | double | Hz | Yes | Sampling rates |
| `meta.sigma_acc`, `meta.sigma_gyro` | double | m/s², rad/s | Yes | IMU white-noise std per sample |
| `meta.bias_acc_sigma`, `meta.bias_gyro_sigma` | double | m/s², rad/s | Yes | Std of the (constant) bias distribution |
| `meta.sigma_gps` | double | m | Yes | GPS noise std per axis |
| `meta.gps_outages` | R×2 double | s | Yes | Rows `[t_start t_end)` of outage windows |
| `meta.init_state` | 1×5 double | m, rad, m/s | Yes | Noisy initial state `[px py yaw vx vy]` |
| `meta.init_sigma` | 1×5 double | m, rad, m/s | Yes | Std of `init_state` |
| `meta.seed`, `meta.description`, `meta.frame`, `meta.units` | – | – | Recommended | Provenance |
| `gt.t` | N×1 | s | If available | Ground-truth timestamps (IMU rate) |
| `gt.pos`, `gt.vel` | N×2 | m, m/s | If available | True position / velocity |
| `gt.yaw` | N×1 | rad | If available | True heading (unwrapped) |
| `gt.bias` | 1×3 | m/s², rad/s | If available | True `[b_ax b_ay b_g]` |

The algorithm must **not** read `gt`. `run_algorithm` removes it before calling the solver.

## 3. Algorithm output

A struct `est`:

| Field | Type/shape | Unit | Required | Description |
|---|---|---|---|---|
| `est.t` | K×1 | s | Yes | Output epochs (= `gps.t`) |
| `est.pos` | K×2 | m | Yes | Estimated [East, North] |
| `est.yaw` | K×1 | rad | Recommended | Heading wrapped to [-π, π) |
| `est.vel` | K×2 | m/s | Optional | Velocity |
| `est.bias` | 1×3 | m/s², rad/s | Optional | Estimated biases |
| `est.pos_std` | K×2 | m | Optional | 1-σ marginal std of position |
| `est.method` | char | – | Recommended | Algorithm name |

## 4. Program entry point

```matlab
est = run_algorithm(data)                 % data: struct or path to .mat
est = run_algorithm(data, opts)           % e.g. opts.robust = 'huber'
M   = evaluate_estimate(data, est)        % requires data.gt
[ok, report] = validate_data(data)        % interface + validity check
```

Minimal example:

```matlab
setup_paths;
est = run_algorithm('data/baseline/baseline_seed42.mat');
data = load('data/baseline/baseline_seed42.mat');
M = evaluate_estimate(data, est);
```

## 5. Evaluator

| Metric | Formula | Unit | Primary |
|---|---|---|---|
| `rmse_pos` | sqrt(mean(‖p̂ₖ − pₖ‖²)) over all output epochs | m | **Yes** |
| `rmse_pos_gps` | same, epochs with GPS | m | |
| `rmse_pos_outage` | same, epochs inside outages | m | |
| `max_err_outage` | max ‖p̂ₖ − pₖ‖ inside outages | m | |
| `final_err` | ‖p̂_K − p_K‖ | m | |
| `rmse_yaw_deg` | RMSE of wrapped heading error | deg | |
| `bias_err` | \|b̂ − b\| per component | m/s², rad/s | |

Lower is better. Ground truth is linearly interpolated to `est.t`.
Execution: `run_challenge(file)` validates, runs, and scores a dataset; runtime limit 120 s.

## 6. Validity conditions (data from any team must satisfy these)

| Quantity | Allowed range | Reason |
|---|---|---|
| True speed | ≤ 30 m/s | Small multirotor / fixed-wing UAV |
| ‖acc‖ (horizontal, measured) | ≤ 2 g | Normal manoeuvres |
| \|gyro\| | ≤ 3 rad/s | Normal manoeuvres |
| `sigma_acc` | 0.001 – 0.5 m/s² | Consumer-to-tactical MEMS |
| `sigma_gyro` | 1e-4 – 0.05 rad/s | Consumer-to-tactical MEMS |
| `sigma_gps` | 0.3 – 10 m | RTK-float to degraded standalone GPS |
| Bias std | acc ≤ 0.5 m/s², gyro ≤ 0.05 rad/s | MEMS turn-on bias |
| Longest GPS gap | ≤ 60 s | Keeps the problem observable for MEMS |

`validate_data.m` implements these checks.

## 7. Versioning and compatibility

- Interface version: 0.1.0 (draft)
- Latest change: initial proposal
- Backward-compatibility policy: a new minor version may only **add** optional fields;
  any change of meaning, unit, or shape of an existing field requires a new major version
  agreed by all three teams.
