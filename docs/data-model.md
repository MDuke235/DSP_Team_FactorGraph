# Data model

## Baseline

Implemented in `src/data_generator/baseline/generate_baseline.m`, parameters in `configs/baseline_config.m`.

**Ground truth.** Planar figure-eight (Lissajous) trajectory

- p(t) = [A sin(ωt), B sin(2ωt)], A = 200 m, B = 100 m, ω = 2π/120 s
- v(t) = ṗ(t), a(t) = p̈(t) (analytic, no numerical differentiation)
- heading ψ(t) = atan2(v_y, v_x) (vehicle points along its velocity), ψ̇ = (v_x a_y − v_y a_x)/‖v‖²
- speed between ≈ 5 and 15 m/s, never zero

**IMU (100 Hz).** Body frame x forward, y left.

- accelerometer: f_b = R(ψ)ᵀ a + b_a + n_a, n_a ~ N(0, σ_a²), σ_a = 0.02 m/s²
- gyro: ω_m = ψ̇ + b_g + n_g, n_g ~ N(0, σ_g²), σ_g = 0.002 rad/s
- biases constant per dataset: b_a ~ N(0, 0.05²) m/s², b_g ~ N(0, 0.005²) rad/s

**GPS (1 Hz).** z_k = p(t_k) + n_k, n_k ~ N(0, σ²I), σ = 2 m; no data for t ∈ [50, 70) s.

**Initial information.** `meta.init_state` = true initial state + N(0, diag(init_sigma²)),
init_sigma = [2 m, 2 m, 0.1 rad, 0.5 m/s, 0.5 m/s].

**Reproducibility.** `rng(seed, 'twister')`, seed = 42.

## Baseline algorithm (summary)

- Mechanization (`imu_propagate_2d.m`): for each IMU sample,
  a = R(ψ)(f_b − b_a), p ← p + v dt + ½ a dt², v ← v + a dt, ψ ← ψ + (ω_m − b_g) dt.
- Factor graph (`fg_ins_gps_2d.m`): one node x_k = [p, ψ, v] per GPS epoch plus one bias node b.
  Factors: prior on x_1, prior on b, IMU factor x_{k+1} − f(x_k, b), GPS factor p_k − z_k.
- IMU-factor std from white-noise propagation over one interval of N samples:
  σ_v = σ_a √N dt, σ_ψ = σ_g √N dt, σ_p = σ_v T/√3, with small floors for discretization error.
- Solver: Levenberg–Marquardt on the sparse normal equations, numerical Jacobians.
- Uncertainty: marginal covariances from (JᵀJ)⁻¹ → 3σ bound in the error plot.

## Advanced model

TODO (Phase 2). Candidates, each must come with a justification and parameter range:

| Phenomenon | Model | Typical range | Why it matters |
|---|---|---|---|
| 3D motion | full SO(3)/quaternion attitude, gravity | climb rate ≤ 5 m/s | Real UAV flight |
| Bias random walk | ḃ = w_b | from Allan variance of the chosen IMU | Bias not constant over minutes |
| GPS outliers / multipath | mixture: N(0, σ²) w.p. 1−ε, large jump w.p. ε | ε = 1–10 %, 10–50 m | Urban canyons, needs robust kernels |
| Lever arm | antenna offset r in body frame: z = p + R(ψ) r | 0.1–1 m | Antenna is not at the IMU |
| Time offset | GPS timestamp delayed by τ | 10–200 ms | Unsynchronized clocks |
| Longer outages | 30–60 s | – | Tests drift and observability |

## Real-world justification

- Noise levels correspond to consumer-grade MEMS IMUs (accelerometer noise density ≈ 200 µg/√Hz,
  gyro ≈ 0.01 °/s/√Hz at 100 Hz) — TODO: cite the datasheet the team chooses (e.g. Bosch BMI088, ADIS16470).
- σ_GPS = 2 m is typical for a standalone single-frequency receiver in open sky.
- TODO: add references (Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*;
  Dellaert & Kaess, *Factor Graphs for Robot Perception*; Forster et al., IEEE T-RO 2017).

## Model limitations

- Planar motion only; no gravity, roll, pitch, or altitude.
- Constant biases; no scale-factor errors, misalignment, or temperature effects.
- Gaussian, independent GPS errors (real GPS errors are time-correlated).
- Perfect time synchronization and zero lever arm.
- Euler integration in both generator truth and algorithm (truth is analytic; algorithm discretization error is small at 100 Hz).
