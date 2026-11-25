# Algorithms

| File | Role |
|---|---|
| `baseline/imu_propagate_2d.m` | Planar strapdown mechanization (Euler integration) |
| `baseline/dead_reckoning_2d.m` | IMU-only reference (shows drift) |
| `baseline/fg_ins_gps_2d.m` | Batch factor-graph smoother, Levenberg–Marquardt, optional Huber GPS factors |
| `advanced/` | Phase-2 algorithms |

The algorithm reads data according to the common interface (docs/interface.md), returns `est`
in the required format, and is called through `run_algorithm.m`.
