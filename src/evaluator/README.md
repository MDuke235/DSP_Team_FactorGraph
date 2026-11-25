# Evaluator and validator

| File | Role |
|---|---|
| `validate_data.m` | Interface + validity-condition checks; returns `ok` and a report |
| `evaluate_estimate.m` | Metrics against ground truth |
| `plot_results.m` | Trajectory plot and error-vs-time plot with outage shading and 3σ bound |

- **Metric formulas, units:** docs/interface.md §5 (primary: horizontal position RMSE in metres, lower is better).
- **Validity conditions:** docs/interface.md §6 (limits are defined at the top of `validate_data.m`).
- **Error handling:** `evaluate_estimate` asserts on missing fields, wrong shapes, or NaN output;
  `validate_data` never throws, it reports FAIL lines.
- **How to run:** `run_all` (own data) or `run_challenge(file)` (other team's data).
- **Example result:** TODO — paste the console output of `run_all` here after the first run.
