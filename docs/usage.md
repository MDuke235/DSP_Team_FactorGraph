# Usage instructions

## Installation

- MATLAB R2019b or newer. The baseline uses base MATLAB only (no toolbox).
- Optional for Phase 2: Navigation Toolbox (`factorGraph`, `factorIMU`, `factorGPS`) or the GTSAM MATLAB toolbox.
- Open the repository root in MATLAB and run `setup_paths` once per session
  (the `run_*.m` scripts call it automatically).

## Data generation

```matlab
setup_paths;
cfg  = baseline_config();                 % edit parameters in configs/baseline_config.m
data = generate_baseline(cfg, 'data/baseline/baseline_seed42.mat');
[ok, report] = validate_data(data);       % must print "dataset VALID"
```

Change `cfg.seed` to create independent realizations (Monte-Carlo runs).

## Running the algorithm

```matlab
est = run_algorithm('data/baseline/baseline_seed42.mat');                 % baseline
est = run_algorithm(data, struct('robust', 'huber', 'huber_k', 1.345));   % robust GPS factors
est_dr = dead_reckoning_2d(data);                                         % IMU-only reference
```

## Evaluation

```matlab
run_all                                    % full pipeline + plots in artifacts/baseline/
M = evaluate_estimate(data, est);          % metrics struct
M = run_challenge('other_team_case.mat');  % cross-test another team's data
```

Expected behaviour of the baseline: dead reckoning drifts to tens or hundreds of metres within
two minutes because of the unknown biases; the factor graph stays below the GPS noise level,
estimates the biases, and its error grows only moderately inside the 20 s outage
(the 3σ bound widens there and shrinks again when GPS returns).
