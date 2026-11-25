# DSP Project 5 — Factor-Graph INS/GPS Fusion

Team repository for the DSP term project **"5. Factor-Graph INS/GPS Fusion"**:
estimate the trajectory of a UAV by fusing high-rate IMU measurements with
low-rate GPS fixes, formulated as a factor graph and solved as a nonlinear
least-squares problem, robust to GPS outages and outliers.

> Team: **Team-XX** (fill in [GROUP_INFO.md](GROUP_INFO.md)) · Language: **MATLAB** (base MATLAB, no toolbox required for the baseline) · Interface version: **0.1.0**

## Getting started

1. Rename the repository using your team/topic identifier, e.g. `DSP-P5-FactorGraph-INS-GPS-TeamXX`.
2. Fill in [GROUP_INFO.md](GROUP_INFO.md) and add the instructor (`V.nghiabac@gmail.com`) as a collaborator.
3. Discuss [docs/phase1-agreement-draft.md](docs/phase1-agreement-draft.md) with the two other Project-5 teams, then freeze it in [docs/interface.md](docs/interface.md) once the instructor approves it.
4. Run the baseline pipeline (below) and check that you understand every line of it.
5. Keep the work log up to date in [logs/](logs/).

## Quick start

From MATLAB, with the repository root as the current folder:

```matlab
run_all                      % generate -> validate -> run -> evaluate -> plot
```

or from a terminal:

```text
matlab -batch "run_all"
```

`run_all` produces `data/baseline/baseline_seed42.mat`, prints the metrics for
IMU-only dead reckoning and for the factor-graph smoother, and saves plots to
`artifacts/baseline/`.

Cross-testing another team's dataset:

```matlab
M = run_challenge('path/to/Team-YY_case1.mat');                       % baseline solver
M = run_challenge('path/to/Team-YY_case1.mat', struct('robust','huber'));  % with outlier rejection
```

## Repository structure

```text
.
├── GROUP_INFO.md                 # Team and topic information
├── AI_USAGE.md                   # AI usage declaration
├── README.md
├── run_all.m                     # End-to-end baseline pipeline
├── run_algorithm.m               # Official algorithm entry point (interface)
├── run_challenge.m               # Run our algorithm on another team's data
├── setup_paths.m                 # Adds configs/ and src/ to the MATLAB path
├── configs/
│   ├── baseline_config.m         # Baseline data-model parameters
│   └── project.yaml              # Project metadata (interface version, commands)
├── docs/
│   ├── phase1-agreement-draft.md # Draft agreement to discuss with the other 2 teams
│   ├── interface.md              # Data package, algorithm output, entry point
│   ├── data-model.md             # Baseline/advanced model and justification
│   └── usage.md                  # How to run everything
├── src/
│   ├── common/                   # wrap_to_pi, load_dataset
│   ├── data_generator/
│   │   ├── baseline/             # generate_baseline.m
│   │   └── advanced/             # Phase-2 extensions (3D, outliers, lever arm, ...)
│   ├── algorithm/
│   │   ├── baseline/             # fg_ins_gps_2d.m, imu_propagate_2d.m, dead_reckoning_2d.m
│   │   └── advanced/             # Phase-2 algorithms (preintegration, iSAM-style, ...)
│   └── evaluator/                # validate_data.m, evaluate_estimate.m, plot_results.m
├── data/
│   ├── baseline/                 # Generated baseline data (+ manifest)
│   ├── advanced/                 # Team-generated advanced data
│   └── tests/                    # Test cases
├── reports/                      # Technical report and appendices
├── slides/                       # Presentation slides
├── logs/
│   ├── meetings/                 # Meeting minutes
│   └── work/                     # Work logs
└── artifacts/                    # Run results, plots, demos, and evidence
```

## Baseline at a glance

| Component | Baseline (Phase 1 agreement, 5/10) | Advanced ideas (Phase 2, towards 10/10) |
|---|---|---|
| Motion | Planar figure-eight, 5–15 m/s | 3D flight, climbs/turns, hover |
| IMU | 100 Hz, white noise + constant bias | Bias random walk, scale factor, vibration |
| GPS | 1 Hz, σ = 2 m, one 20 s outage | Outliers / multipath, longer outages, lever arm, time offset |
| Algorithm | Batch factor graph, LM, numerical Jacobians | IMU preintegration, Huber/Cauchy kernels, sliding window / incremental smoothing |
| Reference | IMU-only dead reckoning | EKF loosely-coupled comparison |

## Challenge data for other teams

Challenge datasets must be uploaded to the shared [Google Drive folder](https://drive.google.com/drive/folders/1ITuDtuFODzus5bAaQVihR5sXhCHIapS5?usp=sharing),
at the latest **48 h before the defence**, under:

```text
Projects/
└── Project-5/
    ├── Team-xx/
    └── ...
```

Each team must:

1. Create one folder named `Team-xx` inside `Projects/Project-5/`.
2. Upload its challenge datasets into that team folder.
3. Keep the folder name consistent with the team ID in `GROUP_INFO.md`.
4. Include a short `README.md` or `manifest` describing each dataset, its configuration, its expected format, and the reason why the simulated cases are realistic.
5. Do not upload passwords, tokens, personal data, or unrelated files.

The repository contains the generator and documentation for the challenge data; the shared Drive folder stores the data files.

## Important rules

- Do not commit passwords, tokens, personal data, or local environment files.
- Every important simulated dataset must have a description and justification in the documentation.
- Commits do not replace work logs. Logs should record what was tried, what happened, and what decisions were made.
- Every component must include instructions for reproducing its results.
- Ask the instructor before using Git LFS or external storage for large files.
- Only submit what the team understands and can explain (see [AI_USAGE.md](AI_USAGE.md)).

## Minimum run commands

```text
# Install the environment
MATLAB R2019b or newer (base MATLAB only). Open the repository root in MATLAB.

# Generate baseline data
matlab -batch "setup_paths; generate_baseline(baseline_config(), 'data/baseline/baseline_seed42.mat');"

# Generate advanced data
TODO (Phase 2): src/data_generator/advanced/

# Run the algorithm
matlab -batch "est = run_algorithm('data/baseline/baseline_seed42.mat');"

# Run the evaluator (full pipeline incl. metrics and plots)
matlab -batch "run_all"
```
