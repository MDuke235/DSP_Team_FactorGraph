# Data generator

- `baseline/generate_baseline.m`: data following the common model defined in Phase 1
  (parameters in `configs/baseline_config.m`, seed for reproducibility).
- `advanced/`: extensions designed by the team in Phase 2 (see docs/data-model.md, "Advanced model").

Run:

```matlab
setup_paths;
data = generate_baseline(baseline_config(), 'data/baseline/baseline_seed42.mat');
```

Each generator must have a configuration, a reproducible seed, and run instructions.
