%RUN_ALL Reproduce the baseline pipeline end to end:
%   generate data -> validate -> run algorithms -> evaluate -> plot.
%   Usage (from the repository root):   run_all
%   or from a terminal:                 matlab -batch "run_all"
clear; close all; clc;
setup_paths();

root    = fileparts(mfilename('fullpath'));
datafile = fullfile(root, 'data', 'baseline', 'baseline_seed42.mat');
outdir   = fullfile(root, 'artifacts', 'baseline');

% 1) generate baseline data
cfg  = baseline_config();
data = generate_baseline(cfg, datafile);

% 2) validate against the interface + validity conditions
ok = validate_data(data);
assert(ok, 'Generated dataset is not valid - fix the generator first.');

% 3) run algorithms
fprintf('\nRunning dead reckoning (IMU only) ...\n');
est_dr = dead_reckoning_2d(data);
fprintf('Running factor-graph smoother ...\n');
est_fg = run_algorithm(data);

% 4) evaluate
fprintf('\n');
M_dr = evaluate_estimate(data, est_dr);
M_fg = evaluate_estimate(data, est_fg);
fprintf('  true bias           : [%.4f %.4f %.5f]\n', data.gt.bias);
fprintf('  estimated bias      : [%.4f %.4f %.5f]\n', est_fg.bias);

% 5) plots + saved results
plot_results(data, {est_dr, est_fg}, {M_dr, M_fg}, outdir);
save(fullfile(outdir, 'results.mat'), 'est_dr', 'est_fg', 'M_dr', 'M_fg');
fprintf('Results saved to %s\n', outdir);
