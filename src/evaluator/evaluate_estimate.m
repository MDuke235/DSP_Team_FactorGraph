function M = evaluate_estimate(data, est, verbose)
%EVALUATE_ESTIMATE Score an algorithm output against ground truth (interface v0.1.0).
%   M = evaluate_estimate(data, est)
%
%   Metrics (all position errors are horizontal Euclidean errors in metres):
%     rmse_pos        RMSE over all output epochs              <- PRIMARY METRIC
%     rmse_pos_gps    RMSE over epochs where GPS was available
%     rmse_pos_outage RMSE over epochs inside GPS outages
%     max_err_outage  maximum error inside GPS outages
%     final_err       error at the last epoch
%     rmse_yaw_deg    heading RMSE [deg]
%     bias_err        |b_est - b_true| per component [m/s^2 m/s^2 rad/s]
%   Lower is better for every metric.

if nargin < 3, verbose = true; end
assert(isfield(data, 'gt'), 'evaluate_estimate: dataset has no ground truth.');

% ---- output format check ----
need = {'t', 'pos'};
for k = 1:numel(need)
    assert(isfield(est, need{k}), 'evaluate_estimate: est.%s missing', need{k});
end
K = numel(est.t);
assert(isequal(size(est.pos), [K 2]), 'evaluate_estimate: est.pos must be K x 2');
assert(all(isfinite(est.pos(:))), 'evaluate_estimate: est.pos has NaN/Inf');

gt = data.gt;
p_true = interp1(gt.t, gt.pos, est.t, 'linear');
err = sqrt(sum((est.pos - p_true).^2, 2));

% epochs inside outage windows
out = false(K, 1);
og = data.meta.gps_outages;
for r = 1:size(og, 1)
    out = out | (est.t >= og(r,1) & est.t < og(r,2));
end

rms = @(x) sqrt(mean(x.^2));
M.method          = get_or(est, 'method', 'unknown');
M.rmse_pos        = rms(err);
M.rmse_pos_gps    = rms(err(~out));
M.rmse_pos_outage = NaN;  M.max_err_outage = NaN;
if any(out)
    M.rmse_pos_outage = rms(err(out));
    M.max_err_outage  = max(err(out));
end
M.final_err = err(end);
M.rmse_yaw_deg = NaN;
if isfield(est, 'yaw')
    y_true = interp1(gt.t, gt.yaw, est.t, 'linear');
    M.rmse_yaw_deg = rms(wrap_to_pi(est.yaw(:) - y_true)) * 180/pi;
end
M.bias_err = [NaN NaN NaN];
if isfield(est, 'bias') && isfield(gt, 'bias')
    M.bias_err = abs(est.bias(:).' - gt.bias(:).');
end
M.err = err;          % per-epoch error, kept for plotting
M.outage_mask = out;

if verbose
    fprintf('--- %s ---\n', M.method);
    fprintf('  RMSE pos (all)      : %8.3f m   <- primary\n', M.rmse_pos);
    fprintf('  RMSE pos (GPS on)   : %8.3f m\n', M.rmse_pos_gps);
    fprintf('  RMSE pos (outage)   : %8.3f m\n', M.rmse_pos_outage);
    fprintf('  max err (outage)    : %8.3f m\n', M.max_err_outage);
    fprintf('  final error         : %8.3f m\n', M.final_err);
    fprintf('  RMSE yaw            : %8.3f deg\n', M.rmse_yaw_deg);
    fprintf('  bias error          : [%.4f %.4f %.5f]\n', M.bias_err);
end
end

function v = get_or(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
