function [ok, report] = validate_data(data, verbose)
%VALIDATE_DATA Check a dataset against interface v0.1.0 and the validity conditions.
%   [ok, report] = validate_data(data)
%   ok     : true if no check failed
%   report : cell array of messages ("FAIL ..." / "WARN ..." / "OK ...")
%
%   The physical limits below are the "validity conditions" of the Phase-1
%   agreement: challenge data from another team that violates them is rejected.

if nargin < 2, verbose = true; end
report = {};
ok = true;

% ---------- validity limits (keep in sync with docs/interface.md §6) ----------
L.max_speed     = 30;        % [m/s]   small UAV
L.max_acc       = 2*9.81;    % [m/s^2] horizontal specific force incl. noise
L.max_gyro      = 3;         % [rad/s]
L.sigma_acc     = [1e-3 0.5];    % [m/s^2] per-sample noise, MEMS range
L.sigma_gyro    = [1e-4 0.05];   % [rad/s]
L.sigma_gps     = [0.3 10];      % [m]
L.bias_acc_max  = 0.5;       % [m/s^2]
L.bias_gyro_max = 0.05;      % [rad/s]
L.max_outage    = 60;        % [s]

% ---------- structure ----------
need = {'meta', 'imu', 'gps'};
for k = 1:numel(need)
    check(isfield(data, need{k}), sprintf('top-level field "%s" exists', need{k}));
end
if ~ok, finish(); return; end

meta_fields = {'interface_version','imu_fs','gps_fs','sigma_acc','sigma_gyro', ...
    'bias_acc_sigma','bias_gyro_sigma','sigma_gps','gps_outages','init_state','init_sigma'};
for k = 1:numel(meta_fields)
    check(isfield(data.meta, meta_fields{k}), sprintf('meta.%s exists', meta_fields{k}));
end
if ~ok, finish(); return; end
check(strcmp(data.meta.interface_version(1:3), '0.1'), ...
    sprintf('interface version is 0.1.x (got %s)', data.meta.interface_version));

% ---------- IMU ----------
imu = data.imu;  N = numel(imu.t);  dt = 1/data.meta.imu_fs;
check(iscolumn(imu.t) && all(diff(imu.t) > 0), 'imu.t is a strictly increasing column');
check(max(abs(diff(imu.t) - dt)) < 1e-6, 'imu.t is uniformly sampled at meta.imu_fs');
check(isequal(size(imu.acc), [N 2]),  'imu.acc is N x 2');
check(isequal(size(imu.gyro), [N 1]), 'imu.gyro is N x 1');
check(all(isfinite(imu.acc(:))) && all(isfinite(imu.gyro)), 'IMU samples are finite');
check(max(sqrt(sum(imu.acc.^2, 2))) <= L.max_acc, sprintf('|acc| <= %.1f m/s^2', L.max_acc));
check(max(abs(imu.gyro)) <= L.max_gyro, sprintf('|gyro| <= %.1f rad/s', L.max_gyro));

% ---------- GPS ----------
gps = data.gps;  M = numel(gps.t);
check(iscolumn(gps.t) && all(diff(gps.t) > 0), 'gps.t is a strictly increasing column');
check(gps.t(1) >= imu.t(1) && gps.t(end) <= imu.t(end), 'GPS epochs lie inside the IMU time span');
align = abs((gps.t - imu.t(1))/dt - round((gps.t - imu.t(1))/dt));
check(max(align) < 0.1, 'GPS epochs coincide with IMU samples (synchronized clocks)');
check(isequal(size(gps.pos), [M 2]), 'gps.pos is M x 2');
check(isequal(size(gps.valid), [M 1]), 'gps.valid is M x 1');
v = logical(gps.valid);
check(all(all(isfinite(gps.pos(v, :)))), 'valid GPS fixes are finite');
check(nnz(v) >= 2, 'at least two valid GPS fixes');

% longest gap without valid GPS
tv = gps.t(v);
gap = max([diff(tv); 0]);
check(gap <= L.max_outage, sprintf('longest GPS gap %.1f s <= %d s', gap, L.max_outage));

% ---------- noise parameters ----------
m = data.meta;
inrange = @(x, r) x >= r(1) && x <= r(2);
check(inrange(m.sigma_acc,  L.sigma_acc),  sprintf('sigma_acc %.3g in MEMS range', m.sigma_acc));
check(inrange(m.sigma_gyro, L.sigma_gyro), sprintf('sigma_gyro %.3g in MEMS range', m.sigma_gyro));
check(inrange(m.sigma_gps,  L.sigma_gps),  sprintf('sigma_gps %.3g in GPS range', m.sigma_gps));
check(m.bias_acc_sigma  <= L.bias_acc_max,  'bias_acc_sigma realistic');
check(m.bias_gyro_sigma <= L.bias_gyro_max, 'bias_gyro_sigma realistic');

% ---------- ground truth (optional) ----------
if isfield(data, 'gt')
    sp = sqrt(sum(data.gt.vel.^2, 2));
    check(max(sp) <= L.max_speed, sprintf('true speed %.1f <= %d m/s', max(sp), L.max_speed));
    if isfield(data.gt, 'bias')
        check(all(abs(data.gt.bias(1:2)) <= L.bias_acc_max) && abs(data.gt.bias(3)) <= L.bias_gyro_max, ...
            'true biases realistic');
    end
else
    report{end+1} = 'WARN no ground truth (gt) -> dataset can be run but not scored';
end

finish();

    function check(cond, msg)
        if cond
            report{end+1} = ['OK   ' msg];
        else
            report{end+1} = ['FAIL ' msg];
            ok = false;
        end
    end

    function finish()
        if verbose
            fprintf('%s\n', report{:});
            if ok, fprintf('==> dataset VALID\n'); else, fprintf('==> dataset INVALID\n'); end
        end
    end
end
