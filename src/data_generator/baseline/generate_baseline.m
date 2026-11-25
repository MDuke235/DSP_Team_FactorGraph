function data = generate_baseline(cfg, outfile)
%GENERATE_BASELINE Simulate planar IMU + GPS data with ground truth (interface v0.1.0).
%   data = generate_baseline(cfg)            returns the data struct
%   data = generate_baseline(cfg, outfile)   also saves it to a .mat file
%
%   Model (see docs/data-model.md):
%     truth   : figure-eight trajectory, heading aligned with velocity
%     IMU     : acc_b = R(yaw)' * a_world + b_a + n_a ,  gyro = yaw_dot + b_g + n_g
%     GPS     : z = p + n_gps, removed inside outage windows
%     biases  : constant, drawn once per dataset

if nargin < 1 || isempty(cfg), cfg = baseline_config(); end
rng(cfg.seed, 'twister');

% ---------------- ground truth at IMU rate ----------------
dt = 1 / cfg.imu.fs;
t  = (0:round(cfg.duration * cfg.imu.fs)).' * dt;
A = cfg.traj.A;  B = cfg.traj.B;  w = 2*pi / cfg.traj.period;

px = A*sin(w*t);              py = B*sin(2*w*t);
vx = A*w*cos(w*t);            vy = 2*B*w*cos(2*w*t);
ax = -A*w^2*sin(w*t);         ay = -4*B*w^2*sin(2*w*t);

yaw   = unwrap(atan2(vy, vx));
omega = (vx.*ay - vy.*ax) ./ (vx.^2 + vy.^2);   % d(yaw)/dt

% world acceleration rotated into the body frame (x forward, y left)
c = cos(yaw);  s = sin(yaw);
acc_b_true = [ c.*ax + s.*ay, -s.*ax + c.*ay ];

% ---------------- IMU measurements ----------------
N   = numel(t);
b_a = cfg.imu.bias_acc_sigma  * randn(1, 2);
b_g = cfg.imu.bias_gyro_sigma * randn();

imu.t    = t;
imu.acc  = acc_b_true + b_a + cfg.imu.sigma_acc  * randn(N, 2);
imu.gyro = omega      + b_g + cfg.imu.sigma_gyro * randn(N, 1);

% ---------------- GPS measurements ----------------
tg  = (0 : 1/cfg.gps.fs : cfg.duration).';
idx = round(tg * cfg.imu.fs) + 1;               % GPS epochs coincide with IMU samples
M   = numel(tg);

gps.t     = tg;
gps.pos   = [px(idx), py(idx)] + cfg.gps.sigma * randn(M, 2);
gps.valid = true(M, 1);
for r = 1:size(cfg.gps.outages, 1)
    gps.valid(tg >= cfg.gps.outages(r,1) & tg < cfg.gps.outages(r,2)) = false;
end
is_outlier = false(M, 1);
if cfg.gps.outlier_rate > 0
    is_outlier = gps.valid & (rand(M, 1) < cfg.gps.outlier_rate);
    gps.pos(is_outlier, :) = gps.pos(is_outlier, :) + ...
        cfg.gps.outlier_mag * randn(nnz(is_outlier), 2);
end
gps.pos(~gps.valid, :) = NaN;                    % no data during outage

% ---------------- metadata (everything the algorithm may use) --------------
x0_true = [px(1) py(1) yaw(1) vx(1) vy(1)];
meta.interface_version = cfg.interface_version;
meta.description = 'Baseline planar INS/GPS dataset (figure-eight UAV trajectory)';
meta.frame       = 'local ENU, planar (x=East, y=North); body: x forward, y left';
meta.units       = 'SI: m, s, rad, m/s, m/s^2, rad/s';
meta.seed        = cfg.seed;
meta.imu_fs      = cfg.imu.fs;
meta.gps_fs      = cfg.gps.fs;
meta.sigma_acc   = cfg.imu.sigma_acc;
meta.sigma_gyro  = cfg.imu.sigma_gyro;
meta.bias_acc_sigma  = cfg.imu.bias_acc_sigma;
meta.bias_gyro_sigma = cfg.imu.bias_gyro_sigma;
meta.sigma_gps   = cfg.gps.sigma;
meta.gps_outages = cfg.gps.outages;
meta.init_state  = x0_true + cfg.init.sigma .* randn(1, 5);   % noisy initial guess
meta.init_sigma  = cfg.init.sigma;
meta.state_order = '[px py yaw vx vy]';

% ---------------- ground truth (evaluation only, NOT for the algorithm) ------
gt.t    = t;
gt.pos  = [px py];
gt.vel  = [vx vy];
gt.yaw  = yaw;
gt.bias = [b_a b_g];          % [b_ax b_ay b_g]
gt.gps_outlier = is_outlier;

data.meta = meta;
data.imu  = imu;
data.gps  = gps;
data.gt   = gt;

if nargin >= 2 && ~isempty(outfile)
    outdir = fileparts(outfile);
    if ~isempty(outdir) && ~exist(outdir, 'dir'), mkdir(outdir); end
    save(outfile, '-struct', 'data');
    fprintf('Saved baseline dataset to %s\n', outfile);
end
end
