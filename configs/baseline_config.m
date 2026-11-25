function cfg = baseline_config()
%BASELINE_CONFIG Parameters of the Phase-1 baseline data model (interface v0.1.0).
%   All units are SI (m, s, rad). Change values here, never inside the generator.
%   Every value below must match docs/data-model.md and the Phase-1 agreement.

cfg.interface_version = '0.1.0';
cfg.seed      = 42;          % RNG seed -> reproducible dataset
cfg.duration  = 120;         % [s] length of the trajectory

% ---- Ground-truth trajectory: planar figure-eight (Lissajous) -------------
% p(t) = [A*sin(w t), B*sin(2 w t)], heading = direction of velocity
cfg.traj.A      = 200;       % [m]
cfg.traj.B      = 100;       % [m]
cfg.traj.period = 120;       % [s] one full figure-eight
                             % -> speed ~ 5..15 m/s, typical small UAV

% ---- IMU (consumer MEMS class) --------------------------------------------
cfg.imu.fs          = 100;   % [Hz]
cfg.imu.sigma_acc   = 0.02;  % [m/s^2] white noise std per sample
cfg.imu.sigma_gyro  = 0.002; % [rad/s] white noise std per sample
cfg.imu.bias_acc_sigma  = 0.05;   % [m/s^2] std of the constant accel bias draw
cfg.imu.bias_gyro_sigma = 0.005;  % [rad/s] std of the constant gyro bias draw

% ---- GPS --------------------------------------------------------------------
cfg.gps.fs     = 1;          % [Hz]
cfg.gps.sigma  = 2.0;        % [m] horizontal position noise std
cfg.gps.outages = [50 70];   % [s] rows of [t_start t_end): GPS unavailable
cfg.gps.outlier_rate = 0;    % baseline: no outliers (advanced: e.g. 0.05)
cfg.gps.outlier_mag  = 30;   % [m] std of outlier jumps (used only if rate > 0)

% ---- Initial state information given to the algorithm -----------------------
% The algorithm receives a noisy initial state and its std (like a rough alignment).
cfg.init.sigma = [2 2 0.1 0.5 0.5];  % [px py yaw vx vy] -> [m m rad m/s m/s]
end
