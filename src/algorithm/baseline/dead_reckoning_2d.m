function est = dead_reckoning_2d(data)
%DEAD_RECKONING_2D IMU-only reference solution (no GPS, zero bias assumed).
%   Used in the report to show how fast pure inertial navigation drifts.
dt  = 1 / data.meta.imu_fs;
idx = round((data.gps.t - data.imu.t(1)) * data.meta.imu_fs) + 1;
K   = numel(idx);
X   = zeros(K, 5);
X(1, :) = data.meta.init_state;
for k = 1:K-1
    s = idx(k):idx(k+1)-1;
    X(k+1, :) = imu_propagate_2d(X(k, :), [0 0 0], data.imu.acc(s, :), data.imu.gyro(s), dt);
end
est.t    = data.gps.t;
est.pos  = X(:, 1:2);
est.yaw  = X(:, 3);
est.vel  = X(:, 4:5);
est.bias = [0 0 0];
est.method = 'dead_reckoning_2d';
end
