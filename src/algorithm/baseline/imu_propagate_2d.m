function x = imu_propagate_2d(x, b, acc, gyro, dt)
%IMU_PROPAGATE_2D Planar strapdown mechanization (Euler integration).
%   x    : 1x5 state [px py yaw vx vy] at the start of the interval
%   b    : 1x3 bias  [b_ax b_ay b_g]
%   acc  : Nx2 body-frame specific force samples inside the interval
%   gyro : Nx1 yaw-rate samples inside the interval
%   dt   : IMU sample period
%   Returns the state at the end of the interval.
p = x(1:2);  yaw = x(3);  v = x(4:5);
for i = 1:size(acc, 1)
    c = cos(yaw);  s = sin(yaw);
    ab = acc(i, :) - b(1:2);
    a  = [c*ab(1) - s*ab(2), s*ab(1) + c*ab(2)];   % body -> world
    p   = p + v*dt + 0.5*a*dt^2;
    v   = v + a*dt;
    yaw = yaw + (gyro(i) - b(3))*dt;
end
x = [p yaw v];
end
