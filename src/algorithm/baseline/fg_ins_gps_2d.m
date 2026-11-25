function est = fg_ins_gps_2d(data, opts)
%FG_INS_GPS_2D Baseline factor-graph INS/GPS smoother for planar motion.
%   est = fg_ins_gps_2d(data)          default options
%   est = fg_ins_gps_2d(data, opts)    see "Options" below
%
%   Unknowns (one state node per GPS epoch k = 1..K, plus one global bias):
%       x_k = [px py yaw vx vy],   b = [b_ax b_ay b_g] (constant in baseline)
%
%   Factors (each is a weighted residual r = (prediction - measurement)/sigma):
%       prior on x_1        : x_1 - init_state
%       prior on b          : b - 0
%       IMU factor  k->k+1  : x_{k+1} - f(x_k, b, IMU samples in [t_k, t_{k+1}))
%       GPS factor  k       : p_k - z_k            (only where gps.valid)
%
%   Solver: Levenberg-Marquardt on the sparse nonlinear least-squares problem
%       min_X  sum_i rho( ||r_i(X)||^2 )
%   with numerical Jacobians for the IMU factor (simple and easy to check).
%
%   Options (fields of opts):
%       max_iter    [30]       maximum LM iterations
%       tol         [1e-6]     stop when max |dx| < tol
%       robust      ['none']   'none' or 'huber' (applied to GPS factors)
%       huber_k     [1.345]    Huber threshold in units of sigma
%       sigma_floor [0.02 0.02 1e-3 0.01 0.01]  minimum IMU-factor std
%       verbose     [true]

if nargin < 2, opts = struct(); end
opts = set_default(opts, 'max_iter', 30);
opts = set_default(opts, 'tol', 1e-6);
opts = set_default(opts, 'robust', 'none');
opts = set_default(opts, 'huber_k', 1.345);
opts = set_default(opts, 'sigma_floor', [0.02 0.02 1e-3 0.01 0.01]);
opts = set_default(opts, 'verbose', true);

meta = data.meta;
ctx.dt   = 1 / meta.imu_fs;
ctx.acc  = data.imu.acc;
ctx.gyro = data.imu.gyro;
ctx.idx  = round((data.gps.t - data.imu.t(1)) * meta.imu_fs) + 1;  % IMU sample of each epoch
ctx.K    = numel(ctx.idx);
ctx.n    = 5*ctx.K + 3;
ctx.z    = data.gps.pos;
ctx.valid = logical(data.gps.valid(:)) & all(isfinite(data.gps.pos), 2);
ctx.sigma_gps = meta.sigma_gps;
ctx.x0_prior  = meta.init_state(:).';
ctx.sigma_x0  = meta.init_sigma(:).';
ctx.sigma_b   = 2 * [meta.bias_acc_sigma meta.bias_acc_sigma meta.bias_gyro_sigma];
ctx.robust    = lower(opts.robust);
ctx.huber_k   = opts.huber_k;

% ---- IMU-factor uncertainty from white-noise propagation over each interval ----
%   velocity random walk : sigma_v   = sigma_acc  * sqrt(N) * dt
%   heading random walk  : sigma_yaw = sigma_gyro * sqrt(N) * dt
%   position (integrated): sigma_p   = sigma_v * T / sqrt(3)
Nk  = diff(ctx.idx);
Tk  = Nk * ctx.dt;
sv  = meta.sigma_acc  * sqrt(Nk) * ctx.dt;
sy  = meta.sigma_gyro * sqrt(Nk) * ctx.dt;
sp  = sv .* Tk / sqrt(3);
ctx.sigma_imu = max([sp sp sy sv sv], repmat(opts.sigma_floor, ctx.K-1, 1));

% ---- initial guess: dead reckoning from the prior, zero bias ----
X = zeros(ctx.n, 1);
X(1:5) = ctx.x0_prior;
for k = 1:ctx.K-1
    s = ctx.idx(k):ctx.idx(k+1)-1;
    X(sid(k+1)) = imu_propagate_2d(X(sid(k)).', [0 0 0], ctx.acc(s,:), ctx.gyro(s), ctx.dt);
end

% ---- Levenberg-Marquardt ----
lambda = 1e-3;
[r, J, cost] = build_system(X, ctx, true);
for it = 1:opts.max_iter
    H  = J.' * J;
    g  = J.' * r;
    D  = spdiags(diag(H), 0, ctx.n, ctx.n);
    dx = -(H + lambda*D) \ g;
    Xn = X + dx;
    [~, ~, cost_new] = build_system(Xn, ctx, false);
    if cost_new < cost
        X = Xn;
        lambda = max(lambda/10, 1e-9);
        [r, J, cost] = build_system(X, ctx, true);
        if opts.verbose
            fprintf('  LM it %2d  cost %12.4f  max|dx| %.2e  lambda %.1e\n', it, cost, max(abs(dx)), lambda);
        end
        if max(abs(dx)) < opts.tol, break; end
    else
        lambda = lambda * 10;
        if lambda > 1e10, break; end
    end
end

% ---- outputs ----
S = reshape(X(1:5*ctx.K), 5, ctx.K).';
est.t    = data.gps.t;
est.pos  = S(:, 1:2);
est.yaw  = wrap_to_pi(S(:, 3));
est.vel  = S(:, 4:5);
est.bias = X(end-2:end).';
est.method = sprintf('fg_ins_gps_2d (robust=%s)', ctx.robust);
est.info.iterations = it;
est.info.final_cost = cost;
est.info.n_gps_used = nnz(ctx.valid);

% marginal standard deviations from the information matrix H = J'J
H = J.' * J;
if ctx.n <= 6000
    P = inv(full(H));
    dP = diag(P);
    est.pos_std  = sqrt([dP(1:5:5*ctx.K) dP(2:5:5*ctx.K)]);
    est.bias_std = sqrt(dP(end-2:end)).';
end
end

% =========================================================================
function c = sid(k)
%SID Column indices of state node k in the unknown vector.
c = 5*(k-1) + (1:5);
end

function [r, J, cost] = build_system(X, ctx, needJ)
%BUILD_SYSTEM Stack all weighted residuals and (optionally) the sparse Jacobian.
K = ctx.K;  n = ctx.n;  bcol = n-2:n;
nv = nnz(ctx.valid);
m  = 5 + 3 + 5*(K-1) + 2*nv;
r  = zeros(m, 1);
nt = 25 + 9 + 65*(K-1) + 4*nv;              % Jacobian entries (dense blocks)
I = zeros(nt, 1);  Jc = zeros(nt, 1);  V = zeros(nt, 1);
row = 0;  t = 0;
cost_gps = 0;

% --- prior on x_1 ---
e = X(1:5).' - ctx.x0_prior;  e(3) = wrap_to_pi(e(3));
r(row+(1:5)) = e ./ ctx.sigma_x0;
if needJ, add_block(row+(1:5), 1:5, diag(1 ./ ctx.sigma_x0)); end
row = row + 5;

% --- prior on bias ---
b = X(bcol).';
r(row+(1:3)) = b ./ ctx.sigma_b;
if needJ, add_block(row+(1:3), bcol, diag(1 ./ ctx.sigma_b)); end
row = row + 3;

% --- IMU factors ---
epsd = 1e-6;
for k = 1:K-1
    s   = ctx.idx(k):ctx.idx(k+1)-1;
    xk  = X(sid(k)).';  xn = X(sid(k+1)).';
    f   = imu_propagate_2d(xk, b, ctx.acc(s,:), ctx.gyro(s), ctx.dt);
    e   = xn - f;  e(3) = wrap_to_pi(e(3));
    w   = 1 ./ ctx.sigma_imu(k, :);
    r(row+(1:5)) = (e .* w).';
    if needJ
        y  = [xk b];
        Jf = zeros(5, 8);
        for j = 1:8                                   % numerical df/d[x_k, b]
            yp = y;  yp(j) = yp(j) + epsd;
            Jf(:, j) = (imu_propagate_2d(yp(1:5), yp(6:8), ctx.acc(s,:), ctx.gyro(s), ctx.dt) - f).' / epsd;
        end
        Wd = diag(w);
        add_block(row+(1:5), sid(k+1), Wd);           % d e / d x_{k+1} = I
        add_block(row+(1:5), sid(k),   -Wd*Jf(:,1:5));
        add_block(row+(1:5), bcol,     -Wd*Jf(:,6:8));
    end
    row = row + 5;
end

% --- GPS factors (optionally robust) ---
for k = find(ctx.valid).'
    c = sid(k);
    e = (X(c(1:2)).' - ctx.z(k, :)) / ctx.sigma_gps;
    en = norm(e);
    if strcmp(ctx.robust, 'huber') && en > ctx.huber_k
        sw = sqrt(ctx.huber_k / en);                  % IRLS weight sqrt(w)
        cost_gps = cost_gps + ctx.huber_k*en - 0.5*ctx.huber_k^2;
    else
        sw = 1;
        cost_gps = cost_gps + 0.5*en^2;
    end
    r(row+(1:2)) = sw * e.';
    if needJ, add_block(row+(1:2), c(1:2), sw*eye(2)/ctx.sigma_gps); end
    row = row + 2;
end

ng = 2*nv;
cost = 0.5*sum(r(1:end-ng).^2) + cost_gps;
if needJ
    J = sparse(I(1:t), Jc(1:t), V(1:t), m, n);
else
    J = [];
end

    function add_block(rows, cols, B)
        [rr, cc] = ndgrid(rows, cols);
        q = numel(B);
        I(t+(1:q))  = rr(:);
        Jc(t+(1:q)) = cc(:);
        V(t+(1:q))  = B(:);
        t = t + q;
    end
end

function s = set_default(s, name, value)
if ~isfield(s, name) || isempty(s.(name)), s.(name) = value; end
end
