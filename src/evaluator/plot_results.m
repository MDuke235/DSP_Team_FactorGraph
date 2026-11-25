function plot_results(data, ests, metrics, outdir)
%PLOT_RESULTS Trajectory and error plots; saves PNG files to outdir.
%   ests, metrics : cell arrays (same order), e.g. {est_dr, est_fg}

if nargin < 4, outdir = ''; end
og = data.meta.gps_outages;
v  = logical(data.gps.valid);

% ---------- trajectory ----------
f1 = figure('Name', 'Trajectory');  hold on;  grid on;  axis equal;
plot(data.gt.pos(:,1), data.gt.pos(:,2), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground truth');
plot(data.gps.pos(v,1), data.gps.pos(v,2), '.', 'Color', [0.85 0.33 0.1], ...
    'MarkerSize', 10, 'DisplayName', 'GPS fixes');
styles = {'--', '-'};
for i = 1:numel(ests)
    plot(ests{i}.pos(:,1), ests{i}.pos(:,2), styles{min(i,2)}, 'LineWidth', 1.2, ...
        'DisplayName', ests{i}.method);
end
for r = 1:size(og, 1)                          % mark the true path inside outages
    m = data.gt.t >= og(r,1) & data.gt.t < og(r,2);
    plot(data.gt.pos(m,1), data.gt.pos(m,2), 'm-', 'LineWidth', 4, ...
        'DisplayName', sprintf('GPS outage %g-%g s', og(r,1), og(r,2)));
end
xlabel('East [m]');  ylabel('North [m]');  legend('Location', 'best');
title('INS/GPS fusion: trajectory');

% ---------- error vs time ----------
f2 = figure('Name', 'Position error');  hold on;  grid on;
yl = 0;
for i = 1:numel(ests)
    yl = max(yl, max(metrics{i}.err));
end
yl = min(yl, 50);                               % DR error explodes; keep FG visible
for r = 1:size(og, 1)
    patch([og(r,1) og(r,2) og(r,2) og(r,1)], [0 0 yl yl], [1 0.85 1], ...
        'EdgeColor', 'none', 'DisplayName', 'GPS outage');
end
for i = 1:numel(ests)
    plot(ests{i}.t, metrics{i}.err, styles{min(i,2)}, 'LineWidth', 1.2, 'DisplayName', ests{i}.method);
    if isfield(ests{i}, 'pos_std')
        s3 = 3*sqrt(sum(ests{i}.pos_std.^2, 2));
        plot(ests{i}.t, s3, ':', 'LineWidth', 1.2, 'DisplayName', [ests{i}.method ' 3\sigma']);
    end
end
ylim([0 yl]);  xlabel('Time [s]');  ylabel('Horizontal position error [m]');
legend('Location', 'best');  title('Position error vs time');

if ~isempty(outdir)
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    saveas(f1, fullfile(outdir, 'trajectory.png'));
    saveas(f2, fullfile(outdir, 'position_error.png'));
    fprintf('Saved plots to %s\n', outdir);
end
end
