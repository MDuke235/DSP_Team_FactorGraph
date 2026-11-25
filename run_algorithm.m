function est = run_algorithm(data, opts)
%RUN_ALGORITHM Official algorithm entry point of the common interface (v0.1.0).
%   est = run_algorithm(data)          data: struct or path to a .mat dataset
%   est = run_algorithm(data, opts)    opts passed to fg_ins_gps_2d
%
%   The algorithm uses data.meta, data.imu, data.gps only. data.gt is never read.
setup_paths();
if ischar(data) || isstring(data), data = load_dataset(char(data)); end
if nargin < 2, opts = struct(); end
if isfield(data, 'gt'), data = rmfield(data, 'gt'); end   % guarantee: no peeking
est = fg_ins_gps_2d(data, opts);
end
