function M = run_challenge(datafile, opts)
%RUN_CHALLENGE Cross-test: run our algorithm on another team's dataset.
%   M = run_challenge('path/to/other_team_data.mat')
%   M = run_challenge(file, struct('robust','huber'))
%   Validates the file first; scores it only if it contains ground truth.
setup_paths();
if nargin < 2, opts = struct(); end
data = load_dataset(datafile);
fprintf('Validating %s\n', datafile);
ok = validate_data(data);
if ~ok
    warning('Dataset violates the agreed interface/validity conditions. Report it to the other team.');
end
est = run_algorithm(data, opts);
M = struct();
if isfield(data, 'gt')
    M = evaluate_estimate(data, est);
else
    fprintf('No ground truth in file: estimate computed but not scored.\n');
end
end
