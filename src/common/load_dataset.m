function data = load_dataset(file)
%LOAD_DATASET Load a dataset saved by a generator (fields: meta, imu, gps, gt).
%   The gt field may be absent in challenge data given to other teams.
data = load(file);
required = {'meta', 'imu', 'gps'};
for k = 1:numel(required)
    if ~isfield(data, required{k})
        error('load_dataset:missingField', 'Dataset %s has no field "%s".', file, required{k});
    end
end
end
