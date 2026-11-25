function setup_paths()
%SETUP_PATHS Add the project folders to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'configs'));
addpath(genpath(fullfile(root, 'src')));
end
