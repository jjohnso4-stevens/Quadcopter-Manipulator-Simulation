% Startup script for project Quadcopter_Drone.prj
% Copyright 2018-2024 The MathWorks, Inc.

%% Code for building Simscape custom library at startup
% Change to folder with package directory
curr_proj = simulinkproject;
cd(curr_proj.RootFolder)

% Change to root folder
cd(curr_proj.RootFolder)

% If running in a parallel pool
% do not open model or demo script
open_start_content = 1;
if(~isempty(ver('parallel')))
    if(~isempty(getCurrentTask()))
        open_start_content = 0;
    end
end

if(open_start_content)
    % Parameters
    quadcopter_package_parameters;
    % Parameters
quadcopter_package_parameters;

% Define trajectory (try a few, fall back gracefully)
trajCandidates = [2 3 1];
picked = false;
for idx = trajCandidates
    try
        [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_package_select_trajectory(idx, true);
        picked = true;
        break
    catch ME
        warning("Trajectory preset %d failed: %s", idx, ME.message);
    end
end
if ~picked
    warning("All presets failed; using minimal default waypoints.");
    waypoints = [0 0 0.8; 2 0 0.9; 2 2 1.0; 0.5 2 0.9; 0.5 0.5 0.8];
    [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = deal([]);
end

% (Optional) Set Python environment if the project uses py
check_pyenv

% Open the demo safely (or comment out while developing)
try
    quadcopter_package_delivery   % Not for Workshop
catch ME
    warning(ME.identifier, 'Skipping auto-open demo: %s', ME.message);
end

    % Set Python environment (if needed)
    check_pyenv
    % Open Model
    quadcopter_package_delivery % Not for Workshop
    % Open Exercises
    %quadcopter_workshop_prefs
    %quadcopter_drone_exercises_app_run  % For Workshop
end
