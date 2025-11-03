%% ME 635 Course Project
%% Quad-rotor Flying Manipulator Simulation
% James Johnson 10/22/2025
%Comment
clc; clear; close all

% 1) Open project if not already open
proj = matlab.project.currentProject;
if isempty(proj)
    % Find project by name relative to this script
    thisFile = mfilename('fullpath');
    thisDir = fileparts(thisFile);
    % Walk up until we find the project file
    prjFile = dir(fullfile(thisDir, '**', 'Quadcopter_Drone.prj'));
    if isempty(prjFile)
        error('Could not locate Quadcopter_Drone.prj. Clone the repo and open it first.');
    end
    openProject(fullfile(prjFile(1).folder, prjFile(1).name));
    proj = matlab.project.currentProject;
end

% 2) Work relative to project root
rootDir  = proj.RootFolder;
dataDir  = fullfile(rootDir, 'data');
modelDir = fullfile(rootDir, 'models');
scriptDir= fullfile(rootDir, 'scripts');

% 3) Create folders if missing
for d = [string(dataDir), string(modelDir), string(scriptDir)]
    if ~exist(d, 'dir'), mkdir(d); end
end

% 4) Save parameters
p.m_drone      = 2.0;      % kg
p.m_payload    = 1.0;      % kg
p.m_arm        = 0.5;      % kg
p.rotor_diam   = 0.254;    % 10 in → 0.254 m
p.max_thrust_N = 3*4.448;  % 3 lbf → N
p.worldL       = 5.0;      % meters
p.n_obstacles  = 6;
save(fullfile(dataDir,'params.mat'), 'p');
disp('Saved params.mat');

% 5) Waypoints (shared file)
waypoints = [0 0 0.8; 4.5 0.5 1.2; 4.5 4.5 1.2; 0.5 4.5 0.8; 0.5 0.5 0.8];
save(fullfile(dataDir,'waypoints.mat'), 'waypoints');

% 6) Create / open model
open_system(new_system('quadPlant','Model'));
set_param('quadPlant','SolverType','Variable-step','StopTime','20');

disp('Project scaffold ready. Open quadPlant and start dropping Simscape blocks.');
