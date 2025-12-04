function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis, obstacle_locations, start_points, end_points] = quadcopter_package_select_trajectory(path_number,varargin)
%quadcopter_select_trajectory Obtain parameters for selected quadcopter trajectory
%   [waypoints, timespot_spl, spline_data, spline_yaw] = quadcopter_select_trajectory(path_number)
%   This function returns the essential parameters that define the
%   quadcopter's trajectory. The function returns
%
%       waypoints       Key x-y-z locations the quadcopter will pass through
%       timespot_spl    Times the quadcopter will pass through points along
%                       the spline that defines its path
%       spline_data     Points used for interpolating the spline that
%                       defines the path of the quadcopter
%       spline_yaw      Yaw angle at the spline_data points

% Copyright 2021-2024 The MathWorks, Inc.

if(nargin == 2)
    roundtrip = varargin{1};
else
    roundtrip = false;
end

switch (path_number)
    case 1
        start_points = [
            1    1  1;
            1    1  1;
            0.15 2  2];
        end_points   = [
            9  9  9;
            9  9  9;
            2  2  0.15];
        num_middle_points = 1000;
        obstacle_clearance = 1.0;
        wall_clearance     = 0.5;
        obstacle_locations = [
    0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5;
    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    -10  -10  -10  5    5    5    5    5    5    5    -10  -10  -10  5    5    5    5    5    5    5    -10  -10  -10    5    5    5    5;
    0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  7.5  7.5  7.5  -10  -10  -10  7.5  7.5  7.5  7.5  8.5  8.5  8.5  -10  -10  -10  8.5  8.5  8.5  8.5  9.5  9.5  9.5  -10  -10  -10  9.5  9.5  9.5  9.5
];

% --- Middle waypoint generation settings ---
middle_start = start_points(:,3);  % Start of the middle section
middle_end   = end_points(:,1);    % End of the middle section

% Force parameters
step_size = 0.1;      % Desired travel distance per step
k_att = 1.5;          % Attractive gain
k_rep = 40;           % Repulsion gain
inflation = 1.2;      % Distance at which repulsion activates
tol = 0.05;           % Distance tolerance to goal
max_iterations = 20000;
max_middle_waypoints = 1000; % cap the number of middle waypoints

% Wall boundaries
xmin = wall_clearance;
xmax = 10 - wall_clearance;
ymin = wall_clearance;
ymax = 10 - wall_clearance;
zmin = wall_clearance;
zmax = 10 - wall_clearance;

% Initialize middle waypoints
middle_waypoints = middle_start;
current_pt = middle_start;

for i = 1:max_iterations
    % Attractive force to goal
    F_att = k_att * (middle_end - current_pt);

    % Repulsive force from obstacles
    F_rep = zeros(3,1);
    for k = 1:size(obstacle_locations,2)
        obs = obstacle_locations(:,k);
        if any(obs == -10)
            continue; % Skip invalid obstacle points
        end
        diff = current_pt - obs;
        dist = norm(diff);
        if dist < inflation && dist > 1e-3
            F_rep = F_rep + k_rep * (1/dist - 1/inflation) * (diff / dist^3);
        end
    end

    % Repulsive force from walls
    F_wall = zeros(3,1);
    % X walls
    if current_pt(1) - xmin < inflation
        F_wall(1) = F_wall(1) + k_rep * (1/(current_pt(1)-xmin) - 1/inflation) / (current_pt(1)-xmin)^2;
    elseif xmax - current_pt(1) < inflation
        F_wall(1) = F_wall(1) - k_rep * (1/(xmax-current_pt(1)) - 1/inflation) / (xmax-current_pt(1))^2;
    end
    % Y walls
    if current_pt(2) - ymin < inflation
        F_wall(2) = F_wall(2) + k_rep * (1/(current_pt(2)-ymin) - 1/inflation) / (current_pt(2)-ymin)^2;
    elseif ymax - current_pt(2) < inflation
        F_wall(2) = F_wall(2) - k_rep * (1/(ymax-current_pt(2)) - 1/inflation) / (ymax-current_pt(2))^2;
    end
    % Z walls
    if current_pt(3) - zmin < inflation
        F_wall(3) = F_wall(3) + k_rep * (1/(current_pt(3)-zmin) - 1/inflation) / (current_pt(3)-zmin)^2;
    elseif zmax - current_pt(3) < inflation
        F_wall(3) = F_wall(3) - k_rep * (1/(zmax-current_pt(3)) - 1/inflation) / (zmax-current_pt(3))^2;
    end

    % Combine all forces
    F_raw = F_att + F_rep + F_wall;

    % Detect stagnation / local minimum
    check_len = min(10, size(middle_waypoints,2)-1);
    if check_len > 0 && norm(current_pt - middle_waypoints(:,end-check_len)) < 0.05
        % Push toward goal + random nudge
        F_raw = F_raw + 0.8*(middle_end - current_pt)/norm(middle_end - current_pt) + 0.5*(rand(3,1)-0.5);
    end

    % Tangential escape along obstacles (optional)
    if norm(F_rep) > 1e-3
        w = 0.5;
        R = [0 -1 0; 1 0 0; 0 0 0];  % rotation matrix in XY plane
        F_raw = F_raw + w * (R * F_rep);
    end

    % Normalize and take step
    if norm(F_raw) > 1e-6
        F = F_raw / norm(F_raw);
        % Damp step near walls
        wall_dist = min([current_pt(1)-xmin, xmax-current_pt(1), ...
                         current_pt(2)-ymin, ymax-current_pt(2), ...
                         current_pt(3)-zmin, zmax-current_pt(3)]);
        step_actual = step_size * min(1, wall_dist/inflation);
        next_pt = current_pt + step_actual * F;
    else
        next_pt = current_pt + step_size * (rand(3,1)-0.5); % nudge if stuck
    end

    % Clamp to box boundaries
    next_pt(1) = min(max(next_pt(1), xmin), xmax);
    next_pt(2) = min(max(next_pt(2), ymin), ymax);
    next_pt(3) = min(max(next_pt(3), zmin), zmax);

    % Store new point
    middle_waypoints(:,end+1) = next_pt;
    current_pt = next_pt;

    % Stop if goal reached or max waypoints reached
    if norm(current_pt - middle_end) < tol || size(middle_waypoints,2) >= max_middle_waypoints
        break
    end
end

% -----------------------------------------------------------
% Final: build complete waypoint matrix
% -----------------------------------------------------------
waypoints = [start_points, middle_waypoints, end_points];

disp('--- Waypoints ---');
disp(waypoints');
        
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;



     
    case 2
        start_points = [
            1    1;
            1    1;
            0.15 5];
        end_points   = [
            9  9;
            9  9;
            5  0.15];
        obstacle_locations = [
    5 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10;
    5 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10;
    5 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10
];

        % --- Path planning parameters ---
    step_size = 0.5;                     % forward progress per waypoint
    min_distance_to_obstacles = 1;       % clearance from obstacles (meters)
    box_limits = [0.5 9.5];              % bounding box limits
    max_iters = 80;                      % maximum intermediate waypoints

    % --- Initialize waypoint list ---
    % Include BOTH start ground and start hover
    waypoints = start_points;       % 1st = ground, 2nd = hover
    current_pt = start_points(:,2); % but pathfinding starts at hover

    goal_pt = end_points(:,1);      % end hover point

    % --- Generate intermediate waypoints ---
    for i = 1:max_iters

        % Direction toward goal (unit vector)
        dir_vec = goal_pt - current_pt;
        dist_to_goal = norm(dir_vec);
        if dist_to_goal < step_size
            break;  % close enough to final hover
        end
        dir_unit = dir_vec / dist_to_goal;

        % Proposed next point
        next_pt = current_pt + step_size * dir_unit;

        % ---------- Obstacle Avoidance ----------
        too_close = true;
        safety_try = 0;
        while too_close && safety_try < 20

            % Distance to all obstacles
            distances = sqrt(sum((obstacle_locations - next_pt).^2, 1));

            if all(distances >= min_distance_to_obstacles)
                too_close = false;  % safe
            else
                % Sideways random nudge to try avoiding obstacle
                sideways = 0.3 * (2*rand(3,1) - 1);
                next_pt = next_pt + sideways;
            end

            safety_try = safety_try + 1;
        end

        % ---------- Keep inside box ----------
        next_pt = min(max(next_pt, box_limits(1)), box_limits(2));

        % Append waypoint
        waypoints = [waypoints next_pt];
        current_pt = next_pt;
    end

    % --- Append final hover and landing point ---
    waypoints = [waypoints end_points(:,1) end_points(:,2)];
        
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;

    case 3
        start_points = [];
        end_points   = [];
        num_middle_points = 1000;
        obstacle_clearance = 1.0;
        wall_clearance     = 0.5;
        obstacle_locations = [];
        % Note: This trajectory defines the waypoints, spline data, and yaw
        % data explicitly and does not use the function to calculate the
        % target speed and yaw angle based on the path.
        waypoints = [];
        spline_data = waypoints';
        timespot_spl = [0:4:11*4 11*4+6 11*4+6+6]';
        spline_yaw   = [0 0 0 pi/4 pi/4 pi/4 0 0 0 -pi/4 -pi/4 -pi/4 -pi/4 -pi/4];

    case 4
        start_points = [
            1    1  1;
            1    1  1;
            0.15 2  2];
        end_points   = [];
        num_middle_points = 1000;
        obstacle_clearance = 1.0;
        wall_clearance     = 0.5;
        obstacle_locations = [];
        waypoints = [];
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 5
        start_points = [
            1    1  1;
            1    1  1;
            0.15 2  2];
        end_points   = [];
        num_middle_points = 1000;
        obstacle_clearance = 1.0;
        wall_clearance     = 0.5;
        obstacle_locations = [];
        waypoints = [];
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;
        
    case 6
        start_points = [
            1    1  1;
            1    1  1;
            0.15 2  2];
        end_points   = [
            9  9  9;
            9  9  9;
            2  2  0.15];
        num_middle_points = 1000;
        obstacle_clearance = 1.0;
        wall_clearance     = 0.5;
        obstacle_locations = [...
            2 8 5 8 2 6
            2 8 5 3 8 2
            4 6 5 5 8 2];
        waypoints = [ ...
            1    1  1  9  9  9  9  9  9  9
            1    9  9  9  9  9  9  1  1  1
            0.15 9  9  1  1  9  9  9  9  0.15];
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;
end

% Only call the function to calculate target speed and yaw angle if needed
% Paths that define the spline data and yaw angles explictly should not
% define parameter xApproach
if(exist("xApproach","var"))
    if(roundtrip)
        [timespot_spl_re, spline_data_re, spline_yaw_re, ~] = ...
            quadcopter_waypoints_to_trajectory(...
            fliplr(waypoints),max_speed,min_speed,xApproach,vApproach);

        [timespot_spl_to, spline_data_to, spline_yaw_to, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
        
        pause_at_target = 5; % sec
        timespot_spl = [timespot_spl_to; timespot_spl_re+timespot_spl_to(end)+pause_at_target];

        spline_data = [spline_data_to;spline_data_re];
        spline_yaw = [spline_yaw_to spline_yaw_re];
        spline_yaw = unwrap(spline_yaw,1.5*pi);
    else
        [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
    end
else
    % Obtain data to visualize path between waypoints
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    if(roundtrip)
        spline_data  = [spline_data; flipud(spline_data)];
        %timespot_spl
        timespot_spl = [timespot_spl; timespot_spl(end)+5; ...
            timespot_spl(end)+5+cumsum(flipud(diff(timespot_spl)))];
        spline_yaw = unwrap([spline_yaw flipud(spline_yaw)+pi],1.5*pi);
    end
end
