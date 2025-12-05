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
            1    1;
            1    1;
            0.15 5];
        end_points   = [
            9  9;
            9  9;
            5  0.15];
        obstacle_locations = [
    0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  3.5  4.5  5.5  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5  0.5  1.5  2.5  -10  -10  -10  6.5  7.5  8.5  9.5;
    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    5    -10  -10  -10  5    5    5    5    5    5    5    -10  -10  -10  5    5    5    5    5    5    5    -10  -10  -10    5    5    5    5;
    0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  1.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  2.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  3.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  4.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  5.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  6.5  7.5  7.5  7.5  -10  -10  -10  7.5  7.5  7.5  7.5  8.5  8.5  8.5  -10  -10  -10  8.5  8.5  8.5  8.5  9.5  9.5  9.5  -10  -10  -10  9.5  9.5  9.5  9.5
        ];

        % --- Path planning parameters ---
    step_size = 0.15;                     % forward progress per waypoint
    min_distance_to_obstacles = 1.5;       % clearance from obstacles (meters)
    box_limits = [0.5 9.5];              % bounding box limits
    dist_to_goal = norm(end_points(:,1) - start_points(:,2));
max_iters = ceil(2 * dist_to_goal / step_size);                      % maximum intermediate waypoints

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
        % ----- Replace random nudge with directed nudges -----
        for j = 1:size(obstacle_locations,2)
            obs_vec = next_pt - obstacle_locations(:,j);
            dist = norm(obs_vec);

            if dist < 1.0  % too close
                nudge = 0.1 * obs_vec / dist;  % push away directly
            elseif dist < 1.5  % moderately close
                % perpendicular nudge
                rand_vec = rand(3,1) - 0.5;
                nudge_dir = rand_vec - (rand_vec'*obs_vec)*obs_vec/(dist^2);
                nudge = 0.05 * nudge_dir / norm(nudge_dir);
            else
                nudge = [0;0;0];  % safe distance, no nudge
            end

            next_pt = next_pt + nudge;
        end
    end

    safety_try = safety_try + 1;
end

        % ---------- Keep inside box ----------
        next_pt = min(max(next_pt, box_limits(1)), box_limits(2));

        % Append waypoint
        waypoints = [waypoints next_pt];
        current_pt = next_pt;
    end

% --- Smooth the intermediate path (columns 3 through end-2) ---
if size(waypoints,2) > 4   % only smooth if enough points
    smooth_section = waypoints(:,3:end); % exclude start ground & hover

    % Apply a simple moving average filter (window size = 3)
    window = 3;
    for dim = 1:3
        smooth_section(dim,:) = movmean(smooth_section(dim,:), window);
    end

    % Reinsert smoothed points back into waypoint list
    waypoints(:,3:end) = smooth_section;
end

    % --- Append final hover and landing point ---
    waypoints = [waypoints end_points(:,1) end_points(:,2)];
        
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [3 0.1];
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
    5 7 3 4 6 9 5 5 2 2 6 3 3 3 7 7 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10;
    5 7 3 8 4 7 5 5 2 6 6 3 2 5 6 8 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10;
    5 5 5 7 4 5 7 3 7 5 6 3 5 4 4 7 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10 -10
        ];

        % --- Path planning parameters ---
    step_size = 0.15;                     % forward progress per waypoint
    min_distance_to_obstacles = 1.5;       % clearance from obstacles (meters)
    box_limits = [0.5 9.5];              % bounding box limits
    dist_to_goal = norm(end_points(:,1) - start_points(:,2));
max_iters = ceil(2 * dist_to_goal / step_size);                      % maximum intermediate waypoints

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
        % ----- Replace random nudge with directed nudges -----
        for j = 1:size(obstacle_locations,2)
            obs_vec = next_pt - obstacle_locations(:,j);
            dist = norm(obs_vec);

            if dist < 1.0  % too close
                nudge = 0.1 * obs_vec / dist;  % push away directly
            elseif dist < 1.5  % moderately close
                % perpendicular nudge
                rand_vec = rand(3,1) - 0.5;
                nudge_dir = rand_vec - (rand_vec'*obs_vec)*obs_vec/(dist^2);
                nudge = 0.05 * nudge_dir / norm(nudge_dir);
            else
                nudge = [0;0;0];  % safe distance, no nudge
            end

            next_pt = next_pt + nudge;
        end
    end

    safety_try = safety_try + 1;
end

        % ---------- Keep inside box ----------
        next_pt = min(max(next_pt, box_limits(1)), box_limits(2));

        % Append waypoint
        waypoints = [waypoints next_pt];
        current_pt = next_pt;
    end

% --- Smooth the intermediate path (columns 3 through end-2) ---
if size(waypoints,2) > 4   % only smooth if enough points
    smooth_section = waypoints(:,3:end); % exclude start ground & hover

    % Apply a simple moving average filter (window size = 3)
    window = 3;
    for dim = 1:3
        smooth_section(dim,:) = movmean(smooth_section(dim,:), window);
    end

    % Reinsert smoothed points back into waypoint list
    waypoints(:,3:end) = smooth_section;
end

    % --- Append final hover and landing point ---
    waypoints = [waypoints end_points(:,1) end_points(:,2)];
        
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [3 0.1];
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
