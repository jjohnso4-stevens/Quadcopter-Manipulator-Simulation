function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis, obstacle_locations, start_points, end_points, num_middle_points, obstacle_clearance, wall_clearance] = quadcopter_package_select_trajectory(path_number,varargin)
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
        waypoints = [ ...
            1    1  1  9  9  9  9  9  9  9
            1    9  9  9  9  9  9  1  1  1
            0.15 9  9  1  1  9  9  9  9  0.15];
        max_speed = 0.3;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 2
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
        xApproach = [2 0.5];
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
