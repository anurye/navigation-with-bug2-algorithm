function [forwBackVel, leftRightVel, rotVel, finish] = solution5(...
    pts, contacts, position, orientation, varargin)

% If not provided, take desired distance from the wall
desired_distance = 0.5;

% check varargin
if length(varargin) == 1
    goal_position = varargin{1};
elseif length(varargin) == 2
    goal_position = varargin{1};
    desired_distance = varargin{2};
else
    error("Wrong number of additional arguments provided %d.\n", length(varargin))
end

% Current position and orientation
current_pos = position(1:2);
phi = orientation(3);

% Initialize variables
u_max = 5;
w_max = 5;
Kp_linear = 2;
Kp_angular = 10;
Kp_goal = 20;

% Error tolerance
tolerance = 0.1;

% Direction of tangential movement
option = 1; % 1 is for right and -1 is for left

% Orientation wrt a vector perpendicular to the robot face for wall
% following
goal_theta = 0;

% Initialize the robot control variables (returned by this function)
finish = false;     
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;

% Finite State Machine initialization
persistent state l oc d wall;
% Where: l is line passing through initial and goal positions
%        oc is coordinate where we encounter obstacle (Wall)
%        d is the dstance from the obstacle to goal
%        wall is a boolean which indicates whether the robot encounters
%        a wall or not
if isempty(state)
    state = 'init';
end

% Initial step
if strcmp(state, 'init')
    % Change state to goal
    state = 'goal';
    disp('Robot starts moving...')

    % Find the line l connecting the initial and goal positions
    % Slope (m)
    l(1) = (goal_position(2) - current_pos(2)) / (goal_position(1) - current_pos(1));
    % Y-intercept (b)
    l(2) = goal_position(2) - l(1) * goal_position(1);

    % Initialize the hit point
    oc = current_pos;
    d = 0;
end

% LIDAR reading processing
% Using polar coordinate as it is more suited for this problem than
% cartesian
[theta, distances] = cart2pol(pts(1, :), pts(2, :));
% There is a 90 degree rotation between the robot coordinate frame and
% the front of the robot we need to take that into consideration
theta = theta + pi/2;

% Lets seclect contacts which satiesfy a minimum Prominence of 0.2
desired_contacts = islocalmin([5 distances 5], 'MinProminence', 0.2);

% Adjust the lenght by romoving the first and last added distances
% [5..5]
desired_contacts = and(desired_contacts(2:end - 1), contacts);

% Compute distance from line l
ql(1) = (current_pos(1) + l(1) * (current_pos(2) - l(2))) / (l(1)^2 + 1);
ql(2) = ql(1) * l(1) + l(2);
diff_ql = ql - current_pos;
dist_line = norm(diff_ql);

% Compute distance and direction to goal
diff_goal = goal_position - current_pos;
dist_goal = norm(diff_goal);
goal_dir = angdiff(atan2(diff_goal(2), diff_goal(1)), phi - pi/2);

% Compute conditions for changing state
% Check if goal is reached
if dist_goal < tolerance
    % clean exit
    finish = true;
    return
end

% Other states
if strcmp(state, 'goal')
    if any(distances(desired_contacts) <= desired_distance)
        % There is a wall, change state to wall
        state = 'wall';
        oc = current_pos;
        wall = true;
        d = dist_goal;
    end

elseif strcmp(state, 'wall')
    % Check if we are crossing the line l
    if (dist_line < tolerance)
        % Check the LIDAR sensor in the direction of the goal
        [~, idx] = min(abs(angdiff(theta, goal_dir)));

        if (distances(idx) > 2 * desired_distance) && (dist_goal < d)
            % No obstacle in the direction of the goal, change state to
            % goal
            state = 'goal';

        elseif norm(oc - current_pos) < tolerance
            % We are back to the starting point whre d is measured
            if ~wall
                error("Robot is trapped, can't reach the goal")
            end
        else
            wall = false;
        end
    end
else
    error("Unknown state %s.\n", state)
end

% Compute movement direction and error for each states
if strcmp(state, 'goal')
    % Compute the direction to the closest point on the goal line
    radial = angdiff(atan2(diff_ql(2), diff_ql(1)), phi - pi/2);

    % Find the direction on the line that leads to the goal
    option = sign(angdiff(goal_dir, radial));

    % Compute the error for proportional control
    diff_dist = dist_line;

    % Make the robot face towards the goal
    goal_theta = -option * pi/2;

elseif strcmp(state, 'wall')
    % Use wall following algorithm
    % Find the indices of those local minimum distances
    local_min_idxs = find(desired_contacts);

    % Sort the distances and find respective idxs
    [~, idxs] = sort(distances(desired_contacts));

    % Get the index of the minimum distance
    min_idx = local_min_idxs(idxs(1));

    % Radial direction (disrection along the minimum distance)
    radial = theta(min_idx);

    % Error in desired distance (radius error)
    diff_dist = distances(min_idx) - desired_distance;
else
    error("Unknown state %s.\n", state)
end

% Tangential direction (perpendicular to radial direction)
tangential = radial + option * pi / 2;

% Weight of radial movement depends on the error from the wall
Pl = Kp_linear*diff_dist;

% Limit Pl
Pl = max(min(Pl, 1), -1);

% The resultant direction of movement
direction = Pl * radial + (1 - Pl) * tangential;

% Magnetude of velocity (limitted)
vel = max(min(Kp_goal*dist_goal, u_max), -u_max);

% Velocity with respect to local frame
u_local = local_velocity(vel, direction);

% Velocity commands
leftRightVel = u_local(1);
forwBackVel = u_local(2);

% Rotational velocity calculation
diff_theta = angdiff(radial, goal_theta);

% Proportional regulatior output
Pr = Kp_angular*diff_theta;

% Rotational velocity command
rotVel = max(min(Pr, w_max), -w_max);

end

function u_local = local_velocity(vel, direction)
% Resolves the velocity into leftRight and forwardBackward
u_local = vel * [sin(direction) -cos(direction)];
end
