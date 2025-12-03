```matlab
% Clear the command window for clean output.
clc;

% Terminate any existing ROS connection and initialize a new one to the specified ROS master URI.
rosshutdown;
rosinit('http://192.168.0.100:11311');

% Set up ROS subscriber for LiDAR data and publisher for velocity commands.
lidarSub = rossubscriber('/scan');
cmdPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

% PD controller gains (within typical tuning range for wall-following).
kP = 2.2;
kD = 0.001;

% Lookahead distance parameter for predictive wall-following (within [0.3, 0.5] m).
s_bar = 0.5;
% Wheelbase (or reference length) for Ackermann-like kinematic conversion.
L = 0.25;
% Initialize previous error for derivative term.
prev_error = 0;
% Target lateral distance to the wall (in meters).
target_dist = 0.5;
% Set control loop frequency to 10 Hz.
rate = robotics.Rate(10);

% Main control loop: continuously process LiDAR and adjust motion.
while true
    % Wait up to 1 second to receive a new LiDAR scan.
    scanMsg = receive(lidarSub, 1);
    if isempty(scanMsg)
        continue;  % Skip iteration if no data received.
    end

    % Reconstruct full angle vector from scan metadata.
    angles = scanMsg.AngleMin + (0:length(scanMsg.Ranges)-1)' * scanMsg.AngleIncrement;
    ranges = scanMsg.Ranges;

    % Identify front-facing beams (±5° around 180°, i.e., backward-facing in some sensor frames;
    % note: this assumes a 0° at robot front and angles increasing counter-clockwise).
    front_mask = (angles >= deg2rad(175)) | (angles <= deg2rad(-175));
    front_ranges = ranges(front_mask);
    % Filter valid front ranges (remove NaN, Inf, and out-of-range values).
    valid_front = front_ranges(~isnan(front_ranges) & ~isinf(front_ranges) & ...
                               (front_ranges > 0.01) & (front_ranges < 8.0));
    if ~isempty(valid_front)
        min_front_dist = min(valid_front);
    else
        min_front_dist = Inf;
    end

    % Emergency stop if an obstacle is too close in front.
    if min_front_dist < 0.34
        fprintf('Obstacle detected at %.2f m. Stopping...\n', min_front_dist);
        stop_msg = rosmessage('geometry_msgs/Twist');
        stop_msg.Linear.X = 0.0;
        stop_msg.Angular.Z = 0.0;
        send(cmdPub, stop_msg);
        break;  % Exit loop immediately.
    end

    % Extract range measurements at specific angles for wall geometry estimation:
    % - 'b': distance at 90° (right perpendicular to robot, assuming 0° is front)
    [~, idx_b] = min(abs(angles - pi/2));        
    b = ranges(idx_b);
    % - 'a': distance at 150° (used with b to compute wall angle)
    [~, idx_a] = min(abs(angles - deg2rad(150)));  
    a = ranges(idx_a);
    % - 'c' and 'c_1': auxiliary ranges at 95° and 165° for corner detection
    angle_assist = pi/2 + deg2rad(5);        % 95°
    angle_assist_1 = pi/2 + deg2rad(75);     % 165°
    [~, idx_angle_assist] = min(abs(angles - angle_assist));
    c = ranges(idx_angle_assist);
    [~, idx_angle_assist_1] = min(abs(angles - angle_assist_1));
    c_1 = ranges(idx_angle_assist_1);

    % Validate measurements and compute geometric estimates.
    if isnan(a) || isnan(b) || isnan(c) || isnan(c_1)
        % Fallback to target distance if any measurement is invalid.
        current_dist = target_dist;
        lookahead_dist = target_dist;
        assist_dist = target_dist;
        assist_dist_1 = target_dist;
    else
        % Define geometric angles used in law-of-sines-based wall angle estimation.
        theta0 = deg2rad(60);      % angle between a and wall normal
        theta_ass = deg2rad(5);    % small offset for c
        theta_ass_1 = deg2rad(75); % larger offset for c_1

        % Compute wall incidence angle 'alpha' using atan2 for robustness.
        denom = a * sin(theta0);
        denom_ass = c * sin(theta_ass);
        denom_ass_1 = c_1 * sin(theta_ass_1);
        if abs(denom) < 1e-3 
            alpha = 0;
        else
            alpha = atan2(a * cos(theta0) - b, denom);
        end
        if abs(denom_ass) < 1e-3 
            alpha_ass = 0;
        else
            alpha_ass = atan2(c * cos(theta_ass) - b, denom_ass);
        end
        if abs(denom_ass_1) < 1e-3 
            alpha_ass_1 = 0;
        else
            alpha_ass_1 = atan2(c_1 * cos(theta_ass_1) - b, denom_ass_1);
        end

        % Estimate current wall distance and lookahead distance (used for control).
        d = b * cos(alpha);                     
        d_ass = b * cos(alpha_ass);             
        d_ass_1 = b * cos(alpha_ass_1);            
        lookahead_dist = d + s_bar * sin(alpha);       
        lookahead_dist_ass = d_ass + s_bar * sin(alpha_ass);     
        lookahead_dist_ass_1 = d_ass_1 + s_bar * sin(alpha_ass_1); 
    end

    % Compute control error based on lookahead distance and handle corner cases.
    if lookahead_dist ~= inf 
        error = target_dist - lookahead_dist;   

        % Only apply advanced logic if all lookahead estimates are valid.
        if lookahead_dist ~= inf && lookahead_dist_ass ~= inf && lookahead_dist_ass_1 ~= inf && ...
           lookahead_dist ~= -inf && lookahead_dist_ass ~= -inf && lookahead_dist_ass_1 ~= -inf 
            error_base = target_dist - lookahead_dist; 
            error_end = target_dist - lookahead_dist_ass;
            error_noforcast = target_dist - d_ass;

            % Compute spatial differences to detect corners.
            delta_front = lookahead_dist - lookahead_dist_ass_1;  % compare 90° vs 165°
            delta_right = lookahead_dist - lookahead_dist_ass;    % compare 90° vs 95°

            % Thresholds for corner detection and decision logic.
            corner_threshold = 0.171;  
            dicision_threshold = 0.1;

            % If significant change in wall distance ahead, classify corner type.
            if abs(delta_front) > dicision_threshold
                if delta_right < -corner_threshold
                    % Inner corner on right wall (robot should turn left).
                    fprintf('Detected RIGHT WALL INNER CORNER (left bend). Turning LEFT.\n');
                    if error_base > 0   
                        fprintf('Turning LEFT Now (left bend). Encourage Turning LEFT.\n');
                        K_L = 1.1;
                        error = error_base * K_L;  
                    else  
                        fprintf('But NOT yet turned (left bend). Encourage Turning LEFT.\n');
                        error = error_base;
                    end
                elseif delta_right > corner_threshold
                    % Outer corner on right wall (robot should turn right).
                    fprintf('Detected RIGHT WALL OUTER CORNER (right bend). Turning RIGHT.\n');
                    if error_base < 0
                        fprintf('approching Turning RIGHT Now (right bend). Slowdown Turning RIGHT.\n');   
                        error = error_noforcast;
                    end
                end
            else
                % Straight wall segment: use base error.
                error = error_base;
            end
        end
    else
        % If lookahead is invalid, hold previous error (fallback).
        error = prev_error; 
    end

    % Compute derivative term (rate = 10 Hz → dt = 0.1 s → derivative gain scaled by 10).
    derivative = (error - prev_error) * 10;
    % Compute raw steering command via PD control.
    steering_angle_rad = kP * error + kD * derivative;
    steering_angle_deg = rad2deg(steering_angle_rad);
    % Clamp steering angle to physical limits (±22°).
    steering_angle_deg = max(-22, min(22, steering_angle_deg));

    % Map steering angle to forward velocity (reduce speed on sharper turns).
    if abs(steering_angle_deg) < 5 
        v = 0.4;
    elseif abs(steering_angle_deg) <= 10
        v = 0.25 + (0.4 - 0.25) * (10 - abs(steering_angle_deg)) / 5;
    else
        v = 0.25;
    end

    % Convert steering angle to angular velocity using bicycle kinematics.
    omega = v * tan(deg2rad(steering_angle_deg)) / L;
    % Assemble and send velocity command.
    cmd = rosmessage('geometry_msgs/Twist');
    cmd.Linear.X = v;
    cmd.Angular.Z = omega;
    send(cmdPub, cmd);

    % Log key state variables for debugging and tuning.
    fprintf('lookaheaddist: %.3f m |  lookaheaddist ass: %.3f m | lookaheaddist ass_1: %.3f m | dist: %.3f | angle %.3f | error %.3f \n', ...
        lookahead_dist, lookahead_dist_ass, lookahead_dist_ass_1, d, steering_angle_deg, error);
    % Update previous error for next iteration.
    prev_error = error;

    % Enforce loop timing.
    waitfor(rate);
end

% Send final stop command to ensure robot halts after loop exit.
final_stop = rosmessage('geometry_msgs/Twist');
final_stop.Linear.X = 0.0;
final_stop.Angular.Z = 0.0;
send(cmdPub, final_stop);
```
