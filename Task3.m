clc;
% ROS setup
rosshutdown;
rosinit('http://192.168.0.100:11311');

lidarSub = rossubscriber('/scan');
cmdPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

% PD gains (within suggested range)
kP = 2.2;
kD = 0.001;

% Lookahead parameter
s_bar = 0.5;  % lookahead distance [0.3, 0.5]
L = 0.25;
prev_error = 0;
% prev_error_ass = 0;
% prev_error_ass_1 = 0;
target_dist = 0.5; 
rate = robotics.Rate(10);

while true
    scanMsg = receive(lidarSub, 1);
    if isempty(scanMsg)
        continue;
    end

    angles = scanMsg.AngleMin + (0:length(scanMsg.Ranges)-1)' * scanMsg.AngleIncrement;
    ranges = scanMsg.Ranges;

    front_mask = (angles >= deg2rad(175)) | (angles <= deg2rad(-175));
    front_ranges = ranges(front_mask);
    valid_front = front_ranges(~isnan(front_ranges) & ~isinf(front_ranges) & ...
                               (front_ranges > 0.01) & (front_ranges < 8.0));
    if ~isempty(valid_front)
        min_front_dist = min(valid_front);
    else
        min_front_dist = Inf;
    end

    if min_front_dist < 0.34
        fprintf('Obstacle detected at %.2f m. Stopping...\n', min_front_dist);
        stop_msg = rosmessage('geometry_msgs/Twist');
        stop_msg.Linear.X = 0.0;
        stop_msg.Angular.Z = 0.0;
        send(cmdPub, stop_msg);
        break;
    end

    % Find distance 'b' at 90 degrees (pi/2)
    [~, idx_b] = min(abs(angles - pi/2));        
    b = ranges(idx_b);
    % Find distance 'a' at 155 degrees
    [~, idx_a] = min(abs(angles - deg2rad(150)));  
    a = ranges(idx_a);
    % Find distance c at angle_assist degrees
    angle_assist = pi/2 + deg2rad(5); 
    angle_assist_1 = pi/2 + deg2rad(75); 
    [~, idx_angle_assist] = min(abs(angles - angle_assist));
    c = ranges(idx_angle_assist);
    [~, idx_angle_assist_1] = min(abs(angles - angle_assist_1));
    c_1 = ranges(idx_angle_assist_1);


    if isnan(a) || isnan(b) || isnan(c) || isnan(c_1)
        current_dist = target_dist;
        lookahead_dist = target_dist;
        assist_dist = target_dist;
        assist_dist_1 = target_dist;
    else
        theta0 = deg2rad(60);    
        theta_ass = deg2rad(5);
        theta_ass_1 = deg2rad(75);
        % Avoid division by zero
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
        d = b * cos(alpha);                     % current distance(forecast)
        d_ass = b * cos(alpha_ass);             % small degree distance
        d_ass_1 = b * cos(alpha_ass_1);            
        lookahead_dist = d + s_bar * sin(alpha); % lookahead distance
        lookahead_dist_ass = d_ass + s_bar * sin(alpha_ass); % assistant lookahead distance
        lookahead_dist_ass_1 = d_ass_1 + s_bar * sin(alpha_ass_1); % assistant lookahead distance_1
    end

    if lookahead_dist ~= inf 
        error = target_dist - lookahead_dist;   

        if lookahead_dist ~= inf && lookahead_dist_ass ~= inf && lookahead_dist_ass_1 ~= inf && lookahead_dist ~= -inf && lookahead_dist_ass ~= -inf && lookahead_dist_ass_1 ~= -inf 
            error_base = target_dist - lookahead_dist; 
            error_end = target_dist - lookahead_dist_ass;
            error_noforcast = target_dist - d_ass;
            % 120deg - 165deg
            delta_front = lookahead_dist - lookahead_dist_ass_1;  
            % 120deg - 92deg
            delta_right = lookahead_dist - lookahead_dist_ass;
    
            corner_threshold = 0.171;  
            dicision_threshold = 0.1;
            % Stopboard_threshold = 0.01;
            if abs(delta_front) > dicision_threshold
                if delta_right < -corner_threshold
                    % 
                    fprintf('Detected RIGHT WALL INNER CORNER (left bend). Turning LEFT.\n');
                    if error_base > 0   
                        fprintf('Turning LEFT Now (left bend). Encourage Turning LEFT.\n');
                        K_L = 1.1;
                        error = error_base.*K_L;  
                    else  
                        fprintf('But NOT yet turned (left bend). Encourage Turning LEFT.\n');
                        error = error_base;
                    end
                elseif delta_right > corner_threshold
                    % 
                    fprintf('Detected RIGHT WALL OUTER CORNER (right bend). Turning RIGHT.\n');
                    if error_base < 0
                        fprintf('approching Turning RIGHT Now (right bend). Slowdown Turning RIGHT.\n');   
                        error = error_noforcast;
                    end
                end
    

            else
                % straight
                error = error_base;
            end
        end
    else
      error = prev_error; 
    end


    derivative = (error - prev_error) * 10;
    steering_angle_rad = kP * error + kD * derivative;
    steering_angle_deg = rad2deg(steering_angle_rad);
    steering_angle_deg = max(-22, min(22, steering_angle_deg));


    steering_angle_deg = max(-22, min(22, steering_angle_deg));

    % 
    if abs(steering_angle_deg) < 5 
        v = 0.4;
    elseif abs(steering_angle_deg) <= 10
        v = 0.25 + (0.4 - 0.25) * (10 - abs(steering_angle_deg)) / 5;
    else
        v = 0.25;
    end
   
    omega = v * tan(deg2rad(steering_angle_deg)) / L;
    cmd = rosmessage('geometry_msgs/Twist');
    cmd.Linear.X = v;
    cmd.Angular.Z = omega;
    send(cmdPub, cmd);

    fprintf('lookaheaddist: %.3f m |  lookaheaddist ass: %.3f m | lookaheaddist ass_1: %.3f m | dist: %.3f | angle %.3f | error %.3f \n', ...
    lookahead_dist, lookahead_dist_ass, lookahead_dist_ass_1, d, steering_angle_deg, error);
    prev_error = error;

    waitfor(rate);

end


% Final stop
final_stop = rosmessage('geometry_msgs/Twist');
final_stop.Linear.X = 0.0;
final_stop.Angular.Z = 0.0;
send(cmdPub, final_stop);
