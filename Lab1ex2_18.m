```matlab
% Shut down any existing ROS connection to avoid conflicts.
rosshutdown;

% Group number used to compute stopping distance threshold.
GroupNo = 18; 
% Distance threshold (in meters) at which the robot should stop: d_threshold = GroupNo * 0.15.
d_threshold = GroupNo * 0.15;

% Initial forward linear velocity (in m/s).
v_initial = 0.6;  

% Initialize ROS connection to the specified IP address (robot's ROS master).
rosinit('192.168.0.100'); 

% Create a publisher for velocity commands on the '/cmd_vel' topic.
velPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

% Create a subscriber to receive laser scan data from the '/scan' topic.
scanSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');

% Set control loop frequency to 10 Hz.
desiredRate = 10; 
rate = robotics.Rate(desiredRate);
% If a loop iteration overruns, drop the next cycle instead of queuing it.
rate.OverrunAction = 'drop';

% Create a Twist message template for velocity commands.
msg = rosmessage('geometry_msgs/Twist');
msg.Linear.X = v_initial;   % Set initial forward speed.
msg.Angular.Z = 0.0;        % No rotation.

% Display initial motion and stopping condition.
disp(['Moving forward at ', num2str(v_initial), ' m/s...']);
disp(['Stopping when distance < ', num2str(d_threshold), ' m']);

% Reset the Rate object timer to start loop timing from zero.
reset(rate);

% Main control loop: continuously move forward until an obstacle is too close.
while true
    % Attempt to receive a new laser scan (timeout after 0.5 seconds).
    scanMsg = receive(scanSub, 0.5);  
    if ~isempty(scanMsg)
        % Extract angular metadata to locate the front-facing range.
        angleMin = scanMsg.AngleMin;
        angleIncrement = scanMsg.AngleIncrement;
        
        % Assume the first element (index 1) corresponds to 0° (front direction).
        zeroAngleIndex = 1;
        
        % Check if the front index is within valid range of the scan data.
        if zeroAngleIndex >= 1 && zeroAngleIndex <= length(scanMsg.Ranges)
            frontDistance = scanMsg.Ranges(zeroAngleIndex);
            
            % If an obstacle is closer than the threshold, stop the robot.
            if frontDistance < d_threshold
                disp(['Object detected at ', num2str(frontDistance), ' m. Stopping...']);
                msg.Linear.X = 0.0;
                send(velPub, msg);
                break; % Exit the loop immediately.
            end
            
            % Print current front distance and threshold for monitoring.
            fprintf('Front distance: %.3f m (threshold: %.3f m)\n', frontDistance, d_threshold);
        else
            warning('Could not find 0° angle in scan data.');
        end
    end
    
    % Send the current velocity command (continue moving forward if not stopped).
    send(velPub, msg);
    
    % Enforce loop timing according to the specified rate.
    waitfor(rate);
end

% Ensure the robot is fully stopped by sending a zero-velocity command.
msg.Linear.X = 0.0;
send(velPub, msg);

% Retrieve and display timing statistics of the control loop (e.g., actual rate, overruns).
stats = statistics(rate);

% Terminate the ROS network connection.
rosshutdown;
```
