% Create a ROS publisher for the '/cmd_vel' topic with message type 'geometry_msgs/Twist'.
[pub,msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

% Set linear velocity (forward) to 0.2 m/s and angular velocity (yaw rate) to -0.5 rad/s (clockwise turn).
msg.Linear.X = 0.2;
msg.Angular.Z = -0.5;

% Define the desired publishing rate (10 Hz) and create a Rate object to control loop timing.
desiredRate = 10;
rate = robotics.Rate(desiredRate);

% Configure the overrun action: if a loop iteration takes longer than the period,
% skip the next iteration rather than queuing it ('drop' mode).
rate.OverrunAction = 'drop';

% Reset the internal timer of the Rate object to start timing from zero.
reset(rate)

% Publish the velocity command repeatedly for 3 seconds at the specified rate.
while rate.TotalElapsedTime < 3
    send(pub,msg)        % Send the current velocity command
    waitfor(rate);       % Wait until the next scheduled time step
end

% Stop the robot by setting both linear and angular velocities to zero.
msg.Linear.X = 0.0;
msg.Angular.Z = 0.0;
send(pub,msg)

% Display timing statistics (e.g., actual rate, number of overruns, total elapsed time).
statistics(rate)
