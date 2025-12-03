```matlab
% Clear the MATLAB command window.
clc;

% Shut down any existing ROS network connection (if active).
rosshutdown;

% Define group number to determine target scanning angles.
GroupNo = 18;

% Compute target angles based on group number: symmetric ±(GroupNo * 5) degrees.
targetAngleDeg = GroupNo * 5;
angle1 = targetAngleDeg;        % e.g., +90° for Group 18
angle2 = -targetAngleDeg;       % e.g., -90° for Group 18
angle3 = 110;                   % Additional fixed angle (currently commented out in output)

% Initialize ROS master connection to the specified IP address.
rosinit('192.168.0.100');  

% Create a subscriber to the '/scan' topic for receiving laser scan data.
scanSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');

% Wait up to 5 seconds to receive a single laser scan message.
scanMsg = receive(scanSub, 5);  

% Throw an error if no scan data is received (indicating connection or topic issues).
if isempty(scanMsg)
    error('No laser scan data received. Check ROS connection and topic.');
end

% Extract angular metadata from the scan message (all in radians).
angleMin = scanMsg.AngleMin;           % Start angle of the scan
angleMax = scanMsg.AngleMax;           % End angle of the scan
angleIncrement = scanMsg.AngleIncrement; % Angular distance between measurements

% Convert target angles from degrees to radians.
angle1_rad = deg2rad(angle1);
angle2_rad = deg2rad(angle2);
angle3_rad = deg2rad(angle3);
a_rad = deg2rad(-180);  % Unused variable (possibly for future use)

% Calculate array indices corresponding to the desired angles.
% LaserScan ranges are indexed starting from angleMin with step = angleIncrement.
idx1 = round((angle1_rad - angleMin) / angleIncrement) + 1;
idx2 = round((angle2_rad - angleMin) / angleIncrement) + 1;
% idx3 = round((angle3_rad - angleMin) / angleIncrement) + 1;  % Currently commented out

% Retrieve distance at angle1 if within valid scan range; otherwise return NaN and warn.
if idx1 < 1 || idx1 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle1)
    dist1 = NaN;
else
    dist1 = scanMsg.Ranges(idx1);
end

% Retrieve distance at angle2 if within valid scan range; otherwise return NaN and warn.
if idx2 < 1 || idx2 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle2)
    dist2 = NaN;
else
    dist2 = scanMsg.Ranges(idx2);
end

% The following block references an undefined variable 'idx3' (since it's commented out above),
% which will cause a runtime error. It is kept as-is per instruction not to modify code.
if idx3 < 1 || idx3 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle1)
    dist3 = NaN;
else
    dist3 = scanMsg.Ranges(idx3);
end

% Print the measured distances at the two primary angles.
fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle1, dist1);
fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle2, dist2);
% fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle3, dist3);  % Output for angle3 is disabled

% Terminate the ROS network connection.
rosshutdown;
```
