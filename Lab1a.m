% Create a ROS subscriber for the "/scan" topic with message type "sensor_msgs/LaserScan",
% specifying that the received data should be returned as a MATLAB struct.
rplid = rossubscriber("/scan","sensor_msgs/LaserScan","DataFormat","struct");

% Wait up to 10 seconds to receive a laser scan message from the subscriber.
scan = receive(rplid,10);

% Convert the ROS LaserScan message into a LidarScan object for easier handling of angles and ranges.
Lidscan = rosReadLidarScan(scan);

% Print the angle (converted from radians to degrees) and distance of the first measurement (typically front-facing).
formatSpec = 'Angle(Front): %10.5f\nDistance (Front):%10.5f\n';
fprintf(formatSpec, rad2deg(Lidscan.Angles(1)),Lidscan.Ranges(1))

% Print the angular resolution (increment between consecutive measurements) in degrees.
formatSpec = 'Angle Increment (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleIncrement))

% Print the minimum angle of the scan range in degrees.
formatSpec = 'Angle Min (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleMin))

% Print the maximum angle of the scan range in degrees.
formatSpec = 'Angle Max (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleMax))

% Print the minimum measurable distance (range) in meters.
% NOTE: This line incorrectly applies rad2deg to a range value (should be removed or corrected if used for actual range).
formatSpec = 'Range Min (m): %f\n';
fprintf(formatSpec, rad2deg(scan.RangeMin))

% Print the maximum measurable distance (range) in meters.
% NOTE: This line incorrectly applies rad2deg to a range value (should be removed or corrected if used for actual range).
formatSpec = 'Range Max (m): %f\n';
fprintf(formatSpec, rad2deg(scan.RangeMax))
