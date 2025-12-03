clc;
rosshutdown;
GroupNo = 18;

targetAngleDeg = GroupNo * 5;
angle1 = targetAngleDeg;    
angle2 = -targetAngleDeg;   
angle3 = 110;

rosinit('192.168.0.100');  


scanSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');


scanMsg = receive(scanSub, 5);  

if isempty(scanMsg)
    error('No laser scan data received. Check ROS connection and topic.');
end


angleMin = scanMsg.AngleMin;           
angleMax = scanMsg.AngleMax;         
angleIncrement = scanMsg.AngleIncrement; 


angle1_rad = deg2rad(angle1);
angle2_rad = deg2rad(angle2);
angle3_rad = deg2rad(angle3);
a_rad = deg2rad(-180);

idx1 = round((angle1_rad - angleMin) / angleIncrement) + 1;
idx2 = round((angle2_rad - angleMin) / angleIncrement) + 1;
% idx3 = round((angle3_rad - angleMin) / angleIncrement) + 1;

if idx1 < 1 || idx1 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle1)
    dist1 = NaN;
else
    dist1 = scanMsg.Ranges(idx1);
end

if idx2 < 1 || idx2 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle2)
    dist2 = NaN;
else
    dist2 = scanMsg.Ranges(idx2);
end

if idx3 < 1 || idx3 > length(scanMsg.Ranges)
    warning('Angle %d° is out of scan range.', angle1)
    dist3 = NaN;
else
    dist3 = scanMsg.Ranges(idx3);
end

fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle1, dist1);
fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle2, dist2);
% fprintf('Group %d: Distance at %+d° = %.3f m\n', GroupNo, angle3, dist3);


rosshutdown;