rosshutdown;
GroupNo = 18; 
d_threshold = GroupNo * 0.15;

v_initial = 0.6;  


rosinit('192.168.0.100'); 


velPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');


scanSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');


desiredRate = 10; 
rate = robotics.Rate(desiredRate);
rate.OverrunAction = 'drop';


msg = rosmessage('geometry_msgs/Twist');
msg.Linear.X = v_initial;  
msg.Angular.Z = 0.0;       


disp(['Moving forward at ', num2str(v_initial), ' m/s...']);
disp(['Stopping when distance < ', num2str(d_threshold), ' m']);

reset(rate);

while true

    scanMsg = receive(scanSub, 0.5);  
    if ~isempty(scanMsg)

        angleMin = scanMsg.AngleMin;
        angleIncrement = scanMsg.AngleIncrement;
        
        zeroAngleIndex = 1;
        

        if zeroAngleIndex >= 1 && zeroAngleIndex <= length(scanMsg.Ranges)
            frontDistance = scanMsg.Ranges(zeroAngleIndex);
            

            if frontDistance < d_threshold
                disp(['Object detected at ', num2str(frontDistance), ' m. Stopping...']);
                msg.Linear.X = 0.0;
                send(velPub, msg);
                break; 
            end
            

            fprintf('Front distance: %.3f m (threshold: %.3f m)\n', frontDistance, d_threshold);
        else
            warning('Could not find 0° angle in scan data.');
        end
    end
    

    send(velPub, msg);
    

    waitfor(rate);
end


msg.Linear.X = 0.0;
send(velPub, msg);


stats = statistics(rate);



rosshutdown;