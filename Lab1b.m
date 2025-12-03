[pub,msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
msg.Linear.X = 0.2;
msg.Angular.Z = -0.5;


desiredRate = 10;
rate = robotics.Rate(desiredRate);
rate.OverrunAction = 'drop';

reset(rate)

while rate.TotalElapsedTime < 3
    send(pub,msg)
    waitfor(rate);
end

msg.Linear.X = 0.0;
msg.Angular.Z = 0.0;
send(pub,msg)

statistics(rate)