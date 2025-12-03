% Set the ROS_MASTER_URI environment variable to point to the robot's ROS master.
% This tells ROS clients (like MATLAB) where the core ROS services are running.
setenv('ROS_MASTERURLException','http://192.168.0.100:11311')  % Robot's IP address

% Set the ROS_IP environment variable to the IP address of the local machine (this computer).
% This ensures the ROS master can correctly communicate back to MATLAB.
% (Uncomment and adjust if needed; here, 192.168.0.189 is used as the host IP.)
setenv('ROS_IP','192.168.0.189')

% Initialize the ROS node in MATLAB with a custom node name.
% This registers the MATLAB session as a ROS node named "/matnode" on the network.
rosinit("NodeName","/matnode")
