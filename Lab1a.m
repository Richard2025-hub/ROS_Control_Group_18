rplid = rossubscriber("/scan","sensor_msgs/LaserScan","DataFormat","struct");
scan = receive(rplid,10);

Lidscan = rosReadLidarScan(scan);

formatSpec = 'Angle(Front): %10.5f\nDistance (Front):%10.5f\n';
fprintf(formatSpec, rad2deg(Lidscan.Angles(1)),Lidscan.Ranges(1))

formatSpec = 'Angle Increment (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleIncrement))


formatSpec = 'Angle Min (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleMin))


formatSpec = 'Angle Max (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleMax))


formatSpec = 'Range Min (m): %f\n';
fprintf(formatSpec, rad2deg(scan.RangeMin))


formatSpec = 'Range Max (m): %f\n';
fprintf(formatSpec, rad2deg(scan.RangeMax))
