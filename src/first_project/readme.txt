Since the reference system of the Wheel odometry isn't exactly aligned with the ENU reference system, when displayed together the two odometry don't go in the same direction.
I tried to rotate the ENU reference system by ~120° counterclock-wise and I was able to verify that the two odometry represent the same movement.
I left a parameter to alter in the gps_to_odom.cpp file that allows to perform such rotation of the reference system to allign it with the wheel odometry one.

The Lidar Points projection with the GPS odometry isn't exactly as still as the one performed with the wheel odometry. 
That is due to the fact that the GPS odometry is updated only five times a second, meaning that most of the time there'll be slight error when computing the heading (yaw) of the robot.