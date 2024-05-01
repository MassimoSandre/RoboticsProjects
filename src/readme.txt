In the launch file rviz is started with config loaded from a file located in src/rviz/config.rviz.

in gps_to_odom.cpp is possible to set parameters to adjust the gps odometry data.
In particular, it is possible to align the gps odometry reference system with the wheel odometry one.
There's also a parameter that allows to set the minimum distance that the robot
needs to travel in order for heading to be computed. Increasing the value helps to
mitigate the spinning effect shown in the gps odometry at the beginning and at the end
of the bag.

pitch value in the robot orietation isn't computed.
I chose not to compute it because the wheel odometry doesn't provide that value.
Also, most of the robot movement doens't involve many pitch rotation.