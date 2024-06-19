define laser ranger
(
  sensor
  (
    # I assumed that the the robot mounted a 360° lidar
    # I took inspiration from the VLP-16 for the data
    fov 360.0  # Field of view of the laser in degrees
    samples 720  # Number of laser samples
    range [0.1 100]  # Distance range of the laser in meters
    resolution 0.01  # Resolution of the laser sensor in meters
    frequency 10  # Scanning frequency of the laser in Hz
    noise 0.01  # Noise of the sensor, standard deviation
    pose [0.0 0.0 0.245 0.0]  # Position and orientation of the laser on the robot
  )
  # generic model properties
  color "black"
  size [ 0.0889 0.0889 0.0717 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.005 0.005 0.005 0.005 0.01 0.01]

  size [ 0.612 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"
  
  laser(pose [ 0.0 0.0 0.0 0.0 ])
)