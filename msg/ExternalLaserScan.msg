# Single scan from a planar laser range-finder and the pose of the robot

sensor_msgs/LaserScan scan
geometry_msgs/TransformStamped scanner_offset   # Scanner pose in robot base_frame
geometry_msgs/PoseWithCovarianceStamped pose    # Robot base_frame pose in a global (map) frame
