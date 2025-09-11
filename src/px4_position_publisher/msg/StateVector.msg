std_msgs/Header header

# Position in ENU frame
geometry_msgs/Point position

# Velocity in ENU frame  
geometry_msgs/Vector3 velocity

# Acceleration in ENU frame
geometry_msgs/Vector3 acceleration

# Orientation as quaternion (ENU frame)
geometry_msgs/Quaternion orientation

# Angular velocity in body frame
geometry_msgs/Vector3 angular_velocity

# Covariance matrices (optional, 6x6 for pose, 6x6 for twist)
float64[36] pose_covariance
float64[36] twist_covariance