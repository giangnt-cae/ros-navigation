uint16 id                               # Unique identifier of the AGV
string state                             # Current state of the AGV
geometry_msgs/PoseStamped robot_pose    # Current position and orientation of the AGV (x, y, z + quaternion)
geometry_msgs/Twist velocity            # Current linear and angular velocity of the AGV
geometry_msgs/PolygonStamped footprint  # Footprint of AGV
nav_msgs/OccupancyGrid map              
nav_msgs/Path path                      
