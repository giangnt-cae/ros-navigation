# Identification
string id              # Unique identifier of the AGV
# string manufacturer    # Manufacturer of the AGV
string model           # Model of the AGV
# string serial_number   # Serial number for tracking

# Status
string state           # Current state of the AGV (e.g., "idlexxx", "movingxx", "charging", "errorxxx")
float32 battery_level  # Battery level in percentage (0-100)

# Pose and Motion
geometry_msgs/Pose current_pose   # Current position and orientation of the AGV (x, y, z + quaternion)
geometry_msgs/Twist velocity      # Current linear and angular velocity of the AGV
# float32 target_speed              # Target speed of the AGV (m/s)

# Dimensions
float32 length         # AGV length in meters
float32 width          # AGV width in meters
float32 height         # AGV height in meters
# float32 turning_radius # Minimum turning radius of the AGV in meters

# Circular Footprint (for round AGVs)
float32 radius          # Radius of the AGV in meters

# Load Information
# bool has_load           # Indicates whether the AGV is carrying a load
# float32 load_weight     # Weight of the load (in kilograms)
# string load_type        # Description of the type of load (e.g., "pallet", "box", etc.)
# geometry_msgs/Point load_center  # Center of mass of the load relative to AGV base frame

# Operational Parameters
# float32 max_payload     # Maximum payload the AGV can carry (in kilograms)
# float32 max_speed       # Maximum speed of the AGV (m/s)
# float32 max_runtime     # Maximum runtime (hours) on a full charge

# Task Information
string task_id          # ID of the current task or mission
string target_location  # Name of the target destination (e.g., "Station A", "Dock 3")
float32 eta             # Estimated time of arrival at the target location (in seconds)
bool is_docked          # Indicates whether the AGV is currently docked at a station

# Fault and Diagnostics
# bool is_faulted                 # Indicates if the AGV has encountered a fault
# string fault_description        # Description of the fault (if any)
# std_msgs/Time fault_time        # Time at which the fault occurred
# std_msgs/Time last_maintenance  # Timestamp of the last maintenance check

# Connectivity
bool is_connected       # Indicates if the AGV is connected to the fleet management system
string network_status   # Status of the network (e.g., "connected", "disconnected", "weak signal")

