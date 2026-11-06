# State of one drone, published by drone_commander (10 Hz) on /<name>/state.
# Shared with the other drones / GCS through the Zenoh bridge.
#
# header.frame_id = "map": swarm frame, ENU (x East, y North, z Up), origin =
# swarm origin (lat / lon of the Gazebo world in simulation). z = height above
# the take-off point of this drone (its EKF origin).

std_msgs/Header header
string name
uint8 id

# --- flight controller (ArduPilot through MAVROS) ---
bool connected                    # MAVROS <-> FCU heartbeat OK
bool armed
bool guided                       # mode == GUIDED (accepts ROS 2 commands)
string mode                       # ArduPilot mode: GUIDED, LOITER, RTL, LAND, AUTO, STABILIZE...
uint8 system_status               # MAV_STATE: 3 standby, 4 active, 5 critical, 6 emergency

# --- position / velocity (EKF3 of ArduPilot) ---
bool position_valid               # local + global position available, swarm frame offset known
geometry_msgs/Point position      # m, swarm frame "map"
geometry_msgs/Vector3 velocity    # m/s, ENU
float64 yaw                       # rad, ENU (0 = East, counter-clockwise +)
float64 latitude                  # deg
float64 longitude                 # deg
float32 relative_altitude         # m above home (FCU)

# --- battery ---
float32 battery_voltage           # V
float32 battery_remaining         # 0..1, NaN = unknown

# --- active command of drone_commander ---
uint8 SETPOINT_NONE = 0           # no ROS command (ArduPilot holds / executes takeoff, RTL...)
uint8 SETPOINT_VELOCITY = 1
uint8 SETPOINT_POSITION = 2
uint8 SETPOINT_GLOBAL = 3
uint8 setpoint_type
geometry_msgs/Point target        # position target in the "map" frame (SETPOINT_POSITION)
