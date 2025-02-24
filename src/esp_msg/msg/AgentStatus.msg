uint8 drone_id                      # Drone ID             
geometry_msgs/Point point           # local position [m] NED local world frame

uint32 state
uint32 STATE_IDLE = 0
uint32 STATE_ARM = 1
uint32 STATE_TELEOP = 2
uint32 STATE_HOLD = 3
uint32 STATE_LOAD = 4
uint32 STATE_TRACK = 5
uint32 STATE_DROP = 6
