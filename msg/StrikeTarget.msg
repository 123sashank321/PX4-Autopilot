# StrikeTarget.msg
# Final simplified version for Strike/Abort operations

uint64 timestamp          # time since system start (microseconds)

float64 lat               # Target Latitude [deg]
float64 lon               # Target Longitude [deg]
float32 alt               # Target Altitude [m]

uint8 action_type         # 0 = STRIKE, 1 = ABORT

uint32 MESSAGE_VERSION = 0
