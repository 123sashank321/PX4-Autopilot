uint64 timestamp        # time since system start (microseconds)

float32 x               # Target NED-x / North [m]
float32 y               # Target NED-y / East  [m]
float32 z               # Target NED-z / Down  [m]  (positive = below home)

float32 ip_x            # Initial Point NED-x [m]
float32 ip_y            # Initial Point NED-y [m]
float32 ip_z            # Initial Point NED-z [m]  (negative = above home)

float32 ahp_x           # Attack Heading Point NED-x [m]
float32 ahp_y           # Attack Heading Point NED-y [m]
float32 ahp_z           # Attack Heading Point NED-z [m]

float32 x_kinematic     # Horizontal reach of terminal dive [m] — TERMINAL transition threshold

bool active             # Guidance activation state
uint8 action_type       # 0=Strike, 1=Abort

uint8 ACTION_STRIKE = 0
uint8 ACTION_ABORT = 1
