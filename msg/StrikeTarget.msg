uint64 timestamp        # time since system start (microseconds)

float32 x               # Target North (meters)
float32 y               # Target East (meters)
float32 z               # Target Down (meters)

bool active             # Guidance activation state
uint8 action_type       # 0=Strike, 1=Abort

uint8 ACTION_STRIKE = 0
uint8 ACTION_ABORT = 1
