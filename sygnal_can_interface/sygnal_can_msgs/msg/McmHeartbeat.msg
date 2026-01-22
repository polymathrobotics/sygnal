# MCM heartbeat message containing system state information
# Header for timestamp
std_msgs/Header header

# System state constants
uint8 FAIL_HARD = 254
uint8 HUMAN_OVERRIDE = 253
uint8 FAIL_OPERATIONAL_2 = 242
uint8 FAIL_OPERATIONAL_1 = 241
uint8 MCM_CONTROL = 1
uint8 HUMAN_CONTROL = 0

# System state for this MCM subsystem (0-255)
uint8 state

# Interface states (5 interfaces, values 0-255 each)
uint8[] interface_states
