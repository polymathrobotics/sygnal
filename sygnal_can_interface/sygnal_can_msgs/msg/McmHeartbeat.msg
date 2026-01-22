# MCM heartbeat message containing system state information
# Header for timestamp
std_msgs/Header header

# System state for MCM subsystem 0 (0-255)
uint8 mcm0_state

# System state for MCM subsystem 1 (0-255)
uint8 mcm1_state

# Interface states (5 interfaces, values 0-255 each)
uint8[] interface_states
