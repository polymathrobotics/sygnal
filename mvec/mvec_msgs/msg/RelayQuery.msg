# MVEC Relay Query Message
# Contains current relay states and default power-on states

# Relay state constants
uint8 RELAY_OFF = 0
uint8 RELAY_ON = 1

# Current relay states for each relay (0-11)
# 0 = RELAY_OFF, 1 = RELAY_ON
uint8[12] relay_states

# Default power-on states for each relay (0-11)
# 0 = RELAY_OFF, 1 = RELAY_ON
uint8[12] relay_defaults

# High side output current state
uint8 high_side_output_state

# High side output default state
uint8 high_side_output_default

# Grid address of the reporting device
uint8 grid_address

# Whether this message contains valid data
bool is_valid
