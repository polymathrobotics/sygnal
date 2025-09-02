# MVEC feedback message containing current relay states
# Header for timestamp
std_msgs/Header header

# Current state of all relays (array of relay states)
Relay[] relay_states

# High side output state
bool high_side_output_state
