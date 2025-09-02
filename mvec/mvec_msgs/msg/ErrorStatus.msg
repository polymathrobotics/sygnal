# MVEC Error Status Message
# Contains general error information from the MVEC device

# Error bits representing various system errors
uint16 error_bits

# Grid address of the reporting device
uint8 grid_address

# Whether this message contains valid data
bool is_valid
