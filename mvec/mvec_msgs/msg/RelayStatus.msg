# MVEC Relay Status Message
# Contains status information for all relays in the MVEC device

# Relay status constants (MvecRelayStatus enum)
uint8 OKAY = 0
uint8 COIL_OPEN = 1
uint8 COIL_SHORTED_OR_DRIVER_FAILED = 2
uint8 NO_CONTACT_OPEN = 3
uint8 NC_CONTACT_OPEN = 4
uint8 COIL_NOT_RECEIVING_POWER = 5
uint8 NO_CONTACT_SHORTED = 6
uint8 NC_CONTACT_SHORTED = 7
uint8 WELDED_CLOSED = 8
uint8 WELDED_OPEN = 9
uint8 RELAY_LOCATION_NOT_USED = 10

# Relay status for each relay (0-11)
uint8[12] relay_status

# Grid address of the reporting device
uint8 grid_address

# Whether this message contains valid data
bool is_valid
