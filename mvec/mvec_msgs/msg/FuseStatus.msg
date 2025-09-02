# MVEC Fuse Status Message
# Contains status information for all fuses in the MVEC device

# Fuse status constants (MvecFuseStatus enum)
uint8 NO_FAULT = 0
uint8 BLOWN = 1
uint8 NOT_POWERED = 2
uint8 NOT_USED = 3

# Fuse status for each fuse (0-15)
uint8[16] fuse_status

# Grid address of the reporting device
uint8 grid_address

# Whether this message contains valid data
bool is_valid
