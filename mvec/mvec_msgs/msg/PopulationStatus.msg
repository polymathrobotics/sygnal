# MVEC Population Status Message
# Contains information about which relays and fuses are physically installed

# Population status for each relay (0-11) - true if populated/installed
bool[12] relay_population

# Population status for each fuse (0-15) - true if populated/installed
bool[16] fuse_population

# Whether high side output is populated
bool high_side_output_population

# Grid address of the reporting device
uint8 grid_address

# Whether this message contains valid data
bool is_valid
