# MCM identity and firmware versions
std_msgs/Header header

# Endpoint this identity belongs to
uint8 bus_id
uint8 subsystem_id

# From IdentifyResponseMain
uint32 module_serial_number
uint16 product_id
uint8 module_boot_state

# Application firmware version
uint16 app_version_major
uint16 app_version_minor
uint16 app_version_patch

# Bootloader firmware version
uint16 bl_version_major
uint16 bl_version_minor
uint16 bl_version_patch

# Which response frames have been received
bool has_main
bool has_app_version
bool has_bl_version
