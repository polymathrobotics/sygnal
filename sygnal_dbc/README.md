# sygnal_dbc

Provides Sygnal DBC files and a C library produced by cantools. The package
installs the raw DBC database and, at build time, generates `.c/.h` files for
each `.dbc` and exposes them via a static library for consumers.

## Contents
- Installed DBCs: `share/sygnal_dbc/database/...`
- Installed headers: `include/sygnal_dbc/<subdir>_<name>.h`
  - Example:
    - DBC Location `share/sygnal_dbc/database/mcm/Heartbeat.dbc`
    - Header Location `include/sygnal_dbc/mcm_heartbeat.h`
- Library: `sygnal_dbc`
- CMake var: `SYGNAL_DBC_DIR` → points to installed `database` directory

## Build
Prereqs:
- ROS 2 ament environment sourced
- Python 3 with cantools: `python3 -m pip install cantools`

Build only this package:

```
colcon build --packages-select sygnal_dbc
```

## Using in C++ (ament_cmake)
In your package CMakeLists.txt:

```
find_package(ament_cmake REQUIRED)
find_package(sygnal_dbc REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node )
# Link the library (headers are exported)
target_link_libraries(my_node sygnal_dbc)
```

Include headers (lowercase file basenames, folder layout mirrors `database/`):

```
#include <sygnal_dbc/cb/configuration.h>
#include <sygnal_dbc/mcm/heartbeat.h>
```

The exact header names come from each DBC file name, lowercased.

## Accessing raw DBCs
After `find_package(sygnal_dbc)`, the CMake variable `SYGNAL_DBC_DIR` is set to
`<install-prefix>/share/sygnal_dbc/database`. Use it to locate raw `.dbc` files:

```
# Example: install a copy of a specific dbc next to your target
install(FILES "${SYGNAL_DBC_DIR}/cb/Configuration.dbc" DESTINATION share/${PROJECT_NAME}/dbc)
```

## Notes
- The installed headers incorporate the folder structure but to prevent conflicts add prefixes `database_`
  (`cb_`, `io_`, `mcm_`, `vehicles_`).
- If you add or rename `.dbc` files, just rebuild; sources regenerate
  automatically.
