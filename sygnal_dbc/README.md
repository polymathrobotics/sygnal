# sygnal_dbc

Sygnal DBC databases and the generated C/C++ CAN message types built from them.

Each subsystem directory is its own `ament_cmake` package that generates and exports
idiomatic C/C++ CAN message types from the DBC files it contains, using `dbc_gen_cpp`:

| Directory      | Package                   | Library prefix  |
|----------------|---------------------------|-----------------|
| `mcm/`         | `dbc_sygnal_mcm`          | `mcm_`          |
| `io/`          | `dbc_sygnal_io`           | `io_`           |
| `cb/`          | `dbc_sygnal_cb`           | `cb_`           |
| `cb_niro_dcan/`| `dbc_sygnal_cb_niro_dcan` | `cb_niro_dcan_` |
| `hpo/`         | `dbc_sygnal_hpo`          | `hpo_`          |
| `vehicles/`    | `dbc_sygnal_vehicles`     | `vehicles_`     |
| `sygnal_dbc/`  | `sygnal_dbc`              | — (umbrella)    |

## How generation works

Each package's `CMakeLists.txt` is a thin stub calling the shared `dbc_package()` helper
(`cmake/dbc_package.cmake`). The helper globs every `*.dbc` in the directory and produces
one exported library per file, so **adding a new DBC needs no CMake changes**.

The library — and its C++ namespace, C-symbol prefix and header directory — is
`<prefix>_<dbc file stem, lowercased>`:

- `mcm/Control.dbc` → library `mcm_control`, headers `mcm_control/mcm_control.{h,hpp}`,
  namespace `mcm_control`, C symbols `mcm_control_*`.

The `PREFIX` is required because stems repeat across subsystems: `Control.dbc`,
`Heartbeat.dbc`, `Identify.dbc`, `Configuration.dbc`, `Error.dbc` and `Commissioning.dbc`
each exist in `cb/`, `hpo/`, `io/` **and** `mcm/`. Naming libraries after the bare stem
would give four libraries called `Control`, all defining `control_*` symbols and all
installing `include/Control/Control.hpp` — a duplicate-symbol error for anything linking
two of them.

`cb_niro_dcan/` is a sibling directory rather than `cb/niro_dcan/` because colcon does not
descend into a directory it has already identified as a package. The installed
`database/` tree still presents it at `cb/niro_dcan/`.

## Using in C++ (ament_cmake)

Add the subsystem package(s) you need as a `<depend>` in your `package.xml`, then link the
generated libraries by name:

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()   # finds dbc_sygnal_mcm (a depend)

# Generated libs are exported as namespaced imported targets: <package>::<lib>.
target_link_libraries(my_node PRIVATE dbc_sygnal_mcm::mcm_control dbc_sygnal_mcm::mcm_relay)
```

```cpp
#include "mcm_control/mcm_control.hpp"   // C++ API
mcm_control::ControlCommand msg;

extern "C" {
#include "mcm_control/mcm_control.h"     // C API
}
```

## Backwards compatibility

This package previously built a single aggregate library named `sygnal_dbc` and installed
every generated header as `sygnal_dbc/<prefix>_<name>.h`. Both still work:

- the `sygnal_dbc` package remains, as an INTERFACE library linking every generated
  library, so `<depend>sygnal_dbc</depend>`, `ament_target_dependencies(<tgt> sygnal_dbc)`
  and `target_link_libraries(<tgt> sygnal_dbc)` all keep resolving;
- forwarding headers at `include/sygnal_dbc/<lib>.{h,hpp}` include the real
  `<lib>/<lib>.{h,hpp}`.

The generated **C symbols are unchanged** across the split: the old build passed
`--database-name <prefix>_<name>` to cantools, which is exactly what the library names are
now. Existing consumers therefore need no changes at all. New code should include
`"<lib>/<lib>.hpp"` and link `dbc_sygnal_<subsystem>::<lib>` directly — the C++ API is new,
the old aggregate library only ever exposed the C one.

## Accessing raw DBCs

`find_package(sygnal_dbc)` sets `SYGNAL_DBC_DIR` to `<install-prefix>/share/sygnal_dbc/database`,
with the original `<subsystem>/<Name>.dbc` layout:

```cmake
install(FILES "${SYGNAL_DBC_DIR}/cb/Configuration.dbc" DESTINATION share/${PROJECT_NAME}/dbc)
```

Each subsystem package also installs its own DBCs under `share/<package>/database`.

## Build

```
colcon build --packages-up-to sygnal_dbc
```
