# MVEC ROS 2

ROS2 composable lifecycle node for MVEC relay control system.

## Usage

### Launch

```bash
ros2 launch mvec_ros2 mvec_bridge_launch.py
```

### Parameters
- `can_interface`: CAN interface (default: can0)
- `config_file`: Preset config file (default: package config/relay_presets.yaml)
- `publish_rate`: Status publish rate in Hz (default: 3.0)
- `timeout_ms`: Communication timeout in ms (default: 500)

### Services
- `~/set_single_relay` - Set individual relay state
- `~/set_multi_relay` - Set multiple relay states
- `~/trigger_preset` - Trigger named preset

### Topics
- `~/feedback` - Current relay states (MvecFeedback)
- `/diagnostics` - System diagnostics with relay status

### Configuration
Define presets in `config/relay_presets.yaml`:

```yaml
mvec_node:
  ros__parameters:
    preset_0_name: "enable"
    preset_0_relays: ["0:1", "2:1", "4:0"]
```

