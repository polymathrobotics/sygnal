# MVEC Relay Control Library

A comprehensive C++ library for controlling and monitoring MVEC (multiplexed Vehicle Electrical Center) devices over J1939 CAN bus. This library provides both synchronous and asynchronous interfaces for relay control, status monitoring, and device management.

## Core Components

### MvecRelay
The foundational class providing J1939 message parsing, relay command generation, and status message handling. Supports all MVEC protocol operations including relay control, population queries, and diagnostic monitoring.

### MvecRelaySocketcan
High-level asynchronous interface built on top of MvecRelay using SocketCAN for Linux CAN communication. Features thread-safe promise/future patterns for non-blocking operations and automatic response handling.

## Quick Start

### Build The Package

```bash
colcon build --packages-select mvec_lib
```

### Test the Package WITH Hardware
Remove CAN_AVAILABLE=1 to run tests without can

```bash
CAN_AVAILABLE=1 colcon test --packages-select mvec_lib
```

And check results!

```bash
colcon test-result
```

You can also run ONLY the necessary test

```bash
cd build/mvec_lib && ./mvec_socketcan_hardware.cpp
```

### Basic Setup

```cpp
#include "mvec_lib/mvec_relay_socketcan.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

// Create SocketCAN adapter
auto adapter = std::make_shared<socketcan::SocketcanAdapter>("can0");
adapter->open();

// Create MVEC controller
polymath::sygnal::MvecRelaySocketcan controller(adapter);

// Set up callback for incoming messages
adapter->setReceptionCallback([&controller](const socketcan::CanFrame& frame) {
    controller.parse(frame);
});
```

### Relay Control

```cpp
// Set relay states
controller.set_relay_in_command(0, 1);  // Turn on relay 0
controller.set_relay_in_command(1, 0);  // Turn off relay 1

// Send command and wait for confirmation
auto command_future = controller.send_relay_command();
auto result = command_future.wait_for(std::chrono::milliseconds(500));

if (result == std::future_status::ready) {
    auto reply = command_future.get();
    if (reply.is_valid() && reply.get_success()) {
        // Command executed successfully
        bool relay_0_result = reply.get_relay_result(0);
    }
}
```

### Query Operations

```cpp
// Query current relay states
auto state_future = controller.get_relay_state();
auto state_result = state_future.wait_for(std::chrono::milliseconds(1000));

if (state_result == std::future_status::ready) {
    auto state_reply = state_future.get();
    if (state_reply.is_valid()) {
        bool relay_0_on = state_reply.get_relay_state(0);
        bool relay_0_default = state_reply.get_relay_default(0);
    }
}

// Query device population
auto pop_future = controller.get_relay_population();
auto pop_result = pop_future.wait_for(std::chrono::milliseconds(1000));

if (pop_result == std::future_status::ready) {
    auto pop_reply = pop_future.get();
    if (pop_reply.is_valid()) {
        bool relay_0_installed = pop_reply.get_relay_population(0);
        bool fuse_5_installed = pop_reply.get_fuse_population(5);
    }
}
```

### Status Monitoring

```cpp
// Access last received status messages
auto fuse_status = controller.get_last_fuse_status();
if (fuse_status && fuse_status->is_valid()) {
    auto status = fuse_status->get_fuse_status(0);
    if (status == polymath::sygnal::MvecFuseStatus::BLOWN) {
        // Handle blown fuse
    }
}

auto relay_status = controller.get_last_relay_status();
if (relay_status && relay_status->is_valid()) {
    auto status = relay_status->get_relay_status(0);
    if (status != polymath::sygnal::MvecRelayStatus::OKAY) {
        // Handle relay fault
    }
}

auto error_status = controller.get_last_error_status();
if (error_status && error_status->is_valid()) {
    if (error_status->has_error(polymath::sygnal::MvecErrorType::OVER_VOLTAGE)) {
        // Handle over-voltage condition
    }
}
```

## Low-Level Interface

For applications requiring direct message control, use the MvecRelay class:

```cpp
#include "mvec_lib/mvec_relay.hpp"

polymath::sygnal::MvecRelay relay;

// Parse incoming CAN frames
auto message_type = relay.parseMessage(can_frame);

// Generate command messages
relay.set_relay_in_command(0, 1);
auto command_frame = relay.getRelayCommandMessage();

// Access parsed data
const auto& fuse_msg = relay.get_fuse_status_message();
if (fuse_msg.is_valid()) {
    auto status = fuse_msg.get_fuse_status(0);
}
```

## Hardware Definitions

- **Relays**: Up to 12 individually controllable relays (ID 0-11)
- **Fuses**: Up to 24 monitored fuses (ID 0-23)
- **High-Side Outputs**: 1 high-side output per device
- **CAN Bus**: J1939 protocol over standard CAN 2.0B

## Integration Requirements

- **SocketCAN Adapter**: Compatible socketcan_adapter library
- **CAN Hardware**: Linux SocketCAN-compatible CAN interface
