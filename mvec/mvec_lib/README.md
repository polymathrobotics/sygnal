# MVEC Relay Control Library

A comprehensive C++ library for controlling and monitoring MVEC (Multi-Vehicular Electronic Controller) devices over J1939 CAN bus. This library provides both synchronous and asynchronous interfaces for relay control, status monitoring, and device management.

## Core Components

### MvecRelay
The foundational class providing J1939 message parsing, relay command generation, and status message handling. Supports all MVEC protocol operations including relay control, population queries, and diagnostic monitoring.

### MvecRelaySocketcan
High-level asynchronous interface built on top of MvecRelay using SocketCAN for Linux CAN communication. Features thread-safe promise/future patterns for non-blocking operations and automatic response handling.

## Features

- **Relay Control**: Set individual relay states (on/off) and high-side outputs
- **Status Monitoring**: Real-time monitoring of fuse status, relay diagnostics, and error conditions
- **Population Queries**: Discover which relays and fuses are physically installed
- **Asynchronous Operations**: Non-blocking commands using std::future with timeout support
- **Message Parsing**: Comprehensive J1939 CAN frame parsing and validation
- **Thread Safety**: Mutex-protected promise queues for concurrent access
- **Hardware Integration**: Direct SocketCAN adapter integration for CAN bus communication

## Quick Start

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

## Hardware Limits

- **Relays**: Up to 12 individually controllable relays (ID 0-11)
- **Fuses**: Up to 24 monitored fuses (ID 0-23)
- **High-Side Outputs**: 1 high-side output per device
- **CAN Bus**: J1939 protocol over standard CAN 2.0B

## Status Types

### Fuse Status
- `NO_FAULT`: Normal operation
- `BLOWN`: Overcurrent protection activated
- `NOT_POWERED`: No power supply detected
- `NOT_USED`: Fuse position not populated

### Relay Status
- `OKAY`: Normal operation
- `COIL_OPEN`: Open coil circuit
- `COIL_SHORTED_OR_DRIVER_FAILED`: Shorted coil or driver failure
- `NO_CONTACT_OPEN`/`NC_CONTACT_OPEN`: Contact stuck open
- `NO_CONTACT_SHORTED`/`NC_CONTACT_SHORTED`: Contact shorted
- `HIGH_SIDE_DRIVER_FAULT`: High-side driver malfunction

### Error Status
Comprehensive error reporting including communication errors, voltage conditions, configuration issues, and firmware validation failures.

## Threading and Safety

The MvecRelaySocketcan class is designed for multi-threaded environments:
- Promise queues are mutex-protected for concurrent access
- Parse method is thread-safe and can be called from CAN reception callbacks
- Future-based operations allow non-blocking command execution
- Optional status getters provide safe access to last received data

## Integration Requirements

- **SocketCAN Adapter**: Compatible socketcan_adapter library
- **CAN Hardware**: Linux SocketCAN-compatible CAN interface
- **Compiler**: C++14 or later with std::future support
- **Threading**: std::mutex and std::queue support required

## Error Handling

All operations return validation status through:
- `is_valid()` methods on message objects
- Future timeout handling for async operations
- Optional return types for status getters
- Comprehensive error status reporting through dedicated error messages

This library provides robust, production-ready MVEC device control suitable for industrial automation, automotive applications, and embedded systems requiring reliable relay control over CAN bus networks.
