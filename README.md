# Controller Utils

A ROS2 control utilities package that provides a powerful abstraction layer for managing hardware & reference interface registration, claiming, and releasing in ros2_control controllers.

## Overview

The `controller_utils` package simplifies the complex lifecycle management of ros2_control command and state interfaces. It provides a registry-based approach to:

- **Register** expected interfaces during controller configuration
- **Claim** interfaces when the controller becomes active
- **Read/Write** values through the interfaces during real-time control loops
- **Unclaim** clamed interfaces during deactivation while preserving registration
- **Unregister** interfaces during cleanup or error handling

This abstraction eliminates boilerplate code and reduces errors when working with multiple joint interfaces across different controller lifecycle states.

## Key Features

- **Type-safe interface management** - Templated registry classes provide compile-time safety
- **Ordered interface access** - Maintains consistent ordering for multi-DOF systems
- **Lifecycle-aware** - Designed to work with ros2_control controller lifecycle
- **Flexible configuration** - Support for optional interfaces and different interface types
- **Real-time safe** - No dynamic allocations during real-time control loops

## Core Concepts

### Interface Registry Pattern

The package uses a three-phase lifecycle pattern:

1. **Registration Phase** (`on_configure`) - Declare which interfaces are needed
2. **Claiming Phase** (`on_activate`) - Claim actual hardware interfaces
3. **Deactivation Phase** (`on_deactivate`/`on_fault`) -  Release interfaces by unclaiming them for another controller to claim. 
3. **Cleanup Phase** (`on_cleanup`) - Unregisters interfaces by unregistering them, allowing the controller to reconfigure and claim new interfaces that were not claimed previously.  

### Key Interface Types

#### Standard Interface Names
```cpp
namespace controller_utils::interfaces {
    constexpr char POSITION_INTERFACE_NAME[] = "position";
    constexpr char VELOCITY_INTERFACE_NAME[] = "velocity";
    constexpr char ACCELERATION_INTERFACE_NAME[] = "acceleration";
    constexpr char TORQUE_INTERFACE_NAME[] = "torque";
    constexpr char ERROR_CODE_INTERFACE_NAME[] = "error_code";
    // ... and more
}
```

## Usage Examples

### Example 1: Direct Usage with State and Command Interfaces

This example shows the basic pattern used in a ros2 controller for managing state and command interfaces through the controller lifecycle.

#### Step 1: Declare Member Variables

```cpp
#include <controller_utils/kinematic_interfaces/KinematicStateInterfaces.hpp>
#include <controller_utils/kinematic_interfaces/JointInterfaces.hpp>

class MyController : public controller_interface::ChainableControllerInterface {
private:
    // State interfaces for reading joint feedback
    controller_utils::interfaces::KinematicStateInterfaces mStateInterfaces;
    
    // Command interfaces for writing joint commands
    using JointCommandInterfaces = controller_utils::interfaces::JointInterfaces<
        hardware_interface::LoanedCommandInterface>;
    JointCommandInterfaces mCommandInterfaces;
};
```

#### Step 2: Register Interfaces in `on_configure`

```cpp
controller_interface::CallbackReturn MyController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    
    // Load parameters (joint names, interface types, etc.)
    mParams = mParamListener->get_params();
    
    // Define which state interfaces are optional
    // {position, velocity, torque}, errorCodes, accelerations
    const controller_utils::interfaces::OptionalStateInterfaces stateFlags{
        {false, false, true},  // pos/vel required, torque optional
        true,   // error codes are optional
        true    // accelerations are optional
    };
    
    // Register state and command interfaces for each joint
    for (const std::string& joint : mParams.joints) {
        // Register state interfaces (position, velocity, etc.)
        if (!mStateInterfaces.registerStateInterfaces(
                joint, 
                mParams.state_interfaces,  // e.g., ["position", "velocity"]
                stateFlags)) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                "Failed to register state interfaces for joint %s", joint.c_str());
            return CallbackReturn::ERROR;
        }
        
        // Register command interfaces (position, velocity, effort)
        const controller_utils::interfaces::OptionalInterfaces cmdFlags{
            false,  // position required, otherwise the controller will throw. 
            false,  // velocity required
            true    // torque optional
        };
        
        if (!mCommandInterfaces.registerBaseInterfaces(
                joint,
                mParams.command_interfaces,  // e.g., ["position", "velocity"]
                cmdFlags)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to register command interfaces for joint %s", joint.c_str());
            return CallbackReturn::ERROR;
        }
    }
    
    return CallbackReturn::SUCCESS;
}
```

#### Step 3: Provide Interface Configuration

```cpp
controller_interface::InterfaceConfiguration 
MyController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // Get the list of registered state interface names
    // Returns strings like: "joint_1/position", "joint_1/velocity", ...
    conf.names = mStateInterfaces.stateInterfaceConfiguration();
    
    return conf;
}

controller_interface::InterfaceConfiguration 
MyController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // Get the list of registered command interface names
    conf.names = mCommandInterfaces.baseInterfaceConfiguration();
    
    return conf;
}
```

#### Step 4: claim Interfaces in `on_activate`

```cpp
controller_interface::CallbackReturn MyController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    
    // claim state interfaces to actual hardware
    if (!mStateInterfaces.claimInterfaces(state_interfaces_)) {
        RCLCPP_ERROR(get_node()->get_logger(), 
            "Could not claim state interfaces");
        return CallbackReturn::ERROR;
    }
    
    // claim command interfaces to actual hardware
    if (!mCommandInterfaces.claimInterfaces(command_interfaces_)) {
        RCLCPP_ERROR(get_node()->get_logger(), 
            "Could not claim command interfaces");
        return CallbackReturn::ERROR;
    }
    
    // Initialize controller state from current hardware state
    const auto currentPositions = mStateInterfaces.getPositionValues();
    const auto currentVelocities = mStateInterfaces.getVelocityValues();
    
    // Set initial commands to current state (hold position)
    mCommandInterfaces.setPositionValues(currentPositions);
    mCommandInterfaces.setVelocityValues(
        std::vector<double>(currentPositions.size(), 0.0));
    
    return CallbackReturn::SUCCESS;
}
```

#### Step 5: Read and Write in Real-Time Loop

```cpp
controller_interface::return_type MyController::update_and_write_commands(
    const rclcpp::Time& time, 
    const rclcpp::Duration& period) {
    
    // READ: Get current joint states from hardware
    const auto positions = mStateInterfaces.getPositionValues();
    const auto velocities = mStateInterfaces.getVelocityValues();
    
    // Optional: Read accelerations if available
    const auto accelerations = mStateInterfaces.getAccelerationValues();
    
    // COMPUTE: Your control algorithm here
    std::vector<double> positionCommands(positions.size());
    std::vector<double> velocityCommands(positions.size());
    
    for (size_t i = 0; i < positions.size(); ++i) {
        // Example: Simple position tracking with velocity feedforward
        positionCommands[i] = computeDesiredPosition(i, positions[i]);
        velocityCommands[i] = computeDesiredVelocity(i, velocities[i]);
    }
    
    // WRITE: Send commands to hardware
    if (!mCommandInterfaces.setPositionValues(positionCommands)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set position commands");
        return return_type::ERROR;
    }
    
    if (!mCommandInterfaces.setVelocityValues(velocityCommands)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set velocity commands");
        return return_type::ERROR;
    }
    
    return return_type::OK;
}
```

#### Step 6: Cleanup on Deactivation

```cpp
void MyController::unclaimAllInterfaces() {
    // unclaim clamed interfaces but keep registration
    // This allows re-activation without re-registration
    mCommandInterfaces.unclaimBaseInterfaces();
    mStateInterfaces.unclaimStateInterfaces();
}

void MyController::unregisterInterfaces() {
    // Completely remove interface registrations
    // Used during cleanup or error recovery
    mCommandInterfaces.unregisterBaseInterfaces();
    mStateInterfaces.unregisterStateInterfaces();
}

controller_interface::CallbackReturn MyController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    // Release hardware interfaces but keep configuration
    unclaimAllInterfaces();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
    // Fully unregister all interfaces
    unregisterInterfaces();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_error(
    const rclcpp_lifecycle::State& previous_state) {
    // Clean up everything on error
    unregisterInterfaces();
    return CallbackReturn::SUCCESS;
}
```
---

## Lifecycle State Machine Summary

The controller_utils interfaces follow the ros2_control controller lifecycle:

```
┌──────────────┐
│  UNCONFIGURED│
└──────┬───────┘
       │ on_configure()
       │ ├─ registerStateInterfaces()
       │ └─ registerBaseInterfaces()
       ▼
┌──────────────┐
│  INACTIVE    │◄─────────────┐
└──────┬───────┘              │
       │ on_activate()        │ on_deactivate()
       │ ├─ unclaimInterfaces() │ └─ unclaimInterfaces()
       │ └─ claimInterfaces()  │
       ▼                      │
┌──────────────┐              │
│    ACTIVE    │──────────────┘
└──────┬───────┘
       │ update_and_write_commands()
       │ ├─ getPositionValues()
       │ ├─ getVelocityValues()
       │ ├─ setPositionValues()
       │ └─ setVelocityValues()
       │
       ▼ on_cleanup()
       │ └─ unregisterInterfaces()
       │
┌──────────────┐
│ FINALIZED    │
└──────────────┘
```

## API Reference

### RegistryInterface<T>

Core template for managing ordered interface collections.

**Key Methods:**
- `registerInterface()` - Declare an expected interface
- `claimInterfaces()` - Match and bind hardware interfaces
- `getValues()` - Read values from all clamed interfaces
- `setValues()` - Write values to all clamed interfaces
- `unclaimRegistry()` - Release clamed interfaces
- `unregisterInterfaces()` - Remove registration

### JointInterfaces<InterfaceType>

Manages position, velocity, and torque interfaces for joint hardware. This is a sugar coating around the above to make working with joints easier. 

**Key Methods:**
- `registerBaseInterfaces()` - Register pos/vel/torque interfaces
- `baseInterfaceConfiguration()` - Get list of interface names
- `claimInterfaces()` - Claim hardware interfaces
- `getPositionValues()`, `getVelocityValues()`, `getTorqueValues()`
- `setPositionValues()`, `setVelocityValues()`, `setTorqueValues()`
- `unclaimBaseInterfaces()` - Release without unregistering
- `unregisterBaseInterfaces()` - Complete cleanup

### KinematicStateInterfaces

Extends `JointInterfaces<LoanedStateInterface>` with acceleration and error code support.

**Additional Methods:**
- `registerStateInterfaces()` - Register all state channels
- `stateInterfaceConfiguration()` - Get complete state interface list
- `getAccelerationValues()` - Read accelerations
- `getErrorCodeValues()` - Read error codes

### JointReferenceInterface

Storage for reference commands used in controller chaining.

**Key Methods:**
- `getReferencePositions()`, `getReferenceVelocities()`, `getReferenceTorques()`
- `setPosition()`, `setVelocity()`, `setTorque()`
- `exportReferencePosition()` - Get raw pointer for external access

## Best Practices

1. **Always match lifecycle phases**
   - Register in `on_configure`
   - claim in `on_activate`
   - unclaim in `on_deactivate`
   - Unregister in `on_cleanup` and `on_error`

2. **Use unclaim vs. unregister appropriately**
   - `unclaim*` methods release hardware but preserve configuration (for re-activation)
   - `unregister*` methods remove configuration entirely (for cleanup/reconfiguration)

3. **Check return values**
   - Always check `registerInterfaces()` and `claimInterfaces()` return values. #TODO: Mark methods with `[[no discard]]`
   - Log errors with joint names for debugging

4. **Leverage optional flags**
   - Mark truly optional interfaces to improve controller portability
   - Required interfaces will fail fast with clear error messages

5. **Maintain consistent ordering**
   - The registry preserves declaration order
   - Critical for multi-DOF systems where joint order matters

6. **Consider using proxy pattern**
   - Encapsulate mode-specific logic (position vs. velocity)
   - Simplify controller implementation
   - Enable easier testing and maintenance

## See Also

- [ros2_control documentation](https://control.ros.org/master/doc/getting_started/getting_started.html)
- [Controller Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- [TrapezoidalJointController source]

## License
