# Motor Configuration and motor_node.cpp Documentation

## Overview

The motor control system has been refactored to use a centralized YAML configuration file instead of hardcoding motor properties. This allows for:
- Easy modification of motor setup without recompilation
- Clear separation of hardware configuration from control logic
- Consistent motor and leg management across the system

## Files

### 1. Configuration File: `conf/motors.yaml`

This YAML file contains all motor and leg configurations. It has three main sections:

#### Global Section
```yaml
global:
  zero_position: 1.5707963267948  # π/2 radians (shared offset for certain motors)
```

#### Motors Section
Each motor has a unique configuration block:

```yaml
motors:
  MOTOR_NAME:
    motor_type: RS06              # Motor model type (e.g., RS06, RS03, RS02, etc.)
    slave_id: 0x01                # CAN slave ID (hex format)
    master_id: 0x0B               # CAN master ID (hex format)
    initial_position: 0.0          # Initial position in radians
    direction: 1                   # Rotation direction: 1 (normal) or -1 (reversed)
```

**Available motor types:** DM4310, DM4310_48V, DM4340, DM4340_48V, DM6006, DM8006, DM8009, DM10010L, DM10010, DMH3510, DMH6215, DMG6220, RS00, RS01, RS02, RS03, RS04, RS05, RS06

#### Legs Section
Each leg groups motors and defines overall leg behavior:

```yaml
legs:
  LEG_NAME:
    name: "Human Readable Name"
    can_port: 0                    # CAN port index (0, 1, 2, 3)
    num_motors: 4                  # Number of motors in this leg
    motors: [M0_NAME, M1_NAME, M2_NAME, M3_NAME]  # References to motor names
    directions: [1, 1, -1, 1]      # Per-motor direction multipliers
    zero_position_indices: [2]     # Motor indices that use ZERO_POS offset
    zero_position_offset: -1       # Offset sign for ZERO_POS (-1 or 1)
```

## Source Code: `src/motor_node.cpp`

### Architecture

The implementation uses a `MotorControlSystem` class that encapsulates all motor control logic:

#### Key Components

1. **MotorConfig Struct**: Stores individual motor configuration
   - Motor type, CAN IDs, initial position, direction

2. **LegConfig Struct**: Stores leg-level configuration
   - CAN port, motors list, direction multipliers, zero position handling

3. **MotorControlSystem Class**: Main control system
   - Loads YAML configuration
   - Manages HSCanT handlers (one per CAN port)
   - Controls motor_Control instances (one per leg)
   - Handles command input and state output
   - Applies direction multipliers and position offsets

#### Main Methods

- **`loadConfiguration(config_file)`**: Parses YAML file and populates configs
- **`initialize()`**: Sets up CAN handlers, motor controllers, and motors
- **`cmdCallback(msg)`**: Receives commands from `/dm_cmd` topic
- **`controlStep()`**: Sends MIT control commands to motors with proper transformations
- **`updateState()`**: Reads motor states and applies reverse transformations
- **`publishState(pub)`**: Publishes state to `/dm_states` topic
- **`shutdown()`**: Cleanly disables all motors and releases resources

### Control Flow

1. **Configuration Loading**:
   - YAML file is read at startup
   - Motor and leg configurations are parsed
   - Motor types are converted from strings to enums

2. **Initialization**:
   - HSCanT library is initialized
   - For each leg:
     - CAN handler is created (linked to specific CAN port)
     - Motor_Control instance is created
     - Motor objects are instantiated with correct IDs
     - Motors are added to the controller
   - All controllers are enabled
   - Initial motor positions are logged

3. **Runtime Operation** (100 Hz loop):
   - Receive commands via ROS subscription
   - Apply direction multipliers to commands
   - Send MIT control to each motor
   - Read motor feedback states
   - Apply reverse transformations to states
   - Publish state via ROS

### Direction and Offset Handling

The system applies two types of transformations:

**For Commands (Motor Input):**
```
q_cmd = input_q * direction - zero_offset
dq_cmd = input_dq * direction
tau_cmd = input_tau * direction
```

**For State (Motor Output):**
```
pos_state = motor_pos * direction + zero_offset
vel_state = motor_vel * direction
tau_state = motor_tau * direction
```

Where:
- `direction`: Per-motor direction multiplier (±1)
- `zero_offset`: Applied only if motor index is in `zero_position_indices`

### Configuration File Path

The node attempts to find the config file in this order:
1. Parameter `/motor_node/config_file` (if set)
2. Default: `<package>/conf/motors.yaml`

## Leg Configuration Example

The default configuration defines four legs:

- **FL (Front Left)**: CAN port 0, 4 motors
  - Zero position offset: -1 (subtract ZERO_POS)
  - Directions: [1, 1, -1, 1]

- **FR (Front Right)**: CAN port 1, 4 motors
  - Zero position offset: +1 (add ZERO_POS)
  - Directions: [1, -1, 1, 1]

- **BL (Back Left)**: CAN port 2, 4 motors
  - Zero position offset: -1
  - Directions: [-1, 1, -1, 1]

- **BR (Back Right)**: CAN port 3, 4 motors
  - Zero position offset: +1
  - Directions: [-1, -1, 1, 1]

## ROS Integration

### Topics

**Subscribed:**
- `/dm_cmd` (dm_motor_ros/DmCommand): Command input
  - `pos[]`: Position commands
  - `vel[]`: Velocity commands
  - `tau[]`: Torque commands
  - `kp[]`: Position gains
  - `kd[]`: Velocity gains

**Published:**
- `/dm_states` (dm_motor_ros/DmState): State feedback
  - `joint_names[]`: Motor names
  - `pos[]`: Positions (with transformations applied)
  - `vel[]`: Velocities (with transformations applied)
  - `tau[]`: Torques (with transformations applied)

### Command Order

Commands are expected in this leg order:
```
[FL_M0, FL_M1, FL_M2, FL_M3, 
 FR_M0, FR_M1, FR_M2, FR_M3, 
 BL_M0, BL_M1, BL_M2, BL_M3, 
 BR_M0, BR_M1, BR_M2, BR_M3]
```

State feedback follows the same order.

## Customization Guide

### Adding a New Motor

1. Add entry to `motors` section in YAML:
```yaml
  MY_NEW_MOTOR:
    motor_type: RS06
    slave_id: 0x05
    master_id: 0x0F
    initial_position: 0.0
    direction: 1
```

2. Reference it in the leg's `motors` list and update other arrays accordingly

### Adding a New Leg

1. Add a new leg configuration:
```yaml
  NEWLEG:
    name: "New Leg Name"
    can_port: 4              # Note: requires additional CAN interface
    num_motors: 4
    motors: [MOTOR1, MOTOR2, MOTOR3, MOTOR4]
    directions: [1, -1, 1, 1]
    zero_position_indices: [2]
    zero_position_offset: 1
```

2. Add corresponding motors to the motors section

### Changing Motor Parameters

Simply edit the YAML file and restart the node. No recompilation needed for:
- Motor IDs
- Directions
- CAN ports
- Zero position offsets
- Motor type assignments

## Build Notes

The node requires:
- `yaml-cpp` library (included in CMakeLists.txt as `yaml`)
- C++17 standard
- ROS roscpp, std_msgs, message_generation
- Motor control headers (motor.h, HSCanT.h, DmCommand.msg, DmState.msg)

## Comparison with rs_node.cpp

Key improvements:

| Aspect | rs_node.cpp | motor_node.cpp |
|--------|------------|----------------|
| Motor config | Hardcoded in C++ | YAML file |
| Leg management | 4 separate control objects | Map-based, scalable |
| CAN handlers | Hardcoded to 4 | Configurable per leg |
| Motor creation | Manual instantiation | Automated from config |
| Command mapping | Hardcoded switch statement | Automated loop |
| Direction handling | Scattered throughout | Centralized, clear |
| Position offsets | Hardcoded ZERO_POS | Configurable per leg |

