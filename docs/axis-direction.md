# axis_direction: Full Breakdown

This parameter exists because the `mecanum_drive_controller` ros2 controller package totally ignores the URDF axes.

Sources about this problem:
- [DiffDriveController ignores wheels orientation — Issue #597](https://github.com/ros-controls/ros2_controllers/issues/597)
- [One motor runs in wrong direction — ODrive ros2_control Issue #29](https://github.com/Factor-Robotics/odrive_ros2_control/issues/29)

## Step 1 — Declaration with default value
[cybergear_hardware_interface.hpp:101](../cybergear_hardware/cybergear_control/include/cybergear_control/cybergear_hardware_interface.hpp#L101)
```cpp
int axis_direction_ = 1;
```
It's a private int member on CybergearActuator. Default is 1 (no inversion). The only valid values are 1 or -1 — it's used purely as a sign multiplier.

## Step 2 — Parsed from URDF hardware params at configure time
[cybergear_hardware_interface.cpp:49](../cybergear_hardware/cybergear_control/src/cybergear_hardware_interface.cpp#L49)
```cpp
axis_direction_ = std::stoi(info_.hardware_parameters["axis_direction"]);
```
When ros2_control loads the hardware plugin, it calls on_configure(). The info_ struct (from ActuatorInterface) contains every <param> you declared in the URDF <hardware> block.
So std::stoi(info_.hardware_parameters["axis_direction"]) converts the string "1" or "-1" from your xacro into the integer.

## Step 3 — Applied on read() (motor feedback → ros2_control)
[cybergear_hardware_interface.cpp:393-398](../cybergear_hardware/cybergear_control/src/cybergear_hardware_interface.cpp#L393-L398)
```cpp
joint_states_[SIF_POSITION] = axis_direction_ * packet_->parsePosition(feedback->data);
joint_states_[SIF_VELOCITY] = axis_direction_ * packet_->parseVelocity(feedback->data);
joint_states_[SIF_TORQUE]   = axis_direction_ * packet_->parseEffort(feedback->data);
joint_states_[SIF_TEMPERATURE] = packet_->parseTemperature(feedback->data);  // NOT flipped
```
The motor sends raw CAN feedback. packet_->parsePosition/Velocity/Effort() decode the raw bytes into physical values. Then before storing them into joint_states_[] (which ros2_control reads), each value is multiplied by axis_direction_.
So if axis_direction_ = -1, the motor reporting +5 rad/s becomes -5 rad/s from ros2_control's perspective. Temperature is intentionally not flipped — it's always positive and has no directional meaning.

## Step 4 — Applied on write() (ros2_control command → motor)
[cybergear_hardware_interface.cpp:454-456](../cybergear_hardware/cybergear_control/src/cybergear_hardware_interface.cpp#L454-L456)
The same multiplication happens in reverse for every control mode:
- OPERATION mode (line 454-456) — position, velocity, and effort commands all multiplied:
```cpp
param.position = axis_direction_ * joint_commands_[HIF_POSITION];
param.velocity = axis_direction_ * joint_commands_[HIF_VELOCITY];
param.effort   = axis_direction_ * joint_commands_[HIF_EFFORT];
```
- CURRENT mode (line 463):
```cpp
frame = packet_->createCurrentCommand(axis_direction_ * joint_commands_[HIF_CURRENT]);
```
- SPEED mode (line 466):
```cpp
frame = packet_->createVelocityCommand(axis_direction_ * joint_commands_[HIF_VELOCITY]);
```
- POSITION mode (line 469):
```cpp
frame = packet_->createPositionCommand(axis_direction_ * joint_commands_[HIF_POSITION]);
```
So if ros2_control sends +2 rad/s velocity and axis_direction_ = -1, the actual CAN command sent to the motor is -2 rad/s.

## How to use it in the URDF

Pass `axis_direction` when calling the macro in the URDF. The value is either `1` (normal) or `-1` (reversed):

[nxp_omniman.urdf.xacro:128-137](../omniman_ros2_control/description/nxp_omniman.urdf.xacro#L128-L137)
```xml
<!-- Left wheels: mirror-mounted, need inversion -->
<xacro:mecanum_wheel_ros2_control ... device_id="3" axis_direction="-1" joint_name="Front_left_plate_continuous_joint"/>
<xacro:mecanum_wheel_ros2_control ... device_id="1" axis_direction="-1" joint_name="Rear_left_plate_continuous_joint"/>

<!-- Right wheels: normal orientation -->
<xacro:mecanum_wheel_ros2_control ... device_id="4" axis_direction="1" joint_name="Front_right_plate_continuous_joint"/>
<xacro:mecanum_wheel_ros2_control ... device_id="2" axis_direction="1" joint_name="Rear_right_plate_continuous_joint"/>

<!-- Arm joints: always 1, MoveIt handles direction via URDF kinematics -->
<xacro:robstride_ros2_control ... device_id="5" joint_name="shoulder_yaw_joint" axis_direction="1"/>
```

## How it works as a whole
The trick is that the inversion is applied symmetrically on both sides:
```
ros2_control command → × axis_direction_ → CAN to motor (write)
motor CAN feedback   → × axis_direction_ → ros2_control state (read)
```
This means the entire ros2_control stack (controllers, URDF kinematics, Nav2, etc.) sees a consistent positive direction — it never knows the physical motor is spinning the other way.
The sign flip cancels out from the perspective of any higher-level controller. the left-side wheels (-1) are mirror-mounted, so their physical "positive" rotation is opposite to what the kinematics expect.
Multiplying by -1 on both read and write transparently corrects.

> **Note:** `axis_direction` is only relevant for wheel motors. Arm joints always use `1` because MoveIt's kinematic solver handles axis orientation through the URDF model — see [coordinate-frames.md](coordinate-frames.md).

---

## Dynamixel: the same feature, built-in

Dynamixel servos have direction inversion as an official firmware feature called **Drive Mode**
(control table address 10). Bit 0 of the register controls rotation direction:

| `Drive Mode` bit 0 | Direction |
|---|---|
| `0` (default) | CCW — normal |
| `1` | CW — reversed |

It is configured in the xacro via the `driver_mode` parameter, which writes directly to the
motor's control table register at startup:

[dynamixel_motor.ros2_control.xacro:49](../dynamixel_hardware/dynamixel_hardware_interface/ros2_control/dynamixel_motor.ros2_control.xacro#L49)
```xml
<param name="Drive Mode">${driver_mode}</param>
```

To reverse a Dynamixel joint, pass `driver_mode:=1` when calling the macro in the URDF:
```xml
<xacro:dynamixel_ros2_control name="PalmYaw" ... driver_mode:="1" />
```

All three Dynamixel joints on this robot currently use `driver_mode:=0` (no inversion needed).

**`axis_direction` on CyberGear is a manual port of this same concept.** Robstride motors have
no equivalent firmware register, so the sign flip was implemented in software inside the
ros2_control hardware plugin — applied symmetrically on both `read()` and `write()` as described
above. The end result is identical: ros2_control and all higher-level stacks see a consistent
positive direction regardless of how the motor is physically mounted.
