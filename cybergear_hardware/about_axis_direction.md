# the full breakdown of how axis_direction is built into the low-level control
This function is created due to the mecanum_drive_controller ros2 controller package are totally ignores the URDF axes.
sources about this problem:
+ [DiffDriveController ignores wheels orientation — Issue #597](https://github.com/ros-controls/ros2_controllers/issues/597)
+ [One motor runs in wrong direction — ODrive ros2_control Issue #29](https://github.com/Factor-Robotics/odrive_ros2_control/issues/29)

# Step 1 — Declaration with default value
[cybergear_hardware_interface.hpp:101](cybergear_control/include/cybergear_control/cybergear_hardware_interface.hpp#L101)
```bash
int axis_direction_ = 1;
```
It's a private int member on CybergearActuator. Default is 1 (no inversion). The only valid values are 1 or -1 — it's used purely as a sign multiplier.

# Step 2 — Parsed from URDF hardware params at configure time
[cybergear_hardware_interface.cpp:49](cybergear_control/src/cybergear_hardware_interface.cpp#L49)
```bash
axis_direction_ = std::stoi(info_.hardware_parameters["axis_direction"]);
```
When ros2_control loads the hardware plugin, it calls on_configure(). The info_ struct (from ActuatorInterface) contains every <param> you declared in the URDF <hardware> block. 
So std::stoi(info_.hardware_parameters["axis_direction"]) converts the string "1" or "-1" from your xacro into the integer.

# Step 3 — Applied on read() (motor feedback → ros2_control)
[cybergear_hardware_interface.cpp:393-398](cybergear_control/src/cybergear_hardware_interface.cpp#L393-L398)
```bash
joint_states_[SIF_POSITION] = axis_direction_ * packet_->parsePosition(feedback->data);
joint_states_[SIF_VELOCITY] = axis_direction_ * packet_->parseVelocity(feedback->data);
joint_states_[SIF_TORQUE]   = axis_direction_ * packet_->parseEffort(feedback->data);
joint_states_[SIF_TEMPERATURE] = packet_->parseTemperature(feedback->data);  // NOT flipped
```
The motor sends raw CAN feedback. packet_->parsePosition/Velocity/Effort() decode the raw bytes into physical values. Then before storing them into joint_states_[] (which ros2_control reads), each value is multiplied by axis_direction_.
So if axis_direction_ = -1, the motor reporting +5 rad/s becomes -5 rad/s from ros2_control's perspective. Temperature is intentionally not flipped — it's always positive and has no directional meaning.

# Step 4 — Applied on write() (ros2_control command → motor)
[cybergear_hardware_interface.cpp:454-456](cybergear_control/src/cybergear_hardware_interface.cpp#L454-L456)
The same multiplication happens in reverse for every control mode:
- OPERATION mode (line 454-456) — position, velocity, and effort commands all multiplied:
```bash
param.position = axis_direction_ * joint_commands_[HIF_POSITION];
param.velocity = axis_direction_ * joint_commands_[HIF_VELOCITY];
param.effort   = axis_direction_ * joint_commands_[HIF_EFFORT];
```
- CURRENT mode (line 463):
```bash
frame = packet_->createCurrentCommand(axis_direction_ * joint_commands_[HIF_CURRENT]);
```
- SPEED mode (line 466):
```bash
frame = packet_->createVelocityCommand(axis_direction_ * joint_commands_[HIF_VELOCITY]);
```
- POSITION mode (line 469):
```bash
frame = packet_->createPositionCommand(axis_direction_ * joint_commands_[HIF_POSITION]);
```
So if ros2_control sends +2 rad/s velocity and axis_direction_ = -1, the actual CAN command sent to the motor is -2 rad/s.

# How it works as a whole
The trick is that the inversion is applied symmetrically on both sides:
```bash
ros2_control command → × axis_direction_ → CAN to motor (write)
motor CAN feedback   → × axis_direction_ → ros2_control state (read)
```
This means the entire ros2_control stack (controllers, URDF kinematics, Nav2, etc.) sees a consistent positive direction — it never knows the physical motor is spinning the other way. 
The sign flip cancels out from the perspective of any higher-level controller. the left-side wheels (-1) are mirror-mounted, so their physical "positive" rotation is opposite to what the kinematics expect. 
Multiplying by -1 on both read and write transparently corrects.

