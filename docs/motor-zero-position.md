# Setting Zero Position via SocketCAN in Ubuntu

## Why SocketCAN?

The official way to configure CyberGear/RobStride motors (set zero position, change IDs, calibrate, etc.) is through the [MotorStudio](https://github.com/RobStride/MotorStudio) desktop app. However, MotorStudio has two major limitations:

1. **Windows only** — it does not run on Ubuntu/Linux, where most robotics projects (ROS2, Isaac Sim, etc.) are developed.
2. **Requires the official RobStride USB-C CAN adapter** — it does not recognize third-party USB-CAN hardware.

Since most robot setups use generic USB-CAN adapters (e.g. STM32-based USB-CAN modules) with Linux's **SocketCAN** interface (`can0`), MotorStudio is simply not usable in these environments.

The solution is to send CAN commands directly from Ubuntu using the `cybergear_socketcan_driver` package, which communicates with the motors over SocketCAN — no Windows or proprietary hardware needed.

## What is ros2_socketcan?

The `ros2_socketcan` package provides ROS2 lifecycle nodes that bridge between Linux SocketCAN and the ROS2 ecosystem. It consists of two nodes:

- **`socket_can_sender_node`** — publishes CAN frames from ROS2 topics to the CAN bus.
- **`socket_can_receiver_node`** — receives CAN frames from the CAN bus and publishes them as ROS2 topics.

The `cybergear_socketcan_driver` depends on `ros2_socketcan` to send and receive CAN frames to/from the CyberGear motors. These lifecycle nodes must be configured and activated before the driver can communicate with the hardware.

## How to Set Zero Position

### 1. Bring up SocketCAN interface

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 2. Launch and activate ros2_socketcan nodes

Run the sender and receiver nodes:

```bash
ros2 run ros2_socketcan socket_can_sender_node_exe --ros-args -p interface:=can0
ros2 run ros2_socketcan socket_can_receiver_node_exe --ros-args -p interface:=can0
```

Then configure and activate both lifecycle nodes:

```bash
ros2 lifecycle set /socket_can_sender_node configure
ros2 lifecycle set /socket_can_sender_node activate
ros2 lifecycle set /socket_can_receiver_node configure
ros2 lifecycle set /socket_can_receiver_node activate
```

Verify both are active:

```bash
ros2 lifecycle get /socket_can_sender_node
ros2 lifecycle get /socket_can_receiver_node
# Both should show: active [3]
```

### 3. Launch the cybergear position driver

```bash
ros2 run cybergear_socketcan_driver cybergear_position_driver_node \
  --ros-args -p primary_id:=5 -p device_id:=20 -p joint_name:=shoulder_yaw_joint
```

Adjust `primary_id`, `device_id`, and `joint_name` to match your motor configuration.

### 4. Disable torque and set zero position

First disable torque (required before setting zero position otherwise the motor will move and very dangerous:

```bash
ros2 service call /cybergear_position_driver/enable_torque std_srvs/srv/SetBool "{data: false}"
```

Then set the current position as the new zero:

```bash
ros2 service call /cybergear_position_driver/zero_position std_srvs/srv/Trigger "{}"
```
