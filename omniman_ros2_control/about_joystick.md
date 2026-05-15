# Joystick Control for OmniMan

## Wireless Multi-Machine Control (No Cables!)

You can control the robot entirely wirelessly by splitting the joystick and robot control across two PCs, as long as both share the same `ROS_DOMAIN_ID`:

- **Robot PC**: Run the main launch with joystick enabled
  ```bash
  ros2 launch omniman_ros2_control omniman.launch.py use_joy:=true
  ```
- **Remote PC**: Connect Xbox controller via Bluetooth and run
  ```bash
  ros2 run joy joy_node
  ```

The `joy_node` on the remote PC publishes `/joy` topics over DDS, and the robot PC picks them up transparently — no USB cable, no physical tether to the robot.

---

## The Problem: Bluetooth vs USB Changes Axis/Button Indices

When connecting an Xbox controller, the **axis and button indices shift** depending on whether the controller is connected via **USB cable** or **Bluetooth**. This means a configuration that works perfectly over USB will produce wrong or no movement over Bluetooth (and vice versa).

This is a kernel/driver-level difference — the Xbox driver maps differently depending on the transport.

---

## Checking Indices with `jstest`

`jstest` is a very useful tool for identifying the correct axis and button indices for your controller in its current connection mode:

```bash
sudo apt install joystick    # install if not available
jstest /dev/input/js0        # test the first joystick device
```

When you run `jstest`, move sticks and press buttons — it will show you in real-time which axis/button index corresponds to each physical input. Use this to verify your configuration matches your connection method.

---

## Solution: Dual Configuration in `joystick.yaml`

The solution is to maintain **two configurations** in `config/joystick.yaml` — one for USB and one for Bluetooth. 
