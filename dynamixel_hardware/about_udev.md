# Dynamixel Udev Rules

we should follow this tutorial from official dynamixel manual for open manipulator: 
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/

## Problem

When multiple USB devices are connected (e.g., Dynamixel U2D2 adapter and RPLidar), Linux assigns `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc. based on the order they are detected. This order can change between boots or when devices are reconnected, so a device that was `/dev/ttyUSB0` yesterday might become `/dev/ttyUSB1` today.

This causes problems because our launch files and config files reference a specific port (e.g., `/dev/ttyUSB0`). If the assignment shifts, the wrong device gets opened.

## Solution: Udev Symlink

The file `dynamixel_hardware_interface/scripts/99-manipulator-cdc.rules` creates a persistent symlink so the Dynamixel U2D2 adapter is always accessible at `/dev/dynamixel`, regardless of which `ttyUSB` number it gets assigned.

The rule:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", SYMLINK+="dynamixel"
```
After this rule is installed, you can use `/dev/dynamixel` in your config files and it will always point to the correct device.

## How to Install

1. Copy the rules file to the udev rules directory:

```bash
sudo cp ~/workspaces/nxp_omniman_ws/src/dynamixel_hardware/dynamixel_hardware_interface/scripts/99-manipulator-cdc.rules /etc/udev/rules.d/
```

2. Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. Verify the symlink exists (with the U2D2 plugged in):

```bash
ls -l /dev/dynamixel
```

You should see something like `dynamixel -> ttyUSB0`.

> **Note:** There is also a helper script `create_udev_rules` that does steps 1-2 automatically via `ros2 run dynamixel_hardware_interface create_udev_rules`, but that requires the workspace to be built and sourced first.

## How to Check Device Attributes Before Writing a Rule

Before writing or updating a udev rule, verify the actual device attributes first so the rule matches the correct device.

**1. List connected USB TTY devices:**

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

**2. Get the full attribute tree for a specific device:**

```bash
udevadm info -a -n /dev/ttyUSB0
```

This prints all attributes you can match against: `idVendor`, `idProduct`, `serial`, `manufacturer`, `product`, etc.

**3. Filter for the most useful fields:**

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial|manufacturer|product'
