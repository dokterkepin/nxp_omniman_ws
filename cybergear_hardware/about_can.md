# Assigning stable CAN bus names with udev rules
By default Linux names USB-CAN adapters `can0`, `can1` in random order depending on which one the USB subsystem sees first. After a reboot or re-plug, they can swap — wheels talk to the arm bus and vice versa. udev rules fix this by matching each adapter's unique serial number and assigning a stable name (`can_base`, `can_arm`).

## Step 1 — Find the serial number of each adapter
Plug in **only one** USB-CAN adapter (e.g. the wheels one). Then run:
```bash
udevadm info -a -p /sys/class/net/can0 | grep serial
```
The serial number is an attribute of the parent USB device, not the network interface itself, so you need `-a` to walk up the device tree. Look for the `ATTRS{serial}` line — that's the unique ID. Write it down and label it (e.g. "wheels").
Unplug it, plug in the **other** adapter, and repeat the same command to get the second serial number.

To know which adapter is which when both show as `can0`/`can1`, use `candump can0` and move one motor by hand — whichever bus shows traffic is the one connected to that adapter.

## Step 2 — Create the udev rules file
```bash
sudo nano /etc/udev/rules.d/80-can.rules
```
Write two lines, substituting your serial numbers:
```bash
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="003D001F5434570120333535", NAME="can_base"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="002D00355642571820363533", NAME="can_arm"
```
This tells udev: when a network device appears with this serial, name it `can_base` or `can_arm` instead of `can0`/`can1`.

## Step 3 — Reload udev rules
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Step 4 — Verify
Unplug and re-plug both adapters (or reboot), then check:
```bash
ip link show can_base
ip link show can_arm
```
Both should appear with the correct names. The order you plug them in no longer matters.

---

# This explain about can bus setup before launching ros2_control
Without running this script, the CAN interfaces exist but are **down and unconfigured**. The ros2_control hardware plugin (`CybergearActuator`) will fail to open the socket because the kernel doesn't know what bitrate to use.

# Boot sequence
1. **Udev rule** detects USB-CAN adapter → creates and names the interface (`can_base`, `can_arm`)
2. **This script** (`canbus_init.sh`) configures the bitrate and brings the interface up
3. **ros2_control** connects to the named interface and starts CAN communication with motors

# What each line does
[canbus_init.sh](canbus_init.sh)
```bash
ip link set can_base type can bitrate 1000000
ip link set can_base txqueuelen 1000
ip link set can_base up
```

| Command | Purpose |
|---|---|
| `type can bitrate 1000000` | Set CAN bus bitrate to 1 Mbps. This is the **factory-fixed baud rate** in the CyberGear/Robstride motor firmware — it is not configurable on the motor side. If the Linux interface uses any other bitrate, the motor will not respond. |
| `txqueuelen 1000` | Increase the outgoing frame buffer from the default (~10) to 1000. Prevents dropped commands when controlling multiple motors on the same bus at high update rates. |
| `up` | Bring the interface online. Equivalent to plugging in a network cable — without this, the interface exists but cannot send or receive frames. |

The same three commands are repeated for `can_arm` (the second CAN bus for arm motors).

# How the interface names connect to ros2_control
In the URDF [nxp_omniman.urdf.xacro:13-14](../omniman_ros2_control/description/nxp_omniman.urdf.xacro#L13-L14):
```xml
<xacro:property name="canbus_base" default="can_base"/>
<xacro:property name="canbus_arm" default="can_arm"/>
```
These names must match exactly what this script configures. `can_base` is for the 4 mecanum wheel motors (device_id 1-4), `can_arm` is for the 4 arm joint motors (device_id 5-8).

# How to run
Run with `sudo` before launching ros2_control:
```bash
sudo bash canbus_init.sh
```
This must be done **after every boot** — CAN interface configuration does not persist across reboots.
make sure the led in hardware is bright after running this file

To verify the interfaces are up:
```bash
ip link show can_base
ip link show can_arm
```
Both should show `state UP` and `qlen 1000`.
