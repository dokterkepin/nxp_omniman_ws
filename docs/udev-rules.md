# Udev Rules

Linux assigns device names (`/dev/ttyUSB0`, `can0`, `/dev/video0`, …) based on detection order, which can change between reboots or reconnects. udev rules create stable symlinks so your config files always point to the right device.

---

## Dynamixel U2D2

the rules file already exists, copy it automatically
```bash
sudo cp ~/workspaces/nxp_omniman_ws/src/dynamixel_hardware/dynamixel_hardware_interface/scripts/99-manipulator-cdc.rules /etc/udev/rules.d/
```

or do it manually — find your device node and attributes
```bash
ls /dev/ttyUSB*
udevadm info -a -n /dev/ttyUSB0
```

create and modify the rules file
```bash
sudo nano /etc/udev/rules.d/99-manipulator-cdc.rules
```

put this inside and save
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT9HD25J" MODE="0666", SYMLINK+="dynamixel"
```

reload and verify
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/dynamixel
```
must show output like: `dynamixel -> ttyUSB0`

---

## RPLidar

the rules file already exists, copy it automatically
```bash
sudo cp ~/workspaces/nxp_omniman_ws/src/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
```

or do it manually — find your device node and attributes
```bash
ls /dev/ttyUSB*
udevadm info -a -n /dev/ttyUSB0
```

create and modify the rules file
```bash
sudo nano /etc/udev/rules.d/rplidar.rules
```

put this inside and save
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0777", SYMLINK+="rplidar"
```

reload and verify
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/rplidar
```
must show output like: `rplidar -> ttyUSB1`

---

## CAN Bus Adapters

plug in **one adapter at a time** and find its serial number
```bash
udevadm info -a -p /sys/class/net/can0 | grep serial
```
repeat for the second adapter — note which serial belongs to wheels and which to the arm

create and modify the rules file
```bash
sudo nano /etc/udev/rules.d/80-can.rules
```

put this inside and save, replacing the serial numbers with yours
```
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="<wheels-serial>", NAME="can_base"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="<arm-serial>", NAME="can_arm"
```

reload and verify
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ip link show can_base && ip link show can_arm
```
must show both interfaces

---

## C920 Webcam

find which video node is the C920 and get its attributes
```bash
v4l2-ctl --list-devices
udevadm info -a -n /dev/video2
```
replace `video2` with whatever node the C920 capture device is on your system

create and modify the rules file
```bash
sudo nano /etc/udev/rules.d/99-camera.rules
```

put this inside and save
```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="08e5", ATTR{index}=="0", SYMLINK+="video_c920", MODE="0666"
```

reload and verify
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux
ls -la /dev/video_c920
```
must show output like: `video_c920 -> video2`
