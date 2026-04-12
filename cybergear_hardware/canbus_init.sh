# can0 — mecanum wheels
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 1000

# can1 — arm CyberGear motors
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
sudo ifconfig can1 txqueuelen 1000
