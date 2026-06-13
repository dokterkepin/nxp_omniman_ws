# SSH & Remote Access

## Teleoperation via ROS2 DDS
in order to teleoperate using joy stick or keyboard, ros2 has a really good feature where we don't 
neccasary to connected the controller in robot body, but instead using the same domain id, we can use different machine. remember to connected either via usb or bluetooth in the different machine in the same domain id and run the node 

### joy stick teleop:
```bash
ros2 run joy_linux joy_linux_node
```

### keyboard teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_stamped -p stamped:="true"
```

## Virtual Display for Headless SSH Sessions
When running GUI applications like RViz2 over SSH on a headless machine
(e.g., ASUS NUC robot PC with no monitor), Qt crashes immediately:

```
qt.qpa.xcb: could not connect to display
This application failed to start because no Qt platform plugin could be initialized.
```

This happens because there is no X display server running on the robot PC,
so `$DISPLAY` is empty and Qt has nowhere to render.

### Solution: Xvfb (X Virtual Framebuffer)

Xvfb creates a fake X display entirely in memory. GUI apps think they have
a real monitor and launch without crashing. No actual screen output is
produced — it only prevents the "no display" error.

### Install Xvfb

```bash
sudo apt install xvfb
```

### Add to `~/.bashrc` on the robot PC

```bash
# Virtual Monitor — activates only when no real display is detected
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    export DISPLAY=:99
    pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1920x1080x24 &
fi
```

### Wayland note

If `QT_QPA_PLATFORM` is set to `wayland` in your environment, Qt will
try the Wayland plugin instead of xcb and fail under Xvfb. Fix by also
adding to `~/.bashrc`:

```bash
export QT_QPA_PLATFORM=xcb
```

---

## Copy files between machines via SSH
Use `rsync` to transfer files from one machine to another over SSH.
It shows progress and can resume if interrupted.

### Pull files from remote to local

```bash
rsync -avz --progress dokterkepin@192.168.51.81:~/ws_moveit2/src/ ~/ws_moveit2/src/
```

### Push files from local to remote

```bash
rsync -avz --progress ~/ws_moveit2/src/ erc-i3@192.168.51.151:~/ws_moveit2/src/
```

---

## Passwordless SSH with Key Pairs

Instead of typing a password every time, you can use SSH keys.

### How it works

Each machine has one key pair:
- **Private key** (`~/.ssh/id_rsa`) — stays on your machine, never share it
- **Public key** (`~/.ssh/id_rsa.pub`) — copy this to any machine you want to access

The remote machine stores your public key in `~/.ssh/authorized_keys`. When you connect, your private key proves your identity without a password ever being sent.

### Setup

**Step 1 — Generate a key pair (if you don't have one):**

```bash
ssh-keygen -t rsa -b 4096
```

Press Enter through all prompts to use defaults.

**Step 2 — Copy your public key to the remote machine:**

```bash
ssh-copy-id erc-i3@192.168.51.151
```

SSH in without a password:

```bash
ssh erc-i3@192.168.51.151
```

### Bidirectional access

SSH works both directions. Each machine needs the other's public key:

```
Laptop → Robot PC:   robot PC's authorized_keys contains laptop's public key
Robot PC → Laptop:   laptop's authorized_keys contains robot PC's public key
```

### GitHub SSH

Same concept. Paste your `id_rsa.pub` into GitHub Settings → SSH Keys. Then push/pull without a password. Verify with:

```bash
ssh -T git@github.com
# Hi <username>! You've successfully authenticated.
```

---

## Remote Access via OpenVPN

Private lab IPs (e.g. `192.168.51.xxx`) are not reachable from outside the lab. OpenVPN creates a virtual tunnel so your home machine appears to be on the lab network.

### How it works

```
Home machine (runs OpenVPN) → tunnel → Lab network → Robot PC
```

Only your **local machine** needs to run OpenVPN. The robot PC does nothing extra — it just sits on the lab network as usual.

### Ubuntu

```bash
sudo openvpn --config client.ovpn
```

Then SSH normally:

```bash
ssh erc-i3@192.168.51.151
```

### Windows

1. Install **OpenVPN GUI** app
2. Import `client.ovpn` into the app
3. Click **Connect**

Then SSH using Windows Terminal (built-in since Windows 10):

```powershell
ssh erc-i3@192.168.51.151
```

The SSH command is the same on both platforms. Only OpenVPN differs (GUI on Windows vs terminal on Ubuntu).