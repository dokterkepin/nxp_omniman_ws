# ROS 2 Coordinate Convention & Motor Direction

## ROS 2 Coordinate Convention (REP 103)

ROS 2 follows **REP 103** which uses the **right-hand rule** for all coordinate frames.

### Body Frame (`base_link`)

```
        Z (up)
        |
        |
        +------ X (forward)
       /
      /
     Y (left)
```

- **X** → forward
- **Y** → left
- **Z** → up

### Rotations

- **Roll** → rotation about X (forward)
- **Pitch** → rotation about Y (left)
- **Yaw** → rotation about Z (up)

Positive rotation follows the right-hand rule: point your right thumb along the positive axis,
your fingers curl in the positive rotation direction.

### Quick Check for `cmd_vel`

- Positive `linear.x` → robot moves **forward**
- Positive `linear.y` → robot moves **left**
- Positive `angular.z` → robot rotates **counter-clockwise** (viewed from above)

If any of these are wrong on the real robot, a motor direction is flipped.

---

## Optical Frame Convention (Cameras / OpenCV)

Cameras do **not** use the REP 103 body convention. Computer vision libraries (OpenCV,
MediaPipe, image pipelines) use the **optical frame convention**, where the axes are defined
from the camera's point of view looking out through the lens:

```
        +------ X (right)
       /|
      / |
     Z  Y (down)
   (forward,
   into scene)
```

- **X** → right (across the image, same as image column `u`)
- **Y** → down (down the image, same as image row `v`)
- **Z** → forward (out of the lens, into the scene = depth)

This is also right-handed, but it is rotated relative to the body convention. The contrast:

| Axis | REP 103 body (`base_link`) | Optical frame (camera) |
|---|---|---|
| X | forward | right |
| Y | left | down |
| Z | up | forward (depth) |

### Why two conventions exist

The optical convention matches how an image is stored: the origin is the top-left pixel, `X`
increases to the right (column `u`), `Y` increases downward (row `v`), and `Z` is depth into the
scene. This makes the pinhole camera projection math clean — a 3D point projects to pixel
coordinates with no axis juggling.

### How ROS bridges the two

ROS keeps **both** frames per camera and connects them with a fixed static transform:

- **`camera_link`** — REP 103 body-style frame (X forward), used for mounting the camera in the
  robot's TF tree
- **`camera_optical_frame`** (or `*_rgb_optical_frame`) — optical convention (Z forward), the
  frame that image data and detections are actually expressed in

The static transform between them is a pure rotation, conventionally:

```bash
# rpy = (-pi/2, 0, -pi/2)  →  rotates body-style axes into optical axes
ros2 run tf2_ros static_transform_publisher \
  0 0 0  -1.5708 0 -1.5708  camera_link  camera_optical_frame
```

### Why this matters for hand teleop

The MediaPipe hand tracker (see [hand-teleop.md](hand-teleop.md)) reports landmarks in **image
space** — `u` to the right, `v` down, and an apparent-size depth proxy. Before those values can
drive the arm, they must be mapped from the optical convention into the robot's `base_link`
(REP 103) frame. That mapping is exactly the X↔Z, Y↔(down/left) swap shown in the table above:
image-right becomes robot-left/right, image-up becomes robot-up, and apparent depth becomes
robot-forward. Getting this swap wrong is the usual cause of an arm that moves "sideways when you
expect forward."

---

## Why the Arm (MoveIt) Doesn't Need `axis_direction` Correction

MoveIt uses a **kinematic solver** (KDL, IKFast, etc.) that works entirely in the URDF's
mathematical model. It reads the `<axis xyz="..."/>` definition on each joint and the current
joint position from `/joint_states` to compute trajectories.

It does **not** care about the physical motor spin direction. Even if some motors are mounted in
opposite orientations (one joint reads positive going clockwise, another reads positive going
counter-clockwise), the solver handles it — because the URDF axis definition already encodes
which direction is "positive" in the kinematic chain.

In practice on this robot, the raw `/joint_states` values appear "random" in sign — some joints
report positive when rotating to the left, others when rotating to the right. This is perfectly
fine. MoveIt resolves all of this from the URDF model and produces correct trajectories regardless.

**We do NOT need `axis_direction` correction on arm joints.** We keep it at `1` for all arm motors.

---

## Why the Mecanum Controller DOES Need `axis_direction` Correction

The `mecanum_drive_controller` is fundamentally different. It performs **open-loop velocity math**:

```
wheel_velocity = f(linear.x, linear.y, angular.z, wheel_position)
```

There is no URDF-based kinematic solver in the loop. The controller computes wheel velocities
using inverse kinematics formula like:

```
front_left_vel  = (1/R) * (vx - vy - (lx+ly)*wz)
front_right_vel = (1/R) * (vx + vy + (lx+ly)*wz)
rear_left_vel   = (1/R) * (vx + vy - (lx+ly)*wz)
rear_right_vel  = (1/R) * (vx - vy + (lx+ly)*wz)
```

When you send `cmd_vel` with `linear.x = 1.0` (move forward), the controller computes a
**positive velocity** for all four wheels.

But here's the problem: the front-left motor and front-right motor are mounted as **mirror images**
of each other. If you send the same positive command to both motors, they both spin clockwise —
but because they face opposite directions, one wheel pushes forward and the other pushes
**backward**. The robot would spin instead of driving straight.

The controller doesn't know about motor mounting. It just outputs numbers from the formula.
It assumes that when it says `+1.0` to a wheel, that wheel contributes to forward motion.
But physically that's only true if the motor happens to be mounted the right way.

See [axis-direction.md](axis-direction.md) for the full implementation breakdown.