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


