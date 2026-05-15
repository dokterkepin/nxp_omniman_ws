# MoveIt Notes

## SRDF End-Effector Definition

In the SRDF file, the end-effector is defined as:

```xml
<end_effector name="ee" parent_link="ee_link" group="arm"/>
```

The `parent_link` here does **not** mean "the parent of the end-effector" in the URDF kinematic chain sense. It means: **which link on the arm group should MoveIt treat as the tip/reference point for IK and motion planning.**

So `parent_link="ee_link"` tells MoveIt: "when planning for the `arm` group, solve IK so that `ee_link` reaches the target pose." It is the **tip link** of the arm — the point you want to position in space.

## Joint Limits Integer vs Float Issue

After running MoveIt Setup Assistant, the generated `joint_limits.yaml` file may contain integer values (e.g., `1` or `0`) instead of floats for fields like `max_velocity`, `max_acceleration`, etc. This causes MoveIt to crash on launch with a type error, because the joint limits parser expects floating-point values.

**Fix:** Change all integer values to floats by adding a decimal point:
```yaml
shoulder_yaw_joint:
  has_velocity_limits: true
  max_velocity: 1.0
  has_acceleration_limits: true
  max_acceleration: 0.0
```

This is a known issue. The Setup Assistant writes YAML values without a decimal point, and the MoveIt joint limits loader uses `declare_parameter<double>()` which rejects plain integers. The fix on the MoveIt side was merged in [moveit/moveit2#2335](https://github.com/moveit/moveit2/pull/2335), but depending on your MoveIt version (especially Humble), you may still hit this. The simplest workaround is to manually ensure all numeric values in `joint_limits.yaml` are floats.

## Problem with kinematics.yaml and some configuration

Make sure we never touch or modify the kinematics.yaml regarless what problem are, if the kinematics solver, controllers, or even srdf is not correct, just create new moveit package and delete the old one. i realize that modifying the existing values or any parameters will break the function and resulting in so much unexpected error that you might won't be able to fix until you recreate the package.

do not modify the existing configuration that we have been create in moveit setup assistant, like controller, group, collision, we should delete and recreate if we found something that we create wrong. also always manually create the  controllers, don't use automation because i realize it will occur some problem.
