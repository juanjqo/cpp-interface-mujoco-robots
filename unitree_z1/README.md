# Unitree Z1 (MJCF)

The MuJoCo model of the [Z1 manipulator](https://www.unitree.com/z1/) of
[Unitree Robotics](https://www.unitree.com/), with and without its gripper, used by
`UnitreeZ1Robot::description()` of [cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco).

| Actuators of the arm | Without the gripper | With the gripper |
|---|---|---|
| **Position**: `ctrl` is the joint position (gain 1000, 1500 for joint2) | `z1.xml` | `z1_gripper.xml` |
| **Velocity**: `ctrl` is the joint velocity, within 3.1415 rad/s, tracked with a gain `kv = 30`. With zero velocities, the arm sinks slowly. | `z1_velocity.xml` | `z1_gripper_velocity.xml` |
| **Torque**: `ctrl` is the joint torque. Without a controller, the arm falls. | `z1_torque.xml` | `z1_gripper_torque.xml` |
| The robot, without actuators, included by the files above | `z1_robot.xml` | `z1_gripper_robot.xml` |

All of them are within the torque limits of the joints: 30 N·m (60 N·m for joint2). The gripper always has
a position actuator (`ctrl` from -1.5, open, to 0, closed), whatever the actuators of the arm. The `home`
keyframe holds the pose with position actuators, and has zero velocities or torques with the others.

## Source

Copied from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_z1),
commit `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35`: `z1.xml`, `z1_gripper.xml`, `assets/` and
`LICENSE`. The Menagerie derived it from the
[URDF of Unitree](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/z1_description).

## Changes

- Added the site `end_effector` to `link06`, at the flange (without the gripper) or at the tip of the
  gripper, with its z axis along the last joint, so that its pose is the forward kinematics of
  `UnitreeZ1Robot::kinematics()`.
- Added `armature="0.01"` to the joints (the inertia of the geared motors). Without it, with the timestep
  of 0.002 s, the stiff position actuators (gain 1000) of the light joints chatter instead of moving:
  joint6, joint5 without the gripper, and jointGripper, which never opened.
- Split each model into the robot (`z1_robot.xml`, `z1_gripper_robot.xml`) and its actuators, and added
  the velocity and torque variants, with the velocity limit (`velocityMax`) and the torque limits
  (`torqueMax`) of the URDF of Unitree. The position variants (`z1.xml`, `z1_gripper.xml`) have the
  actuators of the Menagerie.

## License

These files are released under the [BSD-3-Clause License](LICENSE), Copyright (c) 2016-2022
HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"). This applies to the files in this
directory only.
