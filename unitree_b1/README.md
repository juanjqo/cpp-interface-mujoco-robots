# Unitree B1 (MJCF)

The MuJoCo model of the [B1 quadruped](https://www.unitree.com/b1/) of
[Unitree Robotics](https://www.unitree.com/), used by `UnitreeB1Robot::description()` of
[cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco).

| File | Contents |
|---|---|
| `b1.xml` | The B1 with **position actuators**: `ctrl` is the joint position, tracked with a stiffness `kp = 1000` and a damping `kv = 30`, within the torque limits of the joints. The `home` keyframe stands. |
| `b1_velocity.xml` | The B1 with **velocity actuators**: `ctrl` is the joint velocity, within the velocity limits of the URDF (19.69, 23.32 and 15.55 rad/s), tracked with a gain `kv = 300`, within the torque limits of the joints. With zero velocities, it sinks slowly. |
| `b1_torque.xml` | The B1 with **torque motors**: `ctrl` is the joint torque, within the torque limits of the joints. Without a controller, it falls. |
| `b1_robot.xml` | The robot itself, without actuators, included by all three. |
| `scene.xml` | `b1.xml` on a floor, to view the model (e.g., with MuJoCo's `simulate`). |
| `assets/` | The meshes (OBJ). |

## Names

- Floating base: the body `base`, with an `imu` site at its origin.
- Joints, in the order of the actuators: `FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint`, then `FR_`, `RL_`
  and `RR_` (front/rear, left/right). The hip joints rotate about the x axis of the base (abduction), and
  the thigh and calf joints about its y axis.
- Actuators: `FL_hip`, `FL_thigh`, `FL_calf`, ..., one per joint.
- Feet: the sites `FL_foot`, `FR_foot`, `RL_foot`, `RR_foot`, at the centers of the foot spheres.
- Keyframe `home`: standing, each leg at (hip, thigh, calf) = (0, 0.7, -1.2), with the base at a height
  of 0.615 m (the feet on the floor).

Torque limits (N·m): 91.0035 (hip), 93.33 (thigh), 140 (calf). Joint ranges (rad): [-0.75, 0.75] (hip),
[-1.0, 3.5] (thigh), [-2.6, -0.6] (calf). Total mass: 62.57 kg.

## Source

- The kinematic and inertial values, the joint ranges, the torque limits and the collision shapes come
  from the URDF of [unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/b1_description)
  (`xacro/const.xacro` and `xacro/leg.xacro`), commit `5994d4faef0a9cadd3287f8de0199a67eeb2a259`.
- The meshes and the first version of the model come from
  [unitree_description](https://github.com/juanjqo/unitree_description/tree/main/unitree_b1), commit
  `adceb14be2289a136637974de00c30c5ddf5e806`:
  - `hipb.obj`, `thighb.obj`, `thigh_mirrorb.obj` and `calfb.obj`: converted from the DAE meshes of
    unitree_ros (Unitree Robotics).
  - The other meshes: the trunk and its parts and accessories.

## Changes from unitree_description

- Inertias: the full inertia matrices of the URDF (`fullinertia`), with its products of inertia, instead
  of their diagonals; the center of mass of the rear hips at z = +0.000109, as in the URDF.
- The 12 rotor links of the URDF (fixed to the trunk, hips and thighs, 2.92 kg in total), as bodies without
  joints.
- Actuators: position actuators (`b1.xml`), velocity actuators (`b1_velocity.xml`) and torque motors
  (`b1_torque.xml`), all within the torque limits of the URDF (`actuatorfrcrange` of the joints).
- Keyframe `home`: standing on the floor (the base was 1.5 m above it), with `ctrl`.
- Sites `FL_foot`, `FR_foot`, `RL_foot`, `RR_foot` at the feet.
- Collision geometries in group 3 (hidden by default), as in MuJoCo Menagerie.
- The meshes decimated from 416k to 86k triangles (36 MB to 5.5 MB): the robot renders 157k triangles
  instead of 760k, and compiles in 23 ms instead of 102 ms. Unused meshes removed.

The meshes were generated with (from the root of this repository, with Blender 5.2):

```bash
blender -b --factory-startup --python tools/decimate_meshes.py -- \
    <unitree_description>/unitree_b1/assets unitree_b1/assets --smooth 60 \
    trunk_new_head_mask.obj=6000 trunk_new_base.obj=10000 thighb.obj=12000 thigh_mirrorb.obj=12000 \
    hipb.obj=10000 calfb.obj=6000 covers.obj=5000 trunk_new_head_without_mask.obj=4000 \
    z1_brackets.obj=3000 battery.obj=3000 trunk_decorator.obj=3000 rubber_pads.obj=2000 \
    expansion_box.obj=2000 --copy mount_frames.obj supports.obj eyes.obj battery_leds.obj
```

## License

These files are released under the [BSD-3-Clause License](LICENSE), Copyright (c) 2016-2022 HangZhou
YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics").
