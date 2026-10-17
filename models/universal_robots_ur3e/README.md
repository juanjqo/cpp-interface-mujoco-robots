# Universal Robots UR3e (MJCF)

The MuJoCo model of the [UR3e](https://www.universal-robots.com/products/ur3e/) collaborative arm of
[Universal Robots](https://www.universal-robots.com/), used by `UR3eRobot::description()` of this package.

| File | Contents |
|---|---|
| `ur3e.xml` | The UR3e with **position actuators**: `ctrl` is the joint position, tracked with a stiffness `kp` (2000, 1000 and 500 N·m/rad for the shoulder, elbow and wrist joints) and a damping `kv = kp/20`. |
| `ur3e_velocity.xml` | The UR3e with **velocity actuators**: `ctrl` is the joint velocity, within the velocity limits of the URDF (π rad/s for the shoulder and elbow joints, 2π rad/s for the wrist joints), tracked with a gain `kv` (200, 100 and 20 N·m per rad/s). With zero velocities, the arm sinks slowly. |
| `ur3e_torque.xml` | The UR3e with **torque motors**: `ctrl` is the joint torque. Without a controller, the arm falls. |
| `ur3e_robot.xml` | The robot itself, without actuators, included by all three. |
| `scene.xml` | `ur3e.xml` on a floor, to view the model (e.g., with MuJoCo's `simulate`). |
| `assets/` | The meshes (OBJ), one per material of each link. |

All of them are within the torque limits of the joints (`actuatorfrcrange`): 54 N·m (shoulder joints), 28 N·m
(elbow) and 9 N·m (wrist joints). The `home` keyframe holds the pose with position actuators, and has zero
velocities or torques with the others.

## Names

- Root body: `base_link`, fixed, at the origin of the robot, with its x axis forward (as `base_link` in ROS).
- Bodies: `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`, with
  the frames of the URDF.
- Joints, in the order of the actuators: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
  `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`, with the zero configuration of the robot: at q = 0 the arm is
  stretched forward, horizontally. Ranges: [-2π, 2π] rad, [-π, π] rad for the elbow (as in the URDF, which limits it
  to half of its range to avoid planning problems), and unlimited for `wrist_3_joint` (continuous in the URDF).
- Actuators: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`.
- End effector: the site `end_effector`, at the center of the flange, with its z axis out of it. It is the frame 6
  of the DH parameters of `UR3eRobot::kinematics()`, which is `tool0` of ROS (the frame of `wrist_3_link`) rotated
  by π about its z axis.
- Keyframe `home`: q = (0, -π/2, π/2, -π/2, -π/2, 0): the upper arm up, the forearm forward, and the flange
  pointing down.

Total mass: 10.80 kg (2 kg of `base_link`, as in the URDF).

## Source

Made from the ROS 2 description of Universal Robots,
[Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), commit
`d4042c1f5123c86645cc5dbd650b4f461f697ee9`, whose UR3e files are under the BSD-3-Clause license (`LICENSE`, copied
from it):

- The kinematics (`config/ur3e/default_kinematics.yaml`), the masses, centers of mass and inertias
  (`physical_parameters.yaml`), the joint ranges, torque limits and velocity limits (`joint_limits.yaml`) and the
  poses of the meshes (`visual_parameters.yaml`), from the URDF generated with `xacro` from `urdf/ur.urdf.xacro`
  (`ur_type:=ur3e`). MuJoCo compiled that URDF into the bodies, joints and inertias of `ur3e_robot.xml`.
- The visual meshes (`meshes/ur3e/visual/*.dae`), converted to OBJ, one file per material, and decimated (see
  below). The colors of the materials are those of the DAE files.

## Changes from the URDF

- `base_link_inertia` (fixed to `base_link`, rotated by π about z) merged into `base_link`: the shoulder is rotated
  by π in `base_link`, and the meshes of the base are not rotated.
- Collision geometries designed by hand (cylinders and capsules, in group 3, hidden by default), fitted to the
  collision meshes of the URDF, as in MuJoCo Menagerie. There is no contact between the links at the `home`
  keyframe.
- `armature="0.1"` on the joints (the inertia of the motors through their gears, not in the URDF), as in the UR5e
  of MuJoCo Menagerie.
- Actuators: position actuators (`ur3e.xml`), velocity actuators (`ur3e_velocity.xml`) and torque motors
  (`ur3e_torque.xml`), all within the torque limits of the URDF.
- The site `end_effector`, and the keyframe `home`.
- The visual meshes decimated from 132k to 48k triangles (the largest ones, the others only with normals).

The meshes were generated with (from the root of this repository, with Blender 5.2, and pycollada in a virtual
environment: Blender 5 cannot import DAE files):

```bash
python3 tools/dae_to_obj.py <ur3e_raw> <Universal_Robots_ROS2_Description>/meshes/ur3e/visual/*.dae
blender -b --factory-startup --python tools/decimate_meshes.py -- \
    <ur3e_raw> models/universal_robots_ur3e/assets --smooth 45 \
    shoulder_urblue.obj=5000 upperarm_urblue.obj=6000 upperarm_jointgrey.obj=4000 wrist1_urblue.obj=3500 \
    wrist2_urblue.obj=3500 forearm_urblue.obj=3000 wrist2_jointgrey.obj=2500 forearm_jointgrey.obj=2500 \
    wrist1_jointgrey.obj=2500 base_black.obj=870 base_jointgrey.obj=2436 forearm_black.obj=2038 \
    forearm_linkgrey.obj=585 shoulder_black.obj=852 shoulder_jointgrey.obj=1083 upperarm_black.obj=1994 \
    upperarm_linkgrey.obj=2011 wrist1_black.obj=1594 wrist2_black.obj=1452 wrist3_linkgrey.obj=1127
```

## License

These files are released under the [BSD-3-Clause License](LICENSE) of Universal_Robots_ROS2_Description, whose
contributors are the copyright holders. This applies to the files in this directory only.

Universal Robots releases the meshes of some of its other robots (UR8 Long, UR15, UR18, UR20 and UR30) under its
"Terms and Conditions for Use of Graphical Documentation" instead, which are not an open-source license; the UR3e
is not one of them.
