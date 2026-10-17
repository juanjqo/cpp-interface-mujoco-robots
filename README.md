# cpp-interface-mujoco-models

MuJoCo models (MJCF and meshes) of the robots of
[cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco). Its robots library embeds them at
build time (it fetches this repository at a pinned commit), so programs that use them need no model
files. The models can also be used on their own, e.g., with MuJoCo's `simulate`.

| Model | Robot | Variants | License |
|---|---|---|---|
| [`unitree_b1`](unitree_b1) | Unitree B1, quadruped | position actuators (`b1.xml`), velocity actuators (`b1_velocity.xml`), torque motors (`b1_torque.xml`) | BSD-3-Clause (Unitree Robotics) |
| [`unitree_z1`](unitree_z1) | Unitree Z1, 6-DoF arm | with or without the gripper; position, velocity or torque actuators (`z1_gripper.xml`, `z1_velocity.xml`, ...) | BSD-3-Clause (Unitree Robotics) |

Each model directory has its own `LICENSE`, which applies to the files in that directory, and a
`README.md` with the source of the model, the changes made to it, and how its meshes were generated.

## Conventions

The models follow the conventions that cpp-interface-mujoco relies on:

- One root body attached to the world (the floating base, for legged robots), and no floor, lights or sky:
  scenes add them.
- A `home` keyframe, with `ctrl`, in which the robot holds its pose (with position actuators).
- Sites at the end effectors (e.g., `end_effector`, the feet), whose poses match the kinematic models of
  the robots library.
- Mesh files in `assets/` (`<compiler meshdir="assets"/>`), light enough to load and render fast: see
  [`tools/decimate_meshes.py`](tools/decimate_meshes.py).

## Tools

- [`tools/decimate_meshes.py`](tools/decimate_meshes.py): decimates OBJ meshes with Blender (headless),
  keeping their coordinates. The command used for each model is in its README.

## License

The files of each model directory are under the license in that directory. The other files of this
repository (the tools and this README) are under the [LGPL-2.1](LICENSE).
