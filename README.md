# cpp-interface-mujoco-robots

Models of real robots for [cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco), an unofficial
interface for C++17 between [DQ Robotics](https://dqrobotics.github.io) and [MuJoCo](https://mujoco.org). Each robot
is a class that gives:

- `description()`: its MuJoCo model, embedded in the library (no model files to ship or find at runtime), and the
  names needed to use it (joints, end effectors, floating base, home keyframe), as a `DQ_MujocoRobotDescription`.
- `kinematics()`: its DQ Robotics kinematic model, which matches the MuJoCo model (the tests check that `fkm(q)`
  equals the pose of the end-effector sites).

The models (MJCF and meshes) are in [`models`](models), next to the classes, and can also be used on their own (e.g.,
with MuJoCo's `simulate`).

| Robot | Class | Variants | Model |
|---|---|---|---|
| Unitree B1, quadruped | [`UnitreeB1Robot`](include/dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h) | `ACTUATORS::POSITION` (default), `VELOCITY`, `TORQUE`. Sensors: an IMU (`imu_site`) and touch sensors at the feet. `arm_mount()`: where a Z1 is mounted on its back | [`models/unitree_b1`](models/unitree_b1) |
| Unitree Z1, 6-DoF arm | [`UnitreeZ1Robot`](include/dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h) | `GRIPPER::WITH` (default), `WITHOUT`; `ACTUATORS::POSITION` (default), `VELOCITY`, `TORQUE` for the arm (the gripper always has a position actuator) | [`models/unitree_z1`](models/unitree_z1) |

## Install

Install [cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco) first. Then, from the root of this
repository:

```shell
# 1. Configure. If cpp-interface-mujoco is not installed in a system path, point to it with
#    -DCMAKE_PREFIX_PATH=<its install prefix> (e.g., $HOME/opt).
#    To build only the library, add -DDQ_MUJOCO_ROBOTS_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_ROBOTS_BUILD_TESTS=OFF
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# 2. Build the library, examples and tests.
cmake --build build -j$(nproc)

# 3. (Optional) Run the tests.
ctest --test-dir build --output-on-failure

# 4. Install (without sudo, configure with -DCMAKE_INSTALL_PREFIX=$HOME/opt).
sudo cmake --install build
sudo ldconfig
```

## Usage

```cmake
find_package(dqrobotics-interface-mujoco-robots REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE dqrobotics_extensions::interface-mujoco-robots)  # also links the interface
```

```cpp
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include <dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;

const DQ_MujocoRobotDescription z1 = UnitreeZ1Robot::description();  // with the gripper, position actuators
DQ_MujocoInterface mi;
mi.load_model(z1.model);
mi.reset_simulation(z1.home_keyframe);

DQ_SerialManipulatorDH arm = UnitreeZ1Robot::kinematics();
const VectorXd q = mi.get_joint_positions(z1.joint_names);
const DQ x = arm.fkm(q);                                             // = mi.get_site_pose("end_effector")
```

The robots can also be placed in scenes with `DQ_MujocoSceneBuilder::add_robot()` (see cpp-interface-mujoco), several
times and with any of their variants: their meshes are stored once. A robot can also be mounted on another one, e.g.,
a Z1 on the back of a B1, as in the Unitree B1Z1:

```cpp
DQ_MujocoSceneBuilder scene;
scene.add_floor();
const auto b1 = scene.add_robot(UnitreeB1Robot::description(), "b1_");
const auto z1 = scene.add_robot(UnitreeZ1Robot::description(), "z1_", UnitreeB1Robot::arm_mount(), b1.floating_base);
mi.load_model(scene.build());

// The kinematics of the Z1, carried by the base of the B1
DQ_SerialManipulatorDH arm = UnitreeZ1Robot::kinematics();
const DQ arm_reference = UnitreeB1Robot::arm_mount()*arm.get_reference_frame();
arm.set_reference_frame(mi.get_base_pose(b1.floating_base)*arm_reference);  // update it as the base moves
```

| Example | What it shows |
|---|---|
| [`unitree_b1`](examples/unitree_b1.cpp) | The B1 stands with its feet on the floor and moves its base (up and down, rolling, pitching): each leg follows its foot, relative to the desired base, with a kinematic controller that uses `UnitreeB1Robot::kinematics()` |
| [`unitree_b1z1`](examples/unitree_b1z1.cpp) | A Z1 mounted on the back of a B1 (the Unitree B1Z1): the B1 moves its base as in `unitree_b1`, while the Z1, whose kinematics are carried by the measured base, keeps its gripper still in the world; the gripper opens and closes |
| [`unitree_z1`](examples/unitree_z1.cpp) | Two Z1 arms, with and without the gripper, placed with the scene builder: each end effector follows a moving target with a kinematic controller that uses `UnitreeZ1Robot::kinematics()`, and the gripper opens and closes |

## Conventions of the models

- One root body attached to the world (the floating base, for legged robots), and no floor, lights or sky: scenes
  add them.
- A `home` keyframe, with `ctrl`, in which the robot holds its pose (with position actuators).
- Sites at the end effectors (e.g., `end_effector`, the feet), whose poses match the kinematics of the class.
- The sensors of the real robot as MuJoCo sensors on sites (e.g., the IMU of the B1, on the site `imu`, which is
  `imu_site` of the description), read with the sensor methods of cpp-interface-mujoco (`get_imu_orientation()`,
  `get_touch()`, ...).
- Variants (e.g., actuators) as small MJCF files that include the robot (e.g., `b1.xml`, `b1_torque.xml` and
  `b1_robot.xml`), so that they share the robot and its meshes.
- Mesh files in `assets/` (`<compiler meshdir="assets"/>`), light enough to load and render fast: see
  [`tools/decimate_meshes.py`](tools/decimate_meshes.py).
- Each model directory has its own `LICENSE`, and a `README.md` with the source of the model, the changes made to
  it, and how its meshes were generated.

## Adding a robot

1. Put its model in `models/<robot>/` (main XML directly in that directory, assets below it), following the
   conventions above, with its `LICENSE` and `README.md`.
2. Embed it with `dq_mujoco_embed_model()` in [`CMakeLists.txt`](CMakeLists.txt), and add it to `ROBOT_MODELS` (its
   license is then installed with the library).
3. Add `include/.../robots/<Robot>Robot.h` and `src/<Robot>Robot.cpp`, with `description()` and `kinematics()`.
4. Add tests (in [`tests/test_robots.cpp`](tests/test_robots.cpp)) that check its names, its home keyframe, its
   actuators, and that its kinematics match the sites of the model.

## Tools

- [`tools/decimate_meshes.py`](tools/decimate_meshes.py): decimates OBJ meshes with Blender (headless), keeping their
  coordinates. The command used for each model is in its README.

## License

- The files of each model directory (`models/<robot>/`) are under the license in that directory: BSD-3-Clause,
  Copyright (c) 2016-2022 Unitree Robotics, for the Unitree robots. These licenses are installed with the library.
- The other files of this repository (the C++ code, the tools and the documentation) are under the
  [LGPL-3.0-or-later](LICENSE), which supplements the [GPL-3.0](COPYING).
- The installation includes all these licenses, in `share/doc/dqrobotics-interface-mujoco-robots`.

This project uses [DQ Robotics](https://github.com/dqrobotics/cpp) (LGPL-3.0-or-later) and cpp-interface-mujoco
(LGPL-3.0-or-later), but is not part of the DQ Robotics project.
