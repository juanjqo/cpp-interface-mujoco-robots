/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco-robots, models of real robots for cpp-interface-mujoco, an
unofficial interface for C++17 between DQ Robotics and MuJoCo:
https://github.com/juanjqo/cpp-interface-mujoco-robots
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoRobotDescription.h>

namespace DQ_robotics_extensions
{

/**
 * @brief The Universal Robots UR3e, a 6-DoF manipulator, with position actuators, velocity actuators
 *        or torque motors.
 *
 * The MuJoCo model is in models/universal_robots_ur3e (made from the ROS 2 description of Universal
 * Robots, BSD-3-Clause), with its meshes embedded in the library.
 *
 * - Fixed base: base_link, the root body, at the origin of the robot, with its x axis forward (as
 *   base_link in ROS).
 * - Joints: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,
 *   wrist_3_joint, with the zero configuration of the robot (and of ROS): at q = 0 the arm is
 *   stretched forward, horizontally. Ranges: [-2pi, 2pi], [-pi, pi] for the elbow, and unlimited
 *   for wrist_3_joint.
 * - Actuators, all within the torque limits of the joints (54 N·m for the shoulder joints, 28 N·m for
 *   the elbow, 9 N·m for the wrist joints):
 *   ACTUATORS::POSITION (default): position actuators, for set_joint_target_positions().
 *   ACTUATORS::VELOCITY: velocity actuators, for set_joint_target_velocities(), within pi rad/s
 *   (shoulder and elbow) and 2pi rad/s (wrist). With zero velocities, the arm sinks slowly.
 *   ACTUATORS::TORQUE: torque motors, for set_joint_target_forces(). Without a controller, the arm
 *   falls.
 * - End effector: the site end_effector, at the center of the flange, with its z axis out of it (tool0
 *   of ROS rotated by pi about its z axis).
 * - Keyframe "home": q = (0, -pi/2, pi/2, -pi/2, -pi/2, 0), with the flange pointing down, and the
 *   actuators holding the pose (ACTUATORS::POSITION), at zero velocity or at zero torque.
 * - kinematics(): the DQ Robotics model of the arm. Its forward kinematics, fkm(q), is the pose of
 *   the end_effector site in the world, with q in the order of joint_names.
 *
 * Example:
 * @code
 * #include <dqrobotics_extensions/interfaces/mujoco/robots/UR3eRobot.h>
 *
 * const auto robot = UR3eRobot::description();   // position actuators
 * mi.load_model(robot.model);
 * mi.reset_simulation(robot.home_keyframe);
 *
 * DQ_SerialManipulatorDH arm = UR3eRobot::kinematics();
 * VectorXd q = mi.get_joint_positions(robot.joint_names);
 * DQ x = arm.fkm(q);                              // equals get_site_pose("end_effector")
 * @endcode
 */
class UR3eRobot
{
public:
    /**
     * @brief The actuators of the arm.
     */
    enum class ACTUATORS
    {
        POSITION,
        VELOCITY,
        TORQUE
    };

    static DQ_MujocoRobotDescription description(const ACTUATORS& actuators = ACTUATORS::POSITION);
    static DQ_robotics::DQ_SerialManipulatorDH kinematics();
};

}
