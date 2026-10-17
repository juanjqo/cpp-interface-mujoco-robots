/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco-robots, models of real robots for cpp-interface-mujoco, an
unofficial interface for C++17 between DQ Robotics and MuJoCo:
https://github.com/juanjqo/cpp-interface-mujoco-robots
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include <dqrobotics_extensions/interfaces/mujoco/robots/UR3eRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics_extensions
{

// Generated from models/universal_robots_ur3e by dq_mujoco_embed_model (see CMakeLists.txt): the
// model with position actuators, ur3e.xml, with the other variants (and the robot they include,
// ur3e_robot.xml) as more assets, so that all of them share the meshes
namespace embedded_models
{
DQ_MujocoModel universal_robots_ur3e();
}

/**
 * @brief The MuJoCo model of the Universal Robots UR3e and the names needed to use it.
 * @param actuators ACTUATORS::POSITION (default), ACTUATORS::VELOCITY or ACTUATORS::TORQUE.
 * @return The description. Its model is embedded in the library, so no files are needed.
 */
DQ_MujocoRobotDescription UR3eRobot::description(const ACTUATORS& actuators)
{
    // The main file of the variant, which includes the robot file
    const std::string robot_file = "ur3e_robot.xml";
    std::string main_file = "ur3e";
    if (actuators == ACTUATORS::VELOCITY)
        main_file += "_velocity";
    if (actuators == ACTUATORS::TORQUE)
        main_file += "_torque";
    main_file += ".xml";

    DQ_MujocoModel model = embedded_models::universal_robots_ur3e();
    if (main_file != model.name)
    {
        const auto& xml = model.assets.at(main_file);
        model.xml.assign(xml.begin(), xml.end());
        model.name = main_file;
    }
    // Only the robot file and the meshes are kept
    for (auto it = model.assets.begin(); it != model.assets.end();)
    {
        const std::string& name = it->first;
        const bool other_xml = name.size() > 4 && name.compare(name.size() - 4, 4, ".xml") == 0 && name != robot_file;
        it = other_xml ? model.assets.erase(it) : std::next(it);
    }

    DQ_MujocoRobotDescription robot;
    robot.model = std::move(model);
    robot.floating_base = "";  // fixed base
    robot.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    robot.home_keyframe = "home";
    robot.end_effector_sites = {"end_effector"};
    return robot;
}

/**
 * @brief The DQ Robotics kinematic model of the Universal Robots UR3e.
 * @return A DQ_SerialManipulatorDH (standard DH) whose forward kinematics, fkm(q), is the pose of
 *         the end_effector site of description() in the world, for q in the order of
 *         description().joint_names. The joint limits are the joint ranges of the MuJoCo model
 *         (+-2pi for wrist_3_joint, which is unlimited in the model).
 *
 * The DH parameters are those of the UR3e of sas_robot_driver_ur
 * (https://github.com/MarinhoLab/sas_robot_driver_ur, robots/ur3e.json), used with the real robot:
 * the nominal DH parameters of Universal Robots, with frame 0 rotated by pi about its z axis, so that
 * its x axis points forward, as that of base_link. Frame 0 is at the height of the shoulder, 0.15185 m
 * above the origin of the robot (the reference frame), and frame 6 is the end_effector site.
 */
DQ_robotics::DQ_SerialManipulatorDH UR3eRobot::kinematics()
{
    using namespace DQ_robotics;
    const double pi2 = pi/2.0;

    Matrix<double, 5, 6> dh;
    //     shoulder_pan  shoulder_lift  elbow    wrist_1  wrist_2  wrist_3
    dh <<  0,            0,             0,       0,       0,       0,        // theta (offset added to q)
           0,            0,             0,       0.13105, 0.08535, 0.0921,   // d
           0,            0.24355,       0.2132,  0,       0,       0,        // a
           -pi2,         0,             0,       -pi2,    pi2,     0,        // alpha
           0,            0,             0,       0,       0,       0;        // joint types (all revolute)

    DQ_SerialManipulatorDH arm(dh);
    arm.set_reference_frame(1 + 0.5*E_*(0.15185*k_));
    arm.set_base_frame(arm.get_reference_frame());

    VectorXd q_min(6);
    VectorXd q_max(6);
    q_min << -2*pi, -2*pi, -pi, -2*pi, -2*pi, -2*pi;  // joint ranges in the XML
    q_max <<  2*pi,  2*pi,  pi,  2*pi,  2*pi,  2*pi;
    arm.set_lower_q_limit(q_min);
    arm.set_upper_q_limit(q_max);
    return arm;
}

}
