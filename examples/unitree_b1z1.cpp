/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco-robots, models of real robots for cpp-interface-mujoco, an
unofficial interface for C++17 between DQ Robotics and MuJoCo:
https://github.com/juanjqo/cpp-interface-mujoco-robots
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Unitree B1Z1 example.

A Unitree Z1 (with its gripper) mounted on the back of a Unitree B1, as in the Unitree B1Z1, built
from the models of both robots with DQ_MujocoSceneBuilder: the Z1 is added with the base of the B1
as its parent body, at UnitreeB1Robot::arm_mount(), so that the base carries it.

The B1 stands with its feet on the floor and moves its base (up and down, rolling and pitching), as
in the unitree_b1 example, while the Z1 keeps its gripper still in the world: the kinematics of the
Z1 (UnitreeZ1Robot::kinematics()) has its reference frame on the base of the B1, where it was
mounted, and is updated with the measured pose of the base. The gripper opens and closes.

The models (and their meshes) are embedded in the library, so the example runs from any directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h"
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <marinholab/sas/core/sas_clock.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;
using marinholab::sas::core::Clock;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/

static std::atomic_bool kill_this_process(false);

static void sig_int_handler(int)
{
    kill_this_process = true;
}


int main()
{
    std::signal(SIGINT, sig_int_handler);

    //--------------------------------------------------------------------------------------------
    // The scene: a B1, and a Z1 mounted on its base
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("Unitree B1Z1");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    const DQ_MujocoRobotDescription b1 = scene.add_robot(UnitreeB1Robot::description(), "b1_");
    const DQ_MujocoRobotDescription z1 = scene.add_robot(UnitreeZ1Robot::description(), "z1_",
                                                         UnitreeB1Robot::arm_mount(), b1.floating_base);

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");  // both robots at home
    mi.set_site_frame_view(false);
    mi.add_reference_frame("desired base", 0.3, 0.01, 0.7);
    mi.add_reference_frame("gripper target", 0.15, 0.006, 0.7);

    // The legs, in the frame of the base, and where the feet are in the world
    std::vector<DQ_SerialManipulatorDH> legs = UnitreeB1Robot::kinematics();
    VectorXd q_b1 = mi.get_joint_positions(b1.joint_names);
    const DQ x_base_home = mi.get_base_pose(b1.floating_base);
    std::vector<DQ> p_feet;
    for (std::size_t i = 0; i < legs.size(); i++)
        p_feet.push_back((x_base_home*legs.at(i).fkm(q_b1.segment(3*i, 3))).translation());

    // The arm: its reference frame is on the base of the B1, where it was mounted. It starts in a
    // posture with the gripper in front of the robot, away from the joint limits (at home, joint3 is
    // close to its limit), and the gripper keeps that pose in the world.
    DQ_SerialManipulatorDH arm = UnitreeZ1Robot::kinematics();
    const DQ arm_reference = UnitreeB1Robot::arm_mount()*arm.get_reference_frame();
    VectorXd q_z1(6);
    q_z1 << 0, 1.2, -1.2, 0, 0, 0;
    mi.set_joint_positions(z1.joint_names, q_z1);
    mi.set_joint_target_positions(z1.joint_names, q_z1);
    arm.set_reference_frame(x_base_home*arm_reference);
    const DQ x_gripper_d = arm.fkm(q_z1);
    mi.set_reference_frame_pose("gripper target", x_gripper_d);

    //--------------------------------------------------------------------------------------------
    // Run it in real time: the base moves, the feet stay where they are, and so does the gripper
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const double T = 0.002;      // control loop at 500 Hz
    const double gain = 50.0;
    Clock clock(T);
    clock.init();
    double next_print_time = 1.0;
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();

        // The desired pose of the base: up and down 6 cm, rolling 0.15 rad and pitching 0.1 rad
        const double roll = 0.15*sin(1.1*t);
        const double pitch = 0.10*sin(0.7*t);
        const DQ r = (cos(roll/2) + i_*sin(roll/2))*(cos(pitch/2) + j_*sin(pitch/2));
        const DQ p = 0.06*sin(0.8*t)*k_;
        const DQ x_base_d = x_base_home*(r + 0.5*E_*p*r);
        mi.set_reference_frame_pose("desired base", x_base_d);

        // Each leg follows its foot, relative to the desired base:
        // q_leg_dot = -gain * pinv(Jt) * vec4(p_foot - p_foot_d), integrated
        for (std::size_t i = 0; i < legs.size(); i++)
        {
            const DQ p_foot_d = (conj(x_base_d)*(1 + 0.5*E_*p_feet.at(i))).translation();
            VectorXd q_leg = q_b1.segment(3*i, 3);
            const DQ x_foot = legs.at(i).fkm(q_leg);
            const MatrixXd Jt = DQ_Kinematics::translation_jacobian(legs.at(i).pose_jacobian(q_leg), x_foot);
            q_leg += -gain*pinv(Jt)*vec4(x_foot.translation() - p_foot_d)*T;
            q_b1.segment(3*i, 3) = q_leg.cwiseMax(legs.at(i).get_lower_q_limit())
                                        .cwiseMin(legs.at(i).get_upper_q_limit());
        }
        mi.set_joint_target_positions(b1.joint_names, q_b1);

        // The arm, carried by the measured base, keeps the gripper still:
        // q_arm_dot = -gain * (J^T J + lambda I)^-1 J^T vec8(x - x_d), integrated (a damped
        // pseudoinverse, which stays bounded near singular configurations)
        arm.set_reference_frame(mi.get_base_pose(b1.floating_base)*arm_reference);
        const DQ x_gripper = arm.fkm(q_z1);
        const MatrixXd J = arm.pose_jacobian(q_z1);
        q_z1 += -gain*(J.transpose()*J + 1e-4*MatrixXd::Identity(6, 6)).inverse()*J.transpose()
                *vec8(x_gripper - x_gripper_d)*T;
        q_z1 = q_z1.cwiseMax(arm.get_lower_q_limit()).cwiseMin(arm.get_upper_q_limit());
        mi.set_joint_target_positions(z1.joint_names, q_z1);

        // The gripper opens and closes (from -1.5 rad, open, to 0, closed)
        mi.set_joint_target_positions(z1.gripper_joint_names, (VectorXd(1) << -0.6*(1 - cos(2.0*t))).finished());

        if (t >= next_print_time)
        {
            const DQ x_base = mi.get_base_pose(b1.floating_base);
            const DQ x_gripper_measured = mi.get_site_pose(z1.end_effector_sites.at(0));
            std::cout << "t = " << t << "  base to desired: "
                      << 1000*(x_base.translation() - x_base_d.translation()).vec3().norm() << " mm"
                      << "  gripper to target: "
                      << 1000*(x_gripper_measured.translation() - x_gripper_d.translation()).vec3().norm() << " mm, "
                      << vec4(x_gripper_measured.P() - x_gripper_d.P()).norm() << " (rotation)"
                      << "  (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            next_print_time += 1.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
