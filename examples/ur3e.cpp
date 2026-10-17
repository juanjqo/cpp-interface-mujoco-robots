/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco-robots, models of real robots for cpp-interface-mujoco, an
unofficial interface for C++17 between DQ Robotics and MuJoCo:
https://github.com/juanjqo/cpp-interface-mujoco-robots
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

/**
Universal Robots UR3e example.

A UR3e of cpp-interface-mujoco-robots on a table, placed in a scene with DQ_MujocoSceneBuilder. Its
end effector follows a target (a site) that moves along a horizontal circle, between the home pose
and the base (the reach of the UR3e is 0.5 m), keeping the orientation
of the home pose (the flange pointing down), with a kinematic controller of its pose that uses the
kinematics of the robot (UR3eRobot::kinematics()), moved to where it was placed.

The model (and its meshes) is embedded in the library, so the example runs from any directory.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h"
#include "dqrobotics_extensions/interfaces/mujoco/robots/UR3eRobot.h"
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
    // The scene: a UR3e on a table
    //--------------------------------------------------------------------------------------------
    DQ_MujocoSceneBuilder scene("Universal Robots UR3e");
    scene.add_floor();
    scene.add_light("sun", 3*k_, DQ(0, 0.3, 0.2, -1), true);
    scene.add_box("table", 1 + 0.5*E_*(0.2*i_ + 0.35*k_), {0.4, 0.3, 0.35}, DQ_MujocoSceneBuilder::BODY::FIXED,
                  {0.45, 0.35, 0.25, 1});
    const DQ x_robot = 1 + 0.5*E_*(0.7*k_);  // on the table
    const DQ_MujocoRobotDescription robot = scene.add_robot(UR3eRobot::description(), "ur3e_", x_robot);
    scene.add_site("target", x_robot, 0.02, {1, 0, 0, 0.5});

    DQ_SerialManipulatorDH arm = UR3eRobot::kinematics();
    arm.set_reference_frame(x_robot*arm.get_reference_frame());

    DQ_MujocoInterface mi;
    mi.load_model(scene.build());
    mi.reset_simulation("home");
    VectorXd q = mi.get_joint_positions(robot.joint_names);
    const DQ x_home = arm.fkm(q);

    //--------------------------------------------------------------------------------------------
    // Run it in real time: the end effector follows the target, which moves along a horizontal
    // circle that starts at the home pose and goes toward the base, with the orientation of the
    // home pose
    //--------------------------------------------------------------------------------------------
    mi.start_simulation();
    const double T = 0.002;      // control loop at 500 Hz
    const double gain = 20.0;
    const double radius = 0.08;
    Clock clock(T);
    clock.init();
    double next_print_time = 1.0;
    while (!mi.should_close_window() && !kill_this_process)
    {
        const double t = mi.get_simulation_time();
        const DQ pd = x_home.translation() + radius*(-(1 - cos(1.0*t))*i_ + sin(1.0*t)*j_);
        const DQ xd = x_home.P() + 0.5*E_*pd*x_home.P();
        mi.set_site_pose("target", xd);

        // Kinematic control of the pose: q_dot = -gain * pinv(J) * vec8(x - xd), integrated
        const DQ x = arm.fkm(q);
        const MatrixXd J = arm.pose_jacobian(q);
        const VectorXd q_dot = -gain*pinv(J)*vec8(x - xd);
        q = (q + q_dot*T).cwiseMax(arm.get_lower_q_limit()).cwiseMin(arm.get_upper_q_limit());
        mi.set_joint_target_positions(robot.joint_names, q);

        if (t >= next_print_time)
        {
            const DQ x_measured = mi.get_site_pose(robot.end_effector_sites.at(0));
            std::cout << "t = " << t << "  end effector to target: "
                      << 1000*(x_measured.translation() - pd).vec3().norm() << " mm"
                      << "  (real-time factor " << mi.get_real_time_factor() << ")" << std::endl;
            next_print_time += 1.0;
        }

        clock.update_and_sleep();
    }
    mi.stop_simulation();
    return 0;
}
