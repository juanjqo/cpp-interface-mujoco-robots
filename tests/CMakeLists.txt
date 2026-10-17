# Built from the top-level CMakeLists.txt (DQ_MUJOCO_ROBOTS_BUILD_TESTS=ON)

add_executable(test_robots test_robots.cpp)
target_link_libraries(test_robots PRIVATE dqrobotics_extensions::interface-mujoco-robots)

# Each test runs in its own process, so a crash in one does not hide the others
set(TESTS
    unitree_z1_description
    unitree_z1_actuators
    unitree_z1_kinematics
    edit_scene_with_z1
    unitree_b1_description
    unitree_b1_kinematics
    scene_builder_b1_and_z1
)
foreach(test_name IN LISTS TESTS)
    add_test(NAME ${test_name} COMMAND test_robots ${test_name})
endforeach()
