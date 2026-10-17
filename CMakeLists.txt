#---------------------------------------------------------------------------------------------------
# cpp-interface-mujoco-robots: models of real robots for cpp-interface-mujoco, each with a class that
# gives its MuJoCo model (embedded in the library: no files needed at runtime), the names needed to
# use it, and its DQ Robotics kinematics.
#
# Needs cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco), installed. Build and
# install (from the root of this repository):
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=<prefix of cpp-interface-mujoco>
#   cmake --build build -j$(nproc)
#   ctest --test-dir build --output-on-failure
#   sudo cmake --install build
#
# Then, in another CMake project:
#   find_package(dqrobotics-interface-mujoco-robots REQUIRED)
#   target_link_libraries(my_app PRIVATE dqrobotics_extensions::interface-mujoco-robots)
#---------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.28)
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(dqrobotics-interface-mujoco-robots
        VERSION 1.0
        DESCRIPTION "Models of real robots (MuJoCo) and their kinematics (DQ Robotics) for cpp-interface-mujoco."
        HOMEPAGE_URL "https://github.com/juanjqo/cpp-interface-mujoco-robots"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to an optimized build (-O3 -DNDEBUG) when no build type is given
if(PROJECT_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DQ_MUJOCO_ROBOTS_BUILD_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
option(DQ_MUJOCO_ROBOTS_BUILD_TESTS "Build the tests" ${PROJECT_IS_TOP_LEVEL})

include(GNUInstallDirs)

# The interface, and dq_mujoco_embed_model()
find_package(dqrobotics-interface-mujoco 1.0 REQUIRED)

#---------------------------------------------------------------------------------------------------
# Library
#
# To add a robot:
#   1. put its model in models/<robot>/ (main XML directly in that directory, assets below it), with
#      its LICENSE and a README.md with its source and the changes made to it
#   2. embed it with dq_mujoco_embed_model() below, and add its directory to ROBOT_MODELS
#   3. add include/.../robots/<Robot>Robot.h and src/<Robot>Robot.cpp, which fill the description
#      and give the kinematics, and tests that check that the kinematics match the model
#---------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}
    src/UnitreeB1Robot.cpp
    src/UnitreeZ1Robot.cpp
)
add_library(dqrobotics_extensions::interface-mujoco-robots ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME interface-mujoco-robots)

set(ROBOT_MODELS unitree_b1 unitree_z1)
set(MODELS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models)

# The B1 with position actuators, and with velocity actuators and torque motors (b1_velocity.xml and
# b1_torque.xml, as more assets), sharing the robot (b1_robot.xml) and the meshes
file(GLOB unitree_b1_meshes RELATIVE ${MODELS_DIR}/unitree_b1
     CONFIGURE_DEPENDS ${MODELS_DIR}/unitree_b1/assets/*.obj)
dq_mujoco_embed_model(${PROJECT_NAME}
    FUNCTION  unitree_b1
    DIRECTORY ${MODELS_DIR}/unitree_b1
    MAIN      b1.xml
    ASSETS    b1_velocity.xml b1_torque.xml b1_robot.xml ${unitree_b1_meshes}
)
# The Z1 with its gripper and position actuators, and the other variants (as more assets), sharing the
# robots they include (z1_robot.xml and z1_gripper_robot.xml) and the meshes
file(GLOB unitree_z1_meshes RELATIVE ${MODELS_DIR}/unitree_z1
     CONFIGURE_DEPENDS ${MODELS_DIR}/unitree_z1/assets/*.stl)
dq_mujoco_embed_model(${PROJECT_NAME}
    FUNCTION  unitree_z1
    DIRECTORY ${MODELS_DIR}/unitree_z1
    MAIN      z1_gripper.xml
    ASSETS    z1.xml z1_velocity.xml z1_torque.xml z1_gripper_velocity.xml z1_gripper_torque.xml
              z1_robot.xml z1_gripper_robot.xml ${unitree_z1_meshes}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# Public: the descriptions are loaded with DQ_MujocoInterface
target_link_libraries(${PROJECT_NAME} PUBLIC dqrobotics_extensions::interface-mujoco)

if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

#---------------------------------------------------------------------------------------------------
# Install: provides find_package(dqrobotics-interface-mujoco-robots) and the target
# dqrobotics_extensions::interface-mujoco-robots
#---------------------------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
set(CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# The licenses, with the documentation: this package's (LICENSE, the LGPL-3.0, and COPYING, the
# GPL-3.0 that it supplements)
install(FILES LICENSE COPYING DESTINATION ${CMAKE_INSTALL_DOCDIR})
# The licenses of the embedded models (e.g., BSD-3-Clause asks to ship its notice with the binaries)
foreach(model IN LISTS ROBOT_MODELS)
    install(FILES ${MODELS_DIR}/${model}/LICENSE DESTINATION ${CMAKE_INSTALL_DOCDIR}/models/${model})
endforeach()

install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE dqrobotics_extensions::
    DESTINATION ${CONFIG_INSTALL_DIR}
)
configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CONFIG_INSTALL_DIR}
)

#---------------------------------------------------------------------------------------------------
# Examples and tests
#---------------------------------------------------------------------------------------------------
if(DQ_MUJOCO_ROBOTS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(DQ_MUJOCO_ROBOTS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
