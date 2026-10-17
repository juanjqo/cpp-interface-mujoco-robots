# Built from the top-level CMakeLists.txt (DQ_MUJOCO_ROBOTS_BUILD_EXAMPLES=ON)

set(EXAMPLES
    unitree_b1
    unitree_z1
    unitree_b1z1
)

foreach(example IN LISTS EXAMPLES)
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PRIVATE dqrobotics_extensions::interface-mujoco-robots)
    # The examples pace their loops with a sas Clock
    target_link_libraries(${example} PRIVATE marinholab::sas::core)
endforeach()
