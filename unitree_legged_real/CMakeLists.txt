cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    unitree_legged_msgs
)

catkin_package()

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    "/usr/include/eigen3"
    "/usr/local/include/eigen3"
)

set(CMAKE_CXX_FLAGS "-O3")

# change into your Dir
set(LEGGED_SDK_DIR "/home/kimsooyoung/Downloads/unitree_legged_sdk-3.3.1")

include_directories(/home/kimsooyoung/Downloads/unitree_legged_sdk-3.3.1/include)
link_directories(/home/kimsooyoung/Downloads/unitree_legged_sdk-3.3.1/lib)

string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

# change into your Dir
add_executable(lcm_server /home/kimsooyoung/Downloads/unitree_legged_sdk-3.3.1/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(velocity_lcm src/exe/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(torque_lcm src/exe/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(walk_lcm src/exe/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(stairs_climbing src/exe/stairs_climbing.cpp)
target_link_libraries(stairs_climbing ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(stairs_climbing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_control_go1 src/exe/ros_control_go1.cpp)
# target_link_libraries(ros_control_go1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_control_go1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_control_a1 src/exe/ros_control_a1.cpp)
target_link_libraries(ros_control_a1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_control_a1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rb_walk_mode_4 src/exe/rb_walk_mode_4.cpp)
target_link_libraries(rb_walk_mode_4 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(rb_walk_mode_4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joy_sub_node src/exe/joy_sub_node.cpp)
target_link_libraries(joy_sub_node ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(joy_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
