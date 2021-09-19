cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
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

# include_directories(~/rb_unitree_legged_sdk/include)
# link_directories(~/rb_unitree_legged_sdk/lib)

include_directories(/home/kimsooyoung/Documents/RB/rb_unitree_legged_sdk/include)
link_directories(/home/kimsooyoung/Documents/RB/rb_unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

add_definitions(-DSDK3_2)
add_executable(lcm_server_3_2 /home/kimsooyoung/Documents/RB/rb_unitree_legged_sdk/examples/lcm_server.cpp)
# add_executable(lcm_server_3_2 ~/rb_unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# if( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_1")
#     include_directories($ENV{ALIENGO_SDK_PATH}/include)
#     link_directories($ENV{ALIENGO_SDK_PATH}/lib)
#     string(CONCAT LEGGED_SDK_NAME libaliengo_comm.so)
#     set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

#     add_definitions(-DSDK3_1)

#     add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
#     target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
#     add_dependencies(lcm_server_3_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# elseif( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_2")
#     include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
#     link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
#     string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
#     set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

#     add_definitions(-DSDK3_2)

#     add_executable(lcm_server_3_2 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
#     target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
#     add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# endif()

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

add_executable(rb_walk_mode src/exe/rb_walk_mode.cpp)
target_link_libraries(rb_walk_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(rb_walk_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rb_walk_mode_2 src/exe/rb_walk_mode_2.cpp)
target_link_libraries(rb_walk_mode_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(rb_walk_mode_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rb_walk_mode_3 src/exe/rb_walk_mode_3.cpp)
target_link_libraries(rb_walk_mode_3 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(rb_walk_mode_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sub_test src/exe/sub_test.cpp)
target_link_libraries(sub_test ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(sub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joy_sub_node src/exe/joy_sub_node.cpp)
target_link_libraries(joy_sub_node ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(joy_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
