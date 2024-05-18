cmake_minimum_required(VERSION 2.8.3)
project(unitree_description)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    std_msgs
    tf
    a1_description
    aliengo_description
    go1_description
    unitree_legged_msgs
    unitree_legged_control
    unitree_gazebo
)

catkin_package(
    CATKIN_DEPENDS
    a1_description
    aliengo_description
    go1_description
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
