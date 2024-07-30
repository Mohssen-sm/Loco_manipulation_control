cmake_minimum_required(VERSION 3.14)
project(quadruped_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED 
    COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    std_msgs
    unitree_legged_msgs 
    qpoases_catkin 
)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS 
    unitree_legged_msgs 
    qpoases_catkin
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_LIST
"src/BalanceController/*.cpp"
"src/common/*.cpp"
"src/ConvexMPC/*.cpp"
"src/FSM/*.cpp"
"src/interface/*.cpp"
)

add_library(${PROJECT_NAME}
    ${SRC_LIST}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ) 
