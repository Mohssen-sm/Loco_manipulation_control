cmake_minimum_required(VERSION 3.14)
project(quadruped_sim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED 
    COMPONENTS
    tf
    a1_description
    aliengo_description
    go1_description
    unitree_legged_control
    unitree_gazebo
    quadruped_control
)

find_package(gazebo REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    CATKIN_DEPENDS
    a1_description
    aliengo_description
    go1_description
    unitree_legged_control
    unitree_gazebo
    quadruped_control
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/IOROS.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ) 

# quad_control node
add_executable(quad_sim_control src/quad_sim_control.cpp)
add_dependencies(quad_sim_control ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_sim_control ${PROJECT_NAME} ${catkin_LIBRARIES} -pthread lcm)
