cmake_minimum_required(VERSION 3.14)
project(quadruped_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED 
    COMPONENTS
    quadruped_control
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    CATKIN_DEPENDS
    quadruped_control
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(
    include/sdk/lib/cpp/${ARCH}
)

add_library(${PROJECT_NAME}
        src/sdk/IOSDK.cpp
        src/WirelessHandle.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        libunitree_legged_sdk.a
        ) 

# quad_control node
add_executable(quad_hardware_control src/quad_hardware_control.cpp)
add_dependencies(quad_hardware_control ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_hardware_control ${PROJECT_NAME} ${catkin_LIBRARIES} -pthread lcm)
