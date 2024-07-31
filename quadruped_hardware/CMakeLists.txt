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
    include/sdk_3_8_0/lib/cpp/${ARCH}
    include/sdk_3_3_1/lib/cpp/${ARCH}
)

set(SOURCES
    src/IOSDK.cpp
    src/WirelessHandle.cpp
    src/quad_hardware_control.cpp
)

# a1_control 
add_executable(a1_hardware_control ${SOURCES})
add_dependencies(a1_hardware_control ${catkin_EXPORTED_TARGETS})
target_compile_definitions(a1_hardware_control PRIVATE A1_ROBOT)
target_link_libraries(a1_hardware_control  ${catkin_LIBRARIES} -pthread libunitree_legged_sdk_amd64.so lcm)

# aliengo_control 
add_executable(aliengo_hardware_control ${SOURCES})
add_dependencies(aliengo_hardware_control ${catkin_EXPORTED_TARGETS})
target_compile_definitions(aliengo_hardware_control PRIVATE ALIENGO)
target_link_libraries(aliengo_hardware_control  ${catkin_LIBRARIES} -pthread libunitree_legged_sdk_amd64.so lcm)

# go1_control 
add_executable(go1_hardware_control ${SOURCES})
add_dependencies(go1_hardware_control ${catkin_EXPORTED_TARGETS})
target_compile_definitions(go1_hardware_control PRIVATE GO1)
target_link_libraries(go1_hardware_control  ${catkin_LIBRARIES} -pthread libunitree_legged_sdk.a)
