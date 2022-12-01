cmake_minimum_required(VERSION 3.0.0)
project(robot_kinematics VERSION 0.1.0)

include(CTest)
enable_testing()
include_directories(include)
find_package(Eigen3)

# find_package(math)
add_library(robot_kinematics src/robot_kinematics.cpp)

target_link_libraries(robot_kinematics)

add_executable(main src/main.cpp)
target_link_libraries(main robot_kinematics)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
