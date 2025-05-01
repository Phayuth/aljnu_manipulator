cmake_minimum_required(VERSION 3.8)
project(aljnu_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ur robot package
find_package(ur_robot_driver REQUIRED)
find_package(ur_description REQUIRED)

# onrobotsg gripper package
find_package(onrobotsg REQUIRED)
find_package(onrobotsg_description REQUIRED)
find_package(onrobotsg_interfaces REQUIRED)

# own sub-package
find_package(aljnump_description REQUIRED)

ament_package()
