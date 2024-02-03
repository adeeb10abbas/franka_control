cmake_minimum_required(VERSION 3.5)
project(franka_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED) # Add Eigen3 as a dependency
find_package(Franka REQUIRED) # Add Franka as a dependency
find_package(Threads REQUIRED) # For threading

set(message_files
  "msg/PTIPacket.msg"
  "msg/PInfo.msg"
)
# Define custom messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} # Include Eigen3 directories
)

# Corrected path for the examples_common library source file
add_library(examples_common STATIC
  examples/examples_common.cpp
)
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen)

# Corrected path for the avatar_panda executable source file
add_executable(avatar_panda examples/avatar_panda.cpp)
add_dependencies(avatar_panda ${PROJECT_NAME}__rosidl_typesupport_cpp)
rosidl_target_interfaces(avatar_panda
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(avatar_panda
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install executables and/or libraries
install(TARGETS
  avatar_panda
  examples_common
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# ament_target_dependencies(avatar_panda rclcpp std_msgs geometry_msgs sensor_msgs franka_control)
target_link_libraries(avatar_panda examples_common Franka::Franka Eigen3::Eigen ${rclcpp_LIBRARIES} Threads::Threads)

# Install executables and/or libraries
install(TARGETS
  avatar_panda
  examples_common
  DESTINATION lib/${PROJECT_NAME}
)

# # Install directories such as launch files, if any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
