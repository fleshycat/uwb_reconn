cmake_minimum_required(VERSION 3.5)
project(gazebo_uwb_sensor_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_plugins REQUIRED)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

add_library(uwb_plugin SHARED src/gazebo_uwb_plugin.cpp)

ament_target_dependencies(uwb_plugin
  "rclcpp"
  "gazebo_ros"
  "std_msgs"
  "uwb_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

target_include_directories(uwb_plugin PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(uwb_plugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_export_dependencies(
  rclcpp
  gazebo_ros
  std_msgs
  uwb_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  gazebo_plugins
)

ament_export_libraries(uwb_plugin)

install(
  TARGETS uwb_plugin
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

