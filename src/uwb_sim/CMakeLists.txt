cmake_minimum_required(VERSION 3.8)
project(uwb_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo 11 REQUIRED)
find_package(std_msgs REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY confs
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
