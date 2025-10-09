cmake_minimum_required(VERSION 3.5)
project(husky_velodyne_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(husky_description REQUIRED)
find_package(velodyne_description REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

