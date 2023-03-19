cmake_minimum_required(VERSION 3.5)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegistrationHomos.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

ament_package()