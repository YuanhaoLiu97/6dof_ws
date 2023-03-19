cmake_minimum_required(VERSION 3.8)
project(super4pcs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# find dependencies
set(DEPENDENCIES
    sensor_msgs
    vision_msgs
    message_filters
    image_transport
    pcl_conversions
    pcl_ros
    rclcpp
    OpenGR
    PCL
    Eigen3
    cv_bridge
    OpenCV
)

foreach(dependency IN LISTS DEPENDENCIES)
    find_package(${dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${DEPENDENCIES})

ament_auto_add_library(detect3d_synchronizer SHARED src/detect3d_synchronizer.cpp)

#target_include_directories(detect3d_synchronizer PRIVATE
#   include
#)

ament_auto_add_executable(detect3d_node src/detect3d_node.cpp)
target_link_libraries(detect3d_node ${PCL_LIBRARIES})

ament_auto_add_executable(detect3d_msg_producer src/msg_producer.cpp)

install(TARGETS 
    detect3d_node
    detect3d_msg_producer
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
    detect3d_synchronizer
    DESTINATION lib
)

install(DIRECTORY data/
    DESTINATION data
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
