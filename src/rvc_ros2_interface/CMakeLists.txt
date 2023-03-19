cmake_minimum_required(VERSION 3.8)
project(rvc_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wdeprecated-declarations)
endif()

set(RVC_ROOT "/opt/RVC")
set(RVC_INCLUDE_DIRS "${RVC_ROOT}/include/")
set(RVC_LIB_DIR "${RVC_ROOT}/lib/")
file(GLOB RVC_LIBS "${RVC_LIB_DIR}/libRVC.so" ABSOLUTE)
file(GLOB RVC_DEPS_LIBS "${RVC_LIB_DIR}/externals/*.so" ABSOLUTE)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImagePc.srv"

  DEPENDENCIES
  std_msgs
  sensor_msgs
)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${RVC_INCLUDE_DIRS}
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

link_directories(
    ${RVC_LIB_DIR}
    ${PCL_LIBRARY_DIRS}
)

add_executable(start src/main.cpp)
ament_target_dependencies(start
    cv_bridge 
    rclcpp
    std_msgs
    pcl_conversions
    sensor_msgs
    rosidl_default_generators
    )
# other dependency
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_target_interfaces(start ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(start
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${RVC_LIBS} 
    ${RVC_DEPS_LIBS}
    ${cpp_typesupport_target}
)

#colcon针对start添加rvc运行依赖
set_target_properties(start PROPERTIES INSTALL_RPATH ${RVC_LIB_DIR})

install(TARGETS start
    DESTINATION lib/${PROJECT_NAME}
)

#add_executable(listener src/subscriber_member_function.cpp)
#ament_target_dependencies(listener rclcpp std_msgs)

#install(
#  DIRECTORY lib/    #发现你项目中的lib中所有的文件
#  DESTINATION lib/${PROJECT_NAME} #拷贝到install目录中
#)

#install(TARGETS start
    #listener
#    RUNTIME DESTINATION lib/${PROJECT_NAME}
#) #程序运行的时候调用lib
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
