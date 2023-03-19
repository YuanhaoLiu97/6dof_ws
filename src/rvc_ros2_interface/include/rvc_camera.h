#pragma once

#include <RVC/RVC.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "rvc_ros2_interface/srv/capture_image_pc.hpp"

class RvcCamera
{
public:
  RvcCamera();
  ~RvcCamera();
  rclcpp::Node::SharedPtr node;

private:
  RVC::X1 x1_device;
  RVC::X1::CaptureOptions cap_opt;
  
  int findAndCapture(RVC::X1& x1_device);

  bool save_file = false;  // param save file
  int exposure_time_2d = 100;
  int exposure_time_3d = 50;
  bool transform_to_camera = true;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_color;

  void publishColorMap(RVC::Image& colorMap);
  void publishPointCloud(RVC::PointMap &pointXYZMap, RVC::Image &colorMap);

  
  rclcpp::Service<rvc_ros2_interface::srv::CaptureImagePc>::SharedPtr capture_image_pc_service;

  void capture_callback(const std::shared_ptr<rvc_ros2_interface::srv::CaptureImagePc::Request> req,
                                  std::shared_ptr<rvc_ros2_interface::srv::CaptureImagePc::Response> res);
};