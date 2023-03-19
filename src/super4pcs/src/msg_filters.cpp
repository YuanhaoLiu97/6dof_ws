#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

/*
class ExtractImagePointCloudSubscriber: public rclcpp::Node{
    public:

        ExtractImagePointCloudSubscriber():Node("extract_image_pointcloud_subscriber"){
            sub_image_.subscribe(this, "/rvc/color_image");
            sub_pc_.subscribe(this, "/rvc/point_cloud");
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::PointCloud2> ApproximatePolicy;
            message_filters::Synchronizer<ApproximatePolicy> sync_(ApproximatePolicy(10), sub_image_, sub_pc_);  
            sync_.registerCallback(&ExtractImagePointCloudSubscriber::topic_callback, this);
        }
    
        void topic_callback(const sensor_msgs::msg::Image::SharedPtr img, const sensor_msgs::msg::PointCloud2::SharedPtr cloud2)
        {
            std::cout << "Received message with value:" << std::endl;

            RCLCPP_INFO(this->get_logger(), "received images and clouds");
        }

    public:
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pc_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtractImagePointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}*/

typedef std::shared_ptr<const sensor_msgs::msg::Image> ImageMsgPtr;
typedef std::shared_ptr<const sensor_msgs::msg::PointCloud2> Cloud2MsgPtr;


void topic_callback(const ImageMsgPtr& img, const Cloud2MsgPtr& cloud2)
{
    std::cout << "Received message with value:" << std::endl;
}


 int main(int argc, char** argv)
 {
   rclcpp::init(argc, argv);
   auto nh = std::make_shared<rclcpp::Node>("extract_image_pointcloud_subscriber");
   message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_(nh.get(), "/rvc/color_image");
   message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pc_(nh.get(), "/rvc/point_cloud");
 
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::PointCloud2> ApproximatePolicy;
   message_filters::Synchronizer<ApproximatePolicy> sync_(ApproximatePolicy(10), sub_image_,sub_pc_);  
   sync_.registerCallback(std::bind(&topic_callback, _1, _2));
   rclcpp::spin(nh);
   return 0;
 }
 
