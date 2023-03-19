#include "detect3d_synchronizer/detect3d_synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

typedef image_transport::Publisher ImagePublisher;
typedef sensor_msgs::msg::Image ImageMsg;

class MsgProducer: public rclcpp::Node{
    public:
        MsgProducer():Node("gr_msg_producer"){
            img_publisher_ = image_transport::create_publisher(this, "/rvc/color_image");
            cloud2_publiser_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rvc/point_cloud", 10);

            timer_ = this->create_wall_timer(10s, std::bind(&MsgProducer::timer_callback, this));
        }
    private:
        void timer_callback(){
            //read image 
            cv::Mat image = cv::imread("/tmp/data/test.jpg");
            if(!image.data){
                RCLCPP_ERROR(this->get_logger(), "Image is empty!");
                return;
            }
            auto cv_ptr= cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image);
            
            //read point cloud
            pcl::PointCloud<pcl::PointXYZ> cloud;
            if(pcl::io::loadOBJFile<pcl::PointXYZ>("/tmp/data/hippo1.obj", cloud)== -1){
                RCLCPP_ERROR(this->get_logger(), "cloud is empty");
                return;
            }

	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(cloud, *cloud_xyzrgb_ptr);

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud_xyzrgb_ptr, cloud_msg);
            cloud_msg.header.frame_id = "world"; 

            //pub image
            img_publisher_.publish(cv_ptr.toImageMsg());
            //pub cloud2 
            cloud2_publiser_->publish(cloud_msg);
             
        }

        ImagePublisher img_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_publiser_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgProducer>());
    rclcpp::shutdown();
    return 0;
}