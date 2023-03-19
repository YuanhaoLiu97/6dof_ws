#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "gr/utils/shared.h"

using namespace gr;

struct MatchParams
{
    std::string model_pcd_path;
    std::string file_model_config;
    //std::string ImgFolderPath = "/home/maskimg";
    double normal_estimate_radius_search = 0.002; // m

    double voxel_leaf_size = 0.002;
    int sor_meank = 400; 
    double sor_StddevMulThresh=0.3;

    double match_min_score = 0.5;
    int gr_align_sample_size = 120;
    int gr_max_normal_difference = 30;
    int gr_max_color_distance = 10;
    int gr_max_time_seconds = 3; //s
    double gr_delta = 0.005; //匹配精度 m
    double gr_Overlap = 0.7;

    double icp_max_correspondence_distance = 0.01; //m
    int icp_maximum_iterations = 30;
    double icp_transformation_epsilon = 1e-8;
    double icp_euclidean_fitness_epsilon = 0.001;
};

//define params 
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::Normal>::Ptr normals_scene(new pcl::PointCloud<pcl::Normal>);

class SuperFourPCSPublisher: public rclcpp::Node{
    public:
        //intra_process, avoid memory copy.
        SuperFourPCSPublisher(const std::string & node_name, const std::string & pub_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)){
            //declear parameters
            this->declare_parameter("model_pcd_path", rclcpp::PARAMETER_STRING);
            this->declare_parameter("normal_estimate_radius_search", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("voxel_leaf_size",rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("sor_meank", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("sor_stddev_multhresh", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("match_min_score", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("gr_align_sample_size", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("gr_max_normal_difference", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("gr_max_color_distance", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("gr_max_time_seconds", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("gr_delta", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("gr_overlap", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("icp_max_correspondence_distance", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("icp_maximum_iterations", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("icp_transformation_epsilon", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("icp_euclidean_fitness_epsilon", rclcpp::PARAMETER_DOUBLE);

            detect3d_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(pub_name,10);
        }

    private:
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detect3d_publisher_;
        MatchParams mParams_;
};

class SuperFourPCSConsumer : public rclcpp::Node
{
    public:
        SuperFourPCSConsumer(const std::string & node_name, const std::string & sub_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        {
            // Create a subscription on the input topic which prints on receipt of new messages.
            sub_ = this->create_subscription<std_msgs::msg::Int32>(
            sub_name,
            10,
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                std::cout << "Received message with value:" << msg->data << std::endl;
                RCLCPP_INFO(this->get_logger(), "Publishing");
            });
        }
    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);
    //start single thread
    rclcpp::executors::SingleThreadedExecutor executor;
    auto producer = std::make_shared<SuperFourPCSPublisher>("super4pcs_producer", "global_register");
    auto consumer = std::make_shared<SuperFourPCSConsumer>("super4pcs_consumer", "input");

    executor.add_node(producer);
    executor.add_node(consumer);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}