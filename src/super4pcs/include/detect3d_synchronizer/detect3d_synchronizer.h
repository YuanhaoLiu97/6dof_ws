#ifndef DETECT3D_SYNCHRONIZER_H
#define DETECT3D_SYNCHRONIZER_H

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <memory>
#include "pcl_helper.h"
#include "mmdetection_msgs/msg/detections.hpp"
#include "mmdetection_msgs/msg/detection.hpp"
#include "mmdetection_msgs/msg/mask.hpp"


struct MatchParams
{
    std::string model_pcd_path;
    std::string file_model_config;
    //std::string ImgFolderPath = "/home/maskimg";
    double match_min_score = 0.5;
    int gr_align_sample_size = 120;
    int gr_max_normal_difference = 30;
    int gr_max_color_distance = 10;
    int gr_max_time_seconds = 3; //s
    double gr_delta = 0.005; //匹配精度 m
    double gr_overlap = 0.8;
    pcl_helper::PclParams pcl_params_;
};

namespace detect3d_synchronizer
{
    class Detect3DSynchronizer : public rclcpp::Node
    {
    public:
        explicit Detect3DSynchronizer(const rclcpp::NodeOptions &options);
        ~Detect3DSynchronizer();

        //---------
        // Typedefs
        //---------
        typedef sensor_msgs::msg::Image ImageMsg;
        typedef std::shared_ptr<ImageMsg> ImageMsgPtr;
        typedef std::shared_ptr<ImageMsg const> ImageMsgConstPtr;

        typedef mmdetection_msgs::msg::Detection DetectionMsg;
        typedef mmdetection_msgs::msg::Mask MaskMsg;
        typedef mmdetection_msgs::msg::Detections DetectionsMsg;
        typedef std::shared_ptr<DetectionsMsg> DetectionsMsgPtr;
        typedef std::shared_ptr<DetectionsMsg const> DetectionsMsgConstPtr;

        typedef sensor_msgs::msg::PointCloud2 Cloud2Msg;
        typedef std::shared_ptr<Cloud2Msg> Cloud2MsgPtr;
        typedef std::shared_ptr<Cloud2Msg const> Cloud2MsgConstPtr;

        typedef message_filters::Subscriber<ImageMsg> ImageSubscriber;
        typedef std::shared_ptr<ImageSubscriber> ImageSubscriberPtr;

        typedef message_filters::Subscriber<DetectionsMsg> DetectionsSubscriber;
        typedef std::shared_ptr<DetectionsSubscriber> DetectionsSubscriberPtr;

        typedef message_filters::Subscriber<Cloud2Msg> Cloud2Subscriber;
        typedef std::shared_ptr<Cloud2Subscriber> Cloud2SubscriberPtr;

        typedef message_filters::sync_policies::ApproximateTime<DetectionsMsg, Cloud2Msg> SyncPolicy;

        typedef pcl::PointCloud<pcl::PointXYZRGB> CloudColor;
        typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudColorPtr;

        typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
        typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

        typedef pcl::PointCloud<pcl::Normal> CloudN;
        typedef pcl::PointCloud<pcl::Normal>::Ptr CloudNPtr;

        typedef pcl::PointCloud<pcl::PointNormal> CloudPN;
        typedef pcl::PointCloud<pcl::PointNormal>::Ptr CloudPNPtr;

    private:
        ImageSubscriber image_sub_;
        Cloud2Subscriber  cloud2_sub_;
        DetectionsSubscriber detections_sub_;
        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
        MatchParams matchParams; 



        //publish extract cloud
        sensor_msgs::msg::PointCloud2 extract_cloud_msg;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr extract_cloud_publiser_;

        //---------
        // Declear Params
        //---------
        void declear_params();
        //---------
        // Callbacks
        //---------
        void callback(const DetectionsMsgConstPtr &, const Cloud2MsgConstPtr &);
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &, MatchParams &);
        

        int gr_match(CloudPtr&, CloudPtr&);

        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> m_sync;
    };
} // namespace name

#endif