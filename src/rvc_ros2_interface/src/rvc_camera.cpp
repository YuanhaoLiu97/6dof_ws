#include <rvc_camera.h>
#include <iostream>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointMap2CloudPoint(RVC::PointMap& pm, RVC::Image& img) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    const unsigned int w = pm.GetSize().width, h = pm.GetSize().height;
    cloud->height = h;
    cloud->width = w;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    int pm_sz = cloud->height * cloud->width;
    const double* pm_data = pm.GetPointDataPtr();
    const unsigned char* img_data = img.GetDataPtr();
    RVC::ImageType::Enum image_type = img.GetType();
    for (int i = 0; i < pm_sz; i++) {
        cloud->points[i].x = pm_data[0];
        cloud->points[i].y = pm_data[1];
        cloud->points[i].z = pm_data[2];
        switch (image_type) {
        case RVC::ImageType::Mono8: {
            cloud->points[i].r = img_data[0];
            cloud->points[i].g = img_data[0];
            cloud->points[i].b = img_data[0];
            img_data++;
            break;
        }
        case RVC::ImageType::RGB8: {
            cloud->points[i].r = img_data[0];
            cloud->points[i].g = img_data[1];
            cloud->points[i].b = img_data[2];
            img_data += 3;
            break;
        }
        case RVC::ImageType::BGR8: {
            cloud->points[i].r = img_data[2];
            cloud->points[i].g = img_data[1];
            cloud->points[i].b = img_data[0];
            img_data += 3;
            break;
        }
        default:
            break;
        }
        pm_data += 3;
    }

    return cloud;
}

RvcCamera::RvcCamera(){
    node = rclcpp::Node::make_shared("rvc_camera_publisher_service");
    //param declare
    node->declare_parameter<bool>("save_file", false);
    node->declare_parameter<int>("exposure_time_2d", 100);
    node->declare_parameter<int>("exposure_time_3d", 50);
    node->declare_parameter<bool>("transform_to_camera", true);

    pub_color = node->create_publisher<sensor_msgs::msg::Image>("/rvc/color_image", 1);
    pub_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rvc/point_cloud", 1);

    //connect camera
    if (findAndCapture(x1_device) == -1){
        return;
    }

    capture_image_pc_service = 
        node->create_service<rvc_ros2_interface::srv::CaptureImagePc>("/rvc/capture", std::bind(&RvcCamera::capture_callback, this, std::placeholders::_1, std::placeholders::_2));
}

RvcCamera::~RvcCamera(){
        x1_device.Close();
        RVC::X1::Destroy(x1_device);
        RVC::SystemShutdown();
        std::cout << "Camera is closed!" << std::endl;

}

int RvcCamera::findAndCapture(RVC::X1 &x1_device){
    // Initialize RVC X system.
    RVC::SystemInit();

    // Scan all RVC X Camera devices.
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::GigE);

    // Find whether any RVC X Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any RVC X Camera!" << std::endl;
        return -1;
    }

    // Create a RVC X Camera and choose use left side camera.
    x1_device = RVC::X1::Create(devices[0], RVC::CameraID_Left);

    // Open RVC X Camera.
    x1_device.Open();

    // Test RVC X Camera is opened or not.
    if (!x1_device.IsOpen()) {
        std::cout << "RVC X Camera is not opened!" << std::endl;
        RVC::X1::Destroy(x1_device);
        RVC::SystemShutdown();
        return -1;
    }
    std::cout << "RVC Camera Open!" << std::endl;
    return 1;
}

void RvcCamera::capture_callback(const std::shared_ptr<rvc_ros2_interface::srv::CaptureImagePc::Request> req, std::shared_ptr<rvc_ros2_interface::srv::CaptureImagePc::Response> res)
{
        //param get
    node->get_parameter("exposure_time_2d", exposure_time_2d);
    node->get_parameter("exposure_time_3d", exposure_time_3d);
    node->get_parameter("transform_to_camera", transform_to_camera);

    cap_opt.exposure_time_2d = exposure_time_2d;
    cap_opt.exposure_time_3d = exposure_time_3d;
    cap_opt.transform_to_camera = transform_to_camera;
    if(x1_device.Capture(cap_opt) == true){
        res->error_code = 1;
        res->error_description = "capture successful";
    }else{
        res->error_code = -1;
        res->error_description = "capture failed";
    }

    RVC::Image colorMap = x1_device.GetImage();

    res->error_code = int(colorMap.IsValid());
    res->error_description = colorMap.IsValid() ? "capture successful" : "capture failed";
    publishColorMap(colorMap);
    std::cout << "image published!" << std::endl;

    RVC::PointMap pm = x1_device.GetPointMap();
    publishPointCloud(pm, colorMap);
    std::cout << "pointcloud published!" << std::endl;

}

void RvcCamera::publishColorMap(RVC::Image &colorMap)
{
    cv::Mat color = cv::Mat(colorMap.GetSize().height, colorMap.GetSize().width, CV_8UC3, colorMap.GetDataPtr());
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::msg::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "color_map";
    ros_image.header.stamp = node->now();
    pub_color->publish(ros_image);
    node->get_parameter("save_file", save_file);
    if (save_file){
        std::string img_path= "/tmp/rvc_image.png";
        colorMap.SaveImage(img_path.c_str());
    }
        
}

void RvcCamera::publishPointCloud(RVC::PointMap &pm, RVC::Image &colorMap)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = PointMap2CloudPoint(pm, colorMap);
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    //parent frame id
    ros_cloud.header.frame_id = "camera";
    ros_cloud.header.stamp = node->now();
    pub_pcl->publish(ros_cloud);
    node->get_parameter("save_file", save_file);
    if (save_file)
        pcl::io::savePCDFileBinary("/tmp/rvc_cloud.pcd", *cloud);
}



