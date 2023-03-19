#include "detect3d_synchronizer/detect3d_synchronizer.h"
#include <rclcpp/logging.hpp>
#include<rclcpp/clock.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "pcl/registration/super4pcs.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>

using namespace std::placeholders;
namespace detect3d_synchronizer
{
    Detect3DSynchronizer::Detect3DSynchronizer(const rclcpp::NodeOptions & options)
    : rclcpp::Node("detect3d_synchronizer", options)
    {
        //---------
        //Declare Params
        //---------
       declear_params();

        // create param callback
        params_callback_handle_= this->add_on_set_parameters_callback(
            std::bind(&Detect3DSynchronizer::parametersCallback, this, std::placeholders::_1, matchParams));
        
        //---------
        //Typedefs
        //---------
        image_sub_.subscribe(this, "/rvc/color_image");
        detections_sub_.subscribe(this, "detections");
        cloud2_sub_.subscribe(this, "/rvc/point_cloud");
        extract_cloud_publiser_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pose_detection/extact_cloud", 10);

        m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), detections_sub_,cloud2_sub_);  
        m_sync->registerCallback(std::bind(&Detect3DSynchronizer::callback, this, _1, _2));
    }

    Detect3DSynchronizer::~Detect3DSynchronizer(){
    }

    void Detect3DSynchronizer:: declear_params(){
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

        matchParams.match_min_score = this->get_parameter("match_min_score").as_double();
        matchParams.gr_align_sample_size = this->get_parameter("gr_align_sample_size").as_int();
        matchParams.gr_max_normal_difference = this->get_parameter("gr_max_normal_difference").as_int();
        matchParams.gr_max_color_distance = this->get_parameter("gr_max_color_distance").as_int();
        matchParams.gr_max_time_seconds = this->get_parameter("gr_max_time_seconds").as_int();
        matchParams.gr_delta = this->get_parameter("gr_delta").as_double();
        matchParams.gr_overlap = this->get_parameter("gr_overlap").as_double();
        matchParams.pcl_params_.icp_max_correspondence_distance = this->get_parameter("icp_max_correspondence_distance").as_double();
        matchParams.pcl_params_.icp_maximum_iterations = this->get_parameter("icp_maximum_iterations").as_int();
        matchParams.pcl_params_.icp_transformation_epsilon = this->get_parameter("icp_transformation_epsilon").as_double();
        matchParams.pcl_params_.icp_euclidean_fitness_epsilon = this->get_parameter("icp_euclidean_fitness_epsilon").as_double();
        matchParams.pcl_params_.normal_estimate_radius_search = this->get_parameter("normal_estimate_radius_search").as_double();
        matchParams.pcl_params_.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        matchParams.pcl_params_.sor_meank = this->get_parameter("sor_meank").as_int();
        matchParams.pcl_params_.sor_stddev_multhresh = this->get_parameter("sor_stddev_multhresh").as_double();
    }

    rcl_interfaces::msg::SetParametersResult Detect3DSynchronizer::parametersCallback(const std::vector<rclcpp::Parameter> & parameters, MatchParams &match_params){
        RCLCPP_INFO(this->get_logger(), "params callback!");
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (auto& param : parameters) {
            RCLCPP_INFO(this->get_logger(), "param %s update", param.get_name().c_str());
            if (param.get_name() == "normal_estimate_radius_search") {
                match_params.pcl_params_.normal_estimate_radius_search = param.as_double();
                RCLCPP_INFO(this->get_logger(), "param normal_estimate_radius_search set %lf", match_params.pcl_params_.normal_estimate_radius_search);
            } 
        }
        return result;
    }

/*
    void multi_callback_k(const sensor_msgs::PointCloud2ConstPtr& inputCloud2, const sensor_msgs::ImageConstPtr& image)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*inputCloud2, *cloud);
    cv::Mat img;
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(image, "mono8");
    cv_ptr->image.copyTo(img);
    std::vector<cv::Mat> vecMasks = ExtractMask_k(img);
    cv::imwrite(mParams.ImgFolderPath+"/mask.jpg",img);
    for(int i=0; i< vecMasks.size();i++)
    {
        boost::format f = boost::format(mParams.ImgFolderPath+"/mask%d.jpg") %i;
        cv::imwrite(f.str(),vecMasks[i]);
    }
    }

std::vector<cv::Mat> ExtractMask_k(cv::Mat& orgMask)
{
  clock_t start,end;
  start = clock();

  std::vector<cv::Mat> vecMats;
  if (orgMask.type() != CV_8UC1) {
    ROS_ERROR("mask image data type is wrong : %d",orgMask.type());
		return vecMats;
	}

  double minv,maxv;
  cv::Point pt_min,pt_max;
  cv::minMaxLoc(orgMask,&minv,&maxv,&pt_min,&pt_max);

  for(int i=0; i<maxv; i++)
  {
    cv::Mat mask =cv::Mat::zeros(orgMask.rows,orgMask.cols,CV_8UC1);
    vecMats.push_back(mask);
  }

  for(int r = 0; r < orgMask.rows; r++) {
      uchar* ptr = orgMask.ptr<uchar>(r);
      for(int c = 0; c < orgMask.cols; c++) {
          if(ptr[c]!=0){
              uchar* ptr_org = vecMats[ptr[c]-1].ptr<uchar>(r);
              ptr_org[c]=255;
          }
      }
  }
  end = clock();
  ROS_INFO("ExtractMask time =%.3fs",double(end-start)/CLOCKS_PER_SEC);
  return vecMats;
}


*/

    void Detect3DSynchronizer::callback(const DetectionsMsgConstPtr& msg1, const Cloud2MsgConstPtr& msg2){
        std::cout << "receive image" << std::endl;
        auto detections_ptr = std::const_pointer_cast<DetectionsMsg>(msg1);
        auto cloud_msg_ptr = std::const_pointer_cast<Cloud2Msg>(msg2);
        pcl_helper::PclHelper pcl_helper(matchParams.pcl_params_);

        CloudColorPtr cloud_color_ptr = std::make_shared<CloudColor>();
        pcl::fromROSMsg(*cloud_msg_ptr, *cloud_color_ptr);

        int i = 0;
        for (auto detect : detections_ptr->detections){
            RCLCPP_INFO(this->get_logger(), "object detection: lable %s,  score %lf ", detect.label.c_str(), detect.score);
         
            //mask extract
            int height = detect.mask.height;
            int width = detect.mask.width; 
            std::vector<unsigned char> mask = detect.mask.data; 
            
            CloudColorPtr extract_cloud_ptr(new CloudColor);
            pcl_helper.ExtractROI(height, width, mask, cloud_color_ptr, extract_cloud_ptr);
            
            pcl::toROSMsg(*extract_cloud_ptr, extract_cloud_msg);
            extract_cloud_msg.header.frame_id = "camera"; 
            extract_cloud_publiser_->publish(extract_cloud_msg);
            i++;
        }


        /*auto image_msg_ptr  = std::const_pointer_cast<ImageMsg>(msg1);
        auto cloud_msg_ptr = std::const_pointer_cast<Cloud2Msg>(msg2);
        std::cout << "receive image" << std::endl;

        cv::Mat cv_img;
        cv_bridge::CvImagePtr cv_ptr; 
        try{
            cv_ptr = cv_bridge::toCvCopy(image_msg_ptr, image_msg_ptr->encoding);
        }catch(cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv_img = cv_ptr->image;
        std::string image_name_ = "test" + std::to_string(this->get_clock()->now().seconds()) + ".jpg";
        cv::imwrite(image_name_, cv_img);
        try{
            CloudColorPtr cloud_color_ptr = std::make_shared<CloudColor>();
            pcl::fromROSMsg(*cloud_msg_ptr, *cloud_color_ptr);
            //pcl::io::savePCDFileASCII ("test.pcd", *cloud_ptr);

            CloudPNPtr point_source_ptr(new CloudPN);
            pcl_helper::PclHelper pcl_helper(matchParams.pcl_params_);
            //pcl_helper.PointXYZRGBtoPointNormal(*cloud_color_ptr, *point_source_ptr);
            //pcl::io::savePCDFileASCII ("test_pn.pcd", *point_source_ptr);

            //read template 
            CloudPNPtr point_scene_ptr(new CloudPN);
            pcl_helper.loadPointCloud(*point_source_ptr, "/home/sjl/data/hippo2.obj");
            pcl_helper.loadPointCloud(*point_scene_ptr, "/home/sjl/data/hippo1.obj");

            CloudPNPtr point_aligned_ptr(new CloudPN);

            //super4pcs 
            pcl::Super4PCS<pcl::PointNormal,pcl::PointNormal> align;
            auto &options = align.getOptions();
            options.sample_size = matchParams.gr_align_sample_size;
            //options.max_normal_difference = matchParams.gr_max_normal_difference;
            //options.max_color_distance = matchParams.gr_max_color_distance;
            options.max_time_seconds = matchParams.gr_max_time_seconds;
            options.delta = matchParams.gr_delta;
            options.configureOverlap(matchParams.gr_overlap);
            
            // Perform alignment
            pcl::console::print_highlight ("Starting alignment...\n");
            align.setInputSource (point_scene_ptr);
            align.setInputTarget (point_source_ptr);
            {
                pcl::ScopeTime t("Alignment");
                align.align (*point_aligned_ptr);
                RCLCPP_INFO(this->get_logger(), "super4pcs time comsume %f ms", t.getTime());
            }

            Eigen::Matrix4f transformation_model_to_object, transformation_4pcs,transformation_icp;
            if (align.hasConverged())
            { 
                transformation_4pcs = align.getFinalTransformation ();
                RCLCPP_INFO(this->get_logger(), "    | %6.3f %6.3f %6.3f | \n", transformation_4pcs (0,0), transformation_4pcs (0,1), transformation_4pcs (0,2));
                RCLCPP_INFO(this->get_logger(), "R1= | %6.3f %6.3f %6.3f | \n", transformation_4pcs (1,0), transformation_4pcs (1,1), transformation_4pcs (1,2));
                RCLCPP_INFO(this->get_logger(), "    | %6.3f %6.3f %6.3f | \n", transformation_4pcs (2,0), transformation_4pcs (2,1), transformation_4pcs (2,2));
                RCLCPP_INFO(this->get_logger(), "\n");
                RCLCPP_INFO(this->get_logger(), "t1= < %0.3f, %0.3f, %0.3f >\n", transformation_4pcs (0,3), transformation_4pcs (1,3), transformation_4pcs (2,3));
                RCLCPP_INFO(this->get_logger(), "\n");
                // Show alignment
            }
            else
            {
                pcl::console::print_error ("Alignment failed!\n");
                return;
            }
            CloudPNPtr cloud_source_registered(new CloudPN);

            pcl_helper.icp(point_aligned_ptr, point_source_ptr, cloud_source_registered, transformation_icp);
            transformation_model_to_object = transformation_icp * transformation_4pcs;
            //pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");
            //visu.addPointCloud(point_scene_ptr, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(point_scene_ptr, 0.0, 255.0, 0.0), "model");
            //visu.addPointCloud(cloud_source_registered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_source_registered, 0.0, 0.0, 255.0), "object_aligned");
            //visu.addPointCloud(point_source_ptr, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(point_source_ptr, 255.0, 0.0, 0.0), "object");
            //visu.spin();

        }
        catch (pcl::PCLException& e){
            std::cout << "pcl::PCLException: " << e.what() << std::endl;
        }
        */
 
        //process icp
    }

    int Detect3DSynchronizer::gr_match(CloudPtr& inputCloud,CloudPtr& outputCloud)
    {
  
        // Point clouds
        /*clock_t start,end;
        start = clock();

        PointCloudT::Ptr object (new PointCloudT);
        PointCloudT::Ptr object_aligned (new PointCloudT);
        PointCloudT::Ptr cloud_source_registered(new PointCloudT);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object = inputCloud;
        pcl::PointCloud<pcl::Normal>::Ptr normals_object(new pcl::PointCloud<pcl::Normal>);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud_object, *cloud_object, mapping);

        normalEstimation.setInputCloud(cloud_object);
        normalEstimation.setRadiusSearch(mParams.NormalEstimation_RadiusSearch);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod(kdtree);

        // Perform alignment, modle to object
        ROS_INFO ("Starting alignment1...");
        align.setInputSource (scene);
        align.setInputTarget (object);

        clock_t start1,end1;
        start1 = clock();
        align.align (*object_aligned);
        end1 = clock();
        ROS_INFO("Alignment1 time =%.3fs",double(end1-start1)/CLOCKS_PER_SEC);
        // {
        //   pcl::ScopeTime t("Alignment1");
        //   align.align (*object_aligned);
        // }
        
        Eigen::Matrix4f transformation_4pcs,transformation_icp;
        if (align.hasConverged () && align.getFitnessScore()>= mParams.match_min_score)
        { 
            // Print results
            //printf ("\n");
            transformation_4pcs = align.getFinalTransformation ();
            ROS_DEBUG ("    | %6.3f %6.3f %6.3f | \n", transformation_4pcs (0,0), transformation_4pcs (0,1), transformation_4pcs (0,2));
            ROS_DEBUG ("R1= | %6.3f %6.3f %6.3f | \n", transformation_4pcs (1,0), transformation_4pcs (1,1), transformation_4pcs (1,2));
            ROS_DEBUG ("    | %6.3f %6.3f %6.3f | \n", transformation_4pcs (2,0), transformation_4pcs (2,1), transformation_4pcs (2,2));
            ROS_DEBUG ("\n");
            ROS_DEBUG ("t1= < %0.3f, %0.3f, %0.3f >\n", transformation_4pcs (0,3), transformation_4pcs (1,3), transformation_4pcs (2,3));
            ROS_DEBUG ("\n");
            // Show alignment
            //pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");
            //visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
            //visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
            //visu.addPointCloud(object, ColorHandlerT(object, 255.0, 0.0, 0.0), "object");
            //pcl::console::print_highlight("Saving registered cloud to %s ...\n", Demo::defaultPlyOutput.c_str());
            //pcl::io::savePLYFileBinary<PointNT>(Demo::defaultPlyOutput, *object_aligned);

            ROS_INFO("Starting alignment2...");
            pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
            // Set the input source and target, align modle to object
            icp.setInputSource(object_aligned);
            icp.setInputTarget(object);
            // Set the max correspondence distance to 1cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance(mParams.icp_MaxCorrespondenceDistance);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations(mParams.icp_MaximumIterations);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon(mParams.icp_TransformationEpsilon);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp.setEuclideanFitnessEpsilon(mParams.icp_EuclideanFitnessEpsilon);
            // Perform the alignment
            start1 = clock();
            icp.align(*cloud_source_registered);
            end1 = clock();
            ROS_INFO("Alignment2 time =%.3fs",double(end1-start1)/CLOCKS_PER_SEC);
            ROS_INFO("Alignment2 score: %f", icp.getFitnessScore());

            // {
            //   pcl::ScopeTime t("Alignment2");
            //   icp.align(*cloud_source_registered);
            //   ROS_INFO("Alignment2 score: %f", icp.getFitnessScore());
            // }
            // Obtain the transformation that aligned cloud_source to cloud_source_registered
            if (icp.hasConverged())
            {
            //outputCloud = cloud_source_registered;
            pcl::copyPointCloud(*cloud_source_registered,*outputCloud);
            // Print results

            transformation_icp = icp.getFinalTransformation();
            ROS_DEBUG("    | %6.3f %6.3f %6.3f | \n", transformation_icp(0, 0), transformation_icp(0, 1), transformation_icp(0, 2));
            ROS_DEBUG("R2= | %6.3f %6.3f %6.3f | \n", transformation_icp(1, 0), transformation_icp(1, 1), transformation_icp(1, 2));
            ROS_DEBUG("    | %6.3f %6.3f %6.3f | \n", transformation_icp(2, 0), transformation_icp(2, 1), transformation_icp(2, 2));
            ROS_DEBUG("\n");
            ROS_DEBUG("t2= < %0.3f, %0.3f, %0.3f >\n", transformation_icp(0, 3), transformation_icp(1, 3), transformation_icp(2, 3));
            ROS_DEBUG("\n");
            //visu.addPointCloud(cloud_source_registered, ColorHandlerT(cloud_source_registered, 255.0, 255.0, 255.0), "cloud_source_registered");
            
            out_m_ModleToObjInCamcoor = (transformation_icp * transformation_4pcs);
            
            }
            else
            {
            ROS_ERROR("Alignment2 failed!");
            return (-1);
            }
            //visu.spin ();
        }
        else
        {
            ROS_ERROR("Alignment1 failed!");
            return (-1);
        }
        scores.push_back(align.getFitnessScore());
        end = clock();

        ROS_INFO("total match time =%.3fs",double(end-start)/CLOCKS_PER_SEC);
        return (0);
        */
    }


} // namespace name

