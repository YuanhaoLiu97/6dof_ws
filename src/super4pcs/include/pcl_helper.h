#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
# include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>

namespace pcl_helper
{

  struct PclParams
{
    double normal_estimate_radius_search = 0.0013; // m
    double voxel_leaf_size = 0.002;
    int sor_meank = 400; 
    double sor_stddev_multhresh=0.3;
    double icp_max_correspondence_distance = 0.01; //m
    int icp_maximum_iterations = 30;
    double icp_transformation_epsilon = 1e-8;
    double icp_euclidean_fitness_epsilon = 0.001;
};

class PclHelper
{
private:

    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudColor;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudColorPtr;

    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

    typedef pcl::PointCloud<pcl::Normal> CloudN;
    typedef pcl::PointCloud<pcl::Normal>::Ptr CloudNPtr;

    typedef pcl::PointCloud<pcl::PointNormal> CloudPN;
    typedef pcl::PointCloud<pcl::PointNormal>::Ptr CloudPNPtr;

    PclParams pcl_params;

public:
  PclHelper(PclParams pcl_params)
  { 
    this->pcl_params = pcl_params;
  }

  ~PclHelper()
  {
  }

  void PointXYZRGBtoXYZ(const CloudColor& cloudColor, Cloud& cloud)
  {
    cloud.height = cloudColor.height;
    cloud.width = cloudColor.width;
    cloud.is_dense=false;
    for (const auto& point : cloudColor)
    {
      pcl::PointXYZ p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      cloud.push_back(p);
    }
  }

  void loadPointCloud(CloudPN& cloud_pn_, std::string file_path){
      //read point cloud
      //pcl::PointCloud<pcl::PointXYZ> cloud;
      if(pcl::io::loadOBJFile<pcl::PointNormal>(file_path, cloud_pn_)== -1){
          return;
      }

      //PointXYZtoPointNormal(cloud, cloud_pn_);
  }

  void PointXYZRGBtoPointNormal(const CloudColor& cloudColor, CloudPN& cloudPN){
    CloudPtr cloud_source_ptr(new Cloud);
    PointXYZRGBtoXYZ(cloudColor, *cloud_source_ptr);
    CloudNPtr normal_ptr(new CloudN);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_source_ptr, *cloud_source_ptr, mapping);
    normalEstimation.setInputCloud(cloud_source_ptr);
    normalEstimation.setRadiusSearch(pcl_params.normal_estimate_radius_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normal_ptr);        
    pcl::concatenateFields(*cloud_source_ptr, *normal_ptr, cloudPN);
  }

  void ExtractROI(int height, int width, std::vector<unsigned char> mask, CloudColorPtr cloud_color_ptr, CloudColorPtr outputCloud){
      std::vector<int> indexs;
      for(int h = 0; h < height; h++) {
          for(int w = 0; w < width; w++) {
              unsigned char unit = mask[h*width + w];
              if(unit !=0 ){
                  indexs.push_back(h*width + w);           
              }
          }
      }
      pcl::IndicesPtr index_ptr = std::make_shared<std::vector<int>>(indexs);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_color_ptr);
      extract.setIndices(index_ptr);
      extract.setNegative(false);
      extract.filter(*outputCloud);            
  }

  void PointXYZtoPointNormal(Cloud& cloud, CloudPN& cloudPN){
    CloudNPtr normal_ptr(new CloudN);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(cloud, cloud, mapping);
    normalEstimation.setInputCloud(std::make_shared<Cloud>(cloud));
    normalEstimation.setRadiusSearch(pcl_params.normal_estimate_radius_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normal_ptr);        
    pcl::concatenateFields(cloud, *normal_ptr, cloudPN);
  }

  void icp(CloudPNPtr object_aligned, CloudPNPtr object, CloudPNPtr cloud_source_registered, Eigen::Matrix4f &transformation_icp){
      pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
      // Set the input source and target, align modle to object
      icp.setInputSource(object_aligned);
      icp.setInputTarget(object);
      // Set the max correspondence distance to 1cm (e.g., correspondences with higher distances will be ignored)
      icp.setMaxCorrespondenceDistance(pcl_params.icp_max_correspondence_distance);
      // Set the maximum number of iterations (criterion 1)
      icp.setMaximumIterations(pcl_params.icp_maximum_iterations);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon(pcl_params.icp_transformation_epsilon);
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon(pcl_params.icp_euclidean_fitness_epsilon);
      // Perform the alignment
      icp.align(*cloud_source_registered);
      if (icp.hasConverged())
      {
      //outputCloud = cloud_source_registered;
      //pcl::copyPointCloud(*cloud_source_registered,*outputCloud);
      // Print results

      transformation_icp = icp.getFinalTransformation();
      //ROS_DEBUG("    | %6.3f %6.3f %6.3f | \n", transformation_icp(0, 0), transformation_icp(0, 1), transformation_icp(0, 2));
      //ROS_DEBUG("R2= | %6.3f %6.3f %6.3f | \n", transformation_icp(1, 0), transformation_icp(1, 1), transformation_icp(1, 2));
      //ROS_DEBUG("    | %6.3f %6.3f %6.3f | \n", transformation_icp(2, 0), transformation_icp(2, 1), transformation_icp(2, 2));
      //ROS_DEBUG("\n");
      //ROS_DEBUG("t2= < %0.3f, %0.3f, %0.3f >\n", transformation_icp(0, 3), transformation_icp(1, 3), transformation_icp(2, 3));
      //ROS_DEBUG("\n");
      }
      else
      {
      //ROS_ERROR("Alignment2 failed!");
      return;
      }
  }
};

}  // namespace pcl_helper
