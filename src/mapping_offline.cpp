// C++
#include <filesystem>
#include <cstdlib>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

  // Filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

  // Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

  // Visualization
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// GLOBAL
pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;

struct planeCoefs
{
  float a;
  float b;
  float c;
  float d;
};

PointCloud::Ptr readCloud(fs::path path)
{
  // Read and save PointCloud depending on the extension of the file
  std::string ext = path.extension();
  PointCloud::Ptr cloud_ptr (new PointCloud);

  if(ext == ".pcd")
    pcd_reader.read(path, *cloud_ptr);
  else if ( ext == ".ply")
    ply_reader.read(path, *cloud_ptr);
  else
    ROS_WARN("File %s has not correct extension.", path.filename().c_str());

  return cloud_ptr;
}

// Creates new pointcloud with only point that belongs a plane
pcl::PointCloud<pcl::PointXYZ>::Ptr extractPOI(PointCloud::Ptr &cloud)
{
  PointCloud::Ptr new_cloud (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  pcl::PassThrough<PointT> pass;
  pcl::IndicesPtr indices (new pcl::Indices);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(1,1);
  pass.filter(*indices);


  // int actual_index = 0;
  // for(const auto& point : *cloud)
  // {
  //   if(point.intensity == 1)
  //     indices->push_back(actual_index);

  //   actual_index++;
  // }

  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.filter(*new_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*new_cloud, *outCloud);

  return outCloud;
}


// Plot clouds in two viewports
void visualizeClouds(PointCloud::Ptr &original_cloud, PointCloud::Ptr &filtered_cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  int v1(0);
  int v2(0);

  //Define ViewPorts
  vis.createViewPort(0,0,0.5,1, v1);
  vis.createViewPort(0.5,0,1,1, v2);

  vis.removeAllPointClouds();

  vis.addPointCloud<PointT> (original_cloud, "Original", v1);
  vis.addPointCloud<PointT> (filtered_cloud, "Filtered", v2);

  while(!vis.wasStopped())
    vis.spinOnce(100);

}

// Plot clouds in two viewports
void visualizeClouds(PointCloud::Ptr &original_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &segmented_cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  int v1(0);
  int v2(0);

  //Define ViewPorts
  vis.createViewPort(0,0,0.5,1, v1);
  vis.createViewPort(0.5,0,1,1, v2);

  vis.removeAllPointClouds();

  vis.addPointCloud<PointT> (original_cloud, "Original", v1);
  vis.addPointCloud<pcl::PointXYZRGB> (segmented_cloud, "Segmented", v2);

  while(!vis.wasStopped())
    vis.spinOnce(100);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransacPlaneExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpRGBCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*cloud, *inputCloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> ransac;
  ransac.setInputCloud(inputCloud);
  ransac.setOptimizeCoefficients(true);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(1000);
  ransac.setDistanceThreshold(0.03);
  
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  std::stringstream ss;
  int count = 1;
  while (inputCloud->size() > 0.05*cloud->size())
  {
    ransac.setInputCloud(inputCloud);
    ransac.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() == 0){
      std::cerr << "No se ha podido encontrar ningún plano en la nube." << std::endl;
      break;
    }

    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*tmpCloud);
    extract.setNegative(true);
    extract.filter(*inputCloud);

    pcl::copyPointCloud(*tmpCloud, *tmpRGBCloud);

    int r = std::rand() % 256;
    int g = std::rand() % 256;
    int b = std::rand() % 256;

    for(auto& point : tmpRGBCloud->points)
    {
      point.r = r;
      point.g = g;
      point.b = b;
    }

    *rgbCloud += *tmpRGBCloud;

    count++;
  }
  
  // vis.spin();
  std::cout << "Planes detected: " << count-1 << std::endl;
  return rgbCloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpRGBCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*cloud, *inputCloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> ransac;
  ransac.setInputCloud(inputCloud);
  ransac.setOptimizeCoefficients(true);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(1000);
  ransac.setDistanceThreshold(0.03);
  
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  std::stringstream ss;
  int count = 1;
  while (inputCloud->size() > 0.05*cloud->size())
  {
    ransac.setInputCloud(inputCloud);
    ransac.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() == 0){
      std::cerr << "No se ha podido encontrar ningún plano en la nube." << std::endl;
      break;
    }

    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*tmpCloud);
    extract.setNegative(true);
    extract.filter(*inputCloud);

    pcl::copyPointCloud(*tmpCloud, *tmpRGBCloud);

    //***** Creating the KdTree object for the search method of the extraction *//
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (tmpCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.03);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (tmpCloud);
    ec.extract (cluster_indices);

    //***** Print each cluster in different colors ***************************//
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr tmp_indices (new pcl::PointIndices ());
      
      *tmp_indices = cluster_indices[i];
      
      int r = std::rand() % 256;
      int g = std::rand() % 256;
      int b = std::rand() % 256;
      
      for (auto indice : tmp_indices->indices)
      {
        tmpRGBCloud->points[indice].r = r;
        tmpRGBCloud->points[indice].g = g;
        tmpRGBCloud->points[indice].b = b;
      }
  
    }
    *rgbCloud += *tmpRGBCloud;
  }
  
  std::cout << "Planes detected: " << count-1 << std::endl;
  return rgbCloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr regrowPlaneExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
  //***** Estimación de normales *********************************************//
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(30);
  // ne.setRadiusSearch(0.025);
  ne.compute(*cloud_normals);


  //***** Segmentación basada en crecimiento de regiones *********************//
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (cloud_normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  return colored_cloud;
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr houghPlaneExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
// {
//  int a = 7;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_offline");
  ros::NodeHandle nh;

  PointCloud::Ptr original_cloud (new PointCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);


  fs::path cloud_path = argv[1];

  original_cloud = readCloud(cloud_path);
  filtered_cloud = extractPOI(original_cloud);

  // rgbCloud = ransacPlaneExtraction(filtered_cloud);
  rgbCloud = ransacPlaneSegmentation(filtered_cloud);
  // rgbCloud = regrowPlaneExtraction(filtered_cloud);

  visualizeClouds(original_cloud, rgbCloud);

  return 0;
}
