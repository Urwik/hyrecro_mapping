// C++
#include <filesystem>
#include <cstdlib>
#include <numeric>

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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

  // Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
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


PointCloud::Ptr readCloud(fs::path path)
{
  // Read and save PointCloud depending on the extension of the file
  std::string ext = path.extension();
  PointCloud::Ptr cloud_ptr (new PointCloud);

  if(ext == ".pcd")
  {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path, *cloud_ptr);
  }
  else if ( ext == ".ply")
  {
    pcl::PLYReader ply_reader;
    ply_reader.read(path, *cloud_ptr);
  }
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

  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.filter(*new_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*new_cloud, *outCloud);

  return outCloud;
}

/* // Creates new pointcloud with only point that belongs a plane
Eigen::MatrixXf extractPOI(Eigen::MatrixXf &cloud)
{
  std::vector<int> indices;

  for (size_t i = 0; i < cloud.rows(); i++)
  {
    if(cloud(i, 3) == 1)
      indices.push_back(i);
  }
  
  Eigen::MatrixXf interest_cloud(indices.size(), 3);
  for (size_t i = 0; i < indices.size(); i++)
  {
    interest_cloud(i, 0) = cloud(indices[i], 0);
    interest_cloud(i, 1) = cloud(indices[i], 1);
    interest_cloud(i, 2) = cloud(indices[i], 2);
  }
  
  return interest_cloud;
}

Eigen::MatrixXf cloudToEigen(PointCloud::Ptr &cloud)
{
  Eigen::MatrixXf cloud_Eigen(cloud->points.size(), 4);
  
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    cloud_Eigen(i, 0) = cloud->points[i].x;
    cloud_Eigen(i, 1) = cloud->points[i].y;
    cloud_Eigen(i, 2) = cloud->points[i].z;
    cloud_Eigen(i, 3) = cloud->points[i].label;
  }

  return cloud_Eigen;   
}*/

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

std::vector<pcl::PointIndices> computePlaneClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
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

  return clusters;
}

pcl::ModelCoefficients::Ptr computePlaneCoefs(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const bool optimizeCoefs, float distThreshold = 0.03, int maxIterations = 1000)
{
  pcl::SACSegmentation<pcl::PointXYZ> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices inliers;
  Eigen::Vector4f plane_coefs;

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(inliers, *coefficients);

  return coefficients;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr projectToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::ModelCoefficients::Ptr &plane_coefs)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> project;
  project.setModelType (pcl::SACMODEL_PLANE);
  project.setInputCloud (cloud);
  project.setModelCoefficients (plane_coefs);
  project.filter (*outCloud);

  return outCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeFarthest4Vertex(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  Eigen::Matrix<float, 4, 3> vertex_points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);
  outCloud->width = 4;
  outCloud->height = 1;

  pcl::PointXYZ centroid;
  pcl::computeCentroid<pcl::PointXYZ, pcl::PointXYZ>(*cloud, centroid);

  float max_dist = 0;
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  int index;
  for (size_t j = 0; j < 4; j++)
  {
    for (size_t i=0; i < cloud->points.size(); i++ )
    {
      float dist = pcl::squaredEuclideanDistance<pcl::PointXYZ, pcl::PointXYZ>(centroid, cloud->points[i]);

      if (dist > max_dist)
      {
        max_dist = dist;
        indices->indices[0] = i;
      }
    }
    vertex_points(j,0) = cloud->points[indices->indices[0]].x;
    vertex_points(j,1) = cloud->points[indices->indices[0]].y;
    vertex_points(j,2) = cloud->points[indices->indices[0]].z;

    outCloud->points[j] = cloud->points[indices->indices[0]];

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud);
  }

  return outCloud;
}  


pcl::PointCloud<pcl::PointXYZ>::Ptr computeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud);
  chull.setAlpha(2);
  chull.reconstruct (*cloud_hull);

  return cloud_hull;
}

double computeStdev(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &plane_coefs)
{
  std::vector<double> distances;
  for(auto& point : cloud->points)
  {
    distances.push_back(pcl::pointToPlaneDistance(point, plane_coefs));
  }

  double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
  double mean = sum / distances.size();

  double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / distances.size() - mean * mean);

  return stdev;
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


std::vector<double> computeRandomColor()
{
  std::vector<double> color;

  for (size_t i = 0; i < 3; i++)
  {
    double value = (double) (std::rand() % 255)/255;
    std::cout << value << " ";
    color.push_back(value);
  }
  std::cout << std::endl;


  return color;
}



std::vector<pcl::PointIndices> houghPlaneExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_offline");
  ros::NodeHandle nh;

  PointCloud::Ptr original_cloud (new PointCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr interest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr model_coefs (new pcl::ModelCoefficients);

  fs::path cloud_path = argv[1];
  std::cout << "Segmenting planes in cloud " << argv[1] << std::endl;

  original_cloud = readCloud(cloud_path);
  interest_cloud = extractPOI(original_cloud);

  // segmented_cloud = ransacPlaneExtraction(interest_cloud);
  // segmented_cloud = ransacPlaneSegmentation(interest_cloud);
  // segmented_cloud = regrowPlaneExtraction(interest_cloud);
  std::vector<pcl::PointIndices> indices = computePlaneClusters(interest_cloud);


  // VISUALIZATION

  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");
  int v1(0);
  int v2(0);

  //Define ViewPorts
  vis.createViewPort(0,0,0.5,1, v1);
  vis.createViewPort(0.5,0,1,1, v2);
  vis.removeAllPointClouds();

  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(original_cloud, 100, 100, 100);
  vis.addPointCloud<PointT> (original_cloud, single_color, "original", v1);

  // WORK WITH EACH CLUSTER INDIVIDUALY

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr clust_ind (new pcl::PointIndices);
  extract.setInputCloud(interest_cloud);

  std::stringstream ss;
  int plane_count = 1;
  for (auto& cluster : indices)
  {
    *clust_ind = cluster;
    extract.setIndices(clust_ind);
    extract.filter(*tmp_cloud);

    model_coefs = computePlaneCoefs(tmp_cloud, true, 0.03, 1000);
    planar_cloud = projectToPlane(tmp_cloud, model_coefs);
    vertex_cloud = computeConcaveHull(planar_cloud);

    ss.str("");
    ss << "Poligon_" << plane_count;
    auto color = computeRandomColor();
    
    vis.addPolygon<pcl::PointXYZ>(vertex_cloud, color[0], color[1], color[2], ss.str(), v2);
    plane_count++;
  }
  

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_2(vertex_cloud, 0, 255, 0);
  // vis.addPointCloud<pcl::PointXYZ>(vertex_cloud, color_2, "plane_vertex");
  // vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane_vertex");


  while(!vis.wasStopped())
    vis.spinOnce(100);
  // visualizeClouds(original_cloud, segmented_cloud);

  return 0;
}
