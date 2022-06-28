#include <tinyxml2.h>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_msgs/GridMap.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_checker");
  ros::NodeHandle nh;

  std::string input_pcd = argv[1];
  // std::string input_osm = argv[2];
  std::string config_file = argv[2];

  pcl::shared_ptr<grid_map::GridMapPclLoader> grid_map_pcl_loader;
  std::cout << "ok" << std::endl;
  grid_map_pcl_loader->loadParameters(config_file);
  std::cout << "ok" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(input_pcd, *cloud);
  grid_map_pcl_loader->setInputCloud(cloud);

  return 0;
}
