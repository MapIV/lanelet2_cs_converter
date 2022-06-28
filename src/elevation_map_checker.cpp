#include <tinyxml2.h>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>

#include <grid_map_msgs/GridMap.h>

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_checker");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  std::string input_pcd = argv[1];
  // std::string input_osm = argv[2];
  std::string config_file = argv[2];

  gm::setVerbosityLevelToDebugIfFlagSet(nh);

  grid_map::GridMapPclLoader grid_map_pcl_loader;
  std::cout << "ok" << std::endl;
  grid_map_pcl_loader.loadParameters(config_file);
  std::cout << "ok" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(input_pcd, *cloud);
  grid_map_pcl_loader.setInputCloud(cloud);

  grid_map_pcl_loader.preProcessInputCloud();
  std::cout << "preprocess" << std::endl;
  grid_map_pcl_loader.initializeGridMapGeometryFromInputCloud();
  std::cout << "init" << std::endl;
  grid_map_pcl_loader.addLayerFromInputCloud(std::string("elevation"));

  grid_map::GridMap elevation_map = grid_map_pcl_loader.getGridMap();
  elevation_map.setFrameId(std::string("map"));

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(elevation_map, msg);
  msg.info.header.seq = 0;
  msg.info.header.stamp = ros::Time::now();

  pub.publish(msg);
  std::cout << "published" << std::endl;

  grid_map::Position pos(15797., 38057.);
  std::cout << "cell: " << elevation_map.atPosition(std::string("elevation"), pos) << std::endl;

  ros::spin();

  return 0;
}
