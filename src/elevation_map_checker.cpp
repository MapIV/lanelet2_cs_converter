#include <tinyxml2.h>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>

#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_checker");
  ros::NodeHandle nh;

  ros::Publisher elev_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher og_pub = nh.advertise<nav_msgs::OccupancyGrid>("og_map", 1, true);
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

  double resolution = elevation_map.getResolution();
  int x_size = elevation_map.getLength()[0];
  int y_size = elevation_map.getLength()[1];
  int x_origin = elevation_map.getPosition()[0];
  int y_origin = elevation_map.getPosition()[1];
  int z_origin = elevation_map.atPosition(std::string("elevation"), elevation_map.getPosition());

  std::cout << "Length: " << x_size << ", " << y_size << std::endl;
  std::cout << "Position: " << x_origin << ", " << y_origin << std::endl;
  std::cout << "Resolution: " << resolution << std::endl;

  nav_msgs::OccupancyGrid og;
  og.header.seq = 0;
  og.header.stamp = ros::Time::now();
  og.header.frame_id = "map";

  og.info.map_load_time = og.header.stamp;
  og.info.resolution = resolution;
  og.info.width = x_size / resolution;
  og.info.height = y_size / resolution;

  int width = og.info.width;
  int height = og.info.height;

  geometry_msgs::Point p;
  p.x = x_origin - (x_size / 2);
  p.y = y_origin - (y_size / 2);
  p.z = z_origin;

  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  q.w = 1;

  og.info.origin.position = p;
  og.info.origin.orientation = q;

  og.data.resize(width * height);

  double value_max = -1;
  double value_min = 100000;

  for (int i = 0; i < width * height; i++)
  {
    int id_x = i % width;
    int id_y = i / width;
    double x = p.x + id_x * resolution;
    double y = p.y + id_y * resolution;
    grid_map::Position pos(x, y);
    if (elevation_map.isInside(pos))
    {
      double value = elevation_map.atPosition(std::string("elevation"), pos);
      if (std::isnan(value))
        og.data[i] = -1;
      else
      {
        og.data[i] = value;
        if (value > value_max) value_max = value;
        if (value < value_min) value_min = value;
      }
    }
    else
      og.data[i] = -1;
  }

  std::cout << "value_max: " << value_max << std::endl;
  std::cout << "value_min: " << value_min << std::endl;

  value_min = value_min + 7;
  value_max = value_min + 3;

  for (int i = 0; i < width * height; i++)
  {
    if (og.data[i] != -1)
    {
      og.data[i] = (og.data[i] - value_min) / (value_max - value_min) * 100;
      if (og.data[i] < 0) og.data[i] = 0;
      if (og.data[i] > 100) og.data[i] = 100;
    }
  }

  elev_pub.publish(msg);
  og_pub.publish(og);
  std::cout << "published" << std::endl;

  grid_map::Position pos(p.x + 5, p.y + 5);
  std::cout << "cell: " << elevation_map.atPosition(std::string("elevation"), pos) << std::endl;

  ros::spin();

  return 0;
}
