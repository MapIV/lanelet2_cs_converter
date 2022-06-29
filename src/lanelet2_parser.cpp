#include <iostream>

// #include <lanelet2_core/primitives/Lanelet.h>
// #include <lanelet2_io/Io.h>
// #include <lanelet2_io/io_handlers/Factory.h>
// #include <lanelet2_io/io_handlers/Writer.h>
// #include <lanelet2_projection/UTM.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "lanelet2.hpp"
#include "tinyxml2.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lanelet2_parser");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("lanelet", 1, true);
  std::string input_osm = argv[1];

  // lanelet::projection::UtmProjector projector(lanelet::Origin({49, 8.4}));
  // lanelet::LaneletMapPtr map = lanelet::load(input_osm, projector);

  LL2Points ll2points;
  LL2LineStrings ll2linestrings;

  tinyxml2::XMLDocument doc;
  doc.LoadFile(input_osm.c_str());

  tinyxml2::XMLElement* root = doc.FirstChildElement("osm");
  if (root == NULL)
  {
    std::cerr << "\033[31;1mError: OSM NULL\033[m" << std::endl;
    exit(1);
  }

  for (tinyxml2::XMLElement* element = root->FirstChildElement("node"); element != NULL; element = element->NextSiblingElement("node"))
  {
    LL2Point p;
    double x, y;
    for (tinyxml2::XMLElement* tag = element->FirstChildElement("tag"); tag != NULL; tag = tag->NextSiblingElement("tag"))
    {
      if (std::string(tag->Attribute("k")) == std::string("local_x")) p.local_x = tag->DoubleAttribute("v");
      if (std::string(tag->Attribute("k")) == std::string("local_y")) p.local_y = tag->DoubleAttribute("v");
      if (std::string(tag->Attribute("k")) == std::string("ele")) p.elevation = tag->DoubleAttribute("v");
    }

    p.id = element->UnsignedAttribute("id");
    p.lat = element->DoubleAttribute("lat");
    p.lon = element->DoubleAttribute("lon");

    p.valid = true;

    ll2points.addNewPoint(p);
  }

  ll2points.makeVec();

  for (tinyxml2::XMLElement* element = root->FirstChildElement("way"); element != NULL; element = element->NextSiblingElement("way"))
  {
    LL2LineString ls;
    ls.id = element->UnsignedAttribute("id");

    for (tinyxml2::XMLElement* tag = element->FirstChildElement("nd"); tag != NULL; tag = tag->NextSiblingElement("nd"))
    {
      unsigned int id = tag->UnsignedAttribute("ref");
      ls.id_vec.push_back(id);
    }
    ls.valid = true;

    ll2linestrings.addNewLineString(ls);
  }


  visualization_msgs::Marker ls_msg;
  ls_msg.header.frame_id = "map";
  ls_msg.header.stamp = ros::Time::now();
  ls_msg.action = visualization_msgs::Marker::ADD;
  ls_msg.pose.orientation.w = 1.0;
  ls_msg.id = 0;
  ls_msg.type = visualization_msgs::Marker::LINE_LIST;
  ls_msg.scale.x = 0.2;
  ls_msg.color.g = 1.0;
  ls_msg.color.a = 1.0;

  for (int k = 0; k < ll2linestrings.size(); k++)
  {
    LL2LineString line_string = ll2linestrings.getLineStringBySeq(k);

    for (int i = 0; i < line_string.id_vec.size(); i++)
    {
      LL2Point p = ll2points.getPoint(line_string.id_vec[i]);
      geometry_msgs::Point q;
      q.x = p.local_x;
      q.y = p.local_y;
      q.z = p.elevation;
      if (i != 0 && i != line_string.id_vec.size() - 1)
        ls_msg.points.push_back(q);
      ls_msg.points.push_back(q);
    }
  }

  pub.publish(ls_msg);

  ros::spin();

  return 0;
}
