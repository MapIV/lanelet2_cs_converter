#include <iostream>

#include "lanelet2.hpp"
#include "tinyxml2.h"

int main(int argc, char** argv)
{
  std::string input_osm = argv[1];

  LL2Points ll2points;

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
  ll2points.printVec();

  return 0;
}
