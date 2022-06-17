#include <iostream>
#include <iomanip>
#include <fstream>

#include "tinyxml2.h"

#include <llh_converter/llh_converter.hpp>

#define ARG_NUM 3

int main(int argc, char** argv)
{
  std::string input_osm, output_osm;
  int plane_num;

  if (argc != ARG_NUM + 1)
  {
    std::cerr << "\033[31;1mError: Invalid Arguments" << std::endl;
    std::cerr << argv[0] << " <INPUT_OSM> <OUTPUT_OSM> <PLANE_NUM>\033[m" << std::endl;
    exit(1);
  }

  input_osm = argv[1];
  output_osm = argv[2];
  plane_num = std::stoi(argv[3]);

  llh_converter::LLHConverter llh_conv;
  llh_converter::LLHParam llh_param;
  llh_param.plane_num = plane_num;
  llh_param.use_mgrs = false;

  llh_converter::LLHParam mgrs_param;
  mgrs_param.use_mgrs = true;
  mgrs_param.height_convert_type = llh_converter::ConvertType::NONE;

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
    double x, y;
    for (tinyxml2::XMLElement* tag = element->FirstChildElement("tag"); tag != NULL; tag = tag->NextSiblingElement("tag"))
    {
      if (std::string(tag->Attribute("k")) == std::string("local_x")) x = tag->DoubleAttribute("v");
      if (std::string(tag->Attribute("k")) == std::string("local_y")) y = tag->DoubleAttribute("v");
    }

    double lat_deg, lon_deg;
    llh_conv.revertXYZ2Deg(x, y, lat_deg, lon_deg, llh_param);
    element->SetAttribute("lat", lat_deg);
    element->SetAttribute("lon", lon_deg);
    double mgrs_x, mgrs_y, mgrs_z;
    llh_conv.convertDeg2XYZ(lat_deg, lon_deg, 0, mgrs_x, mgrs_y, mgrs_z, mgrs_param);

    std::string grid_code = llh_conv.getMGRSGridCode();
    int int_x = std::floor(mgrs_x / 100);
    int int_y = std::floor(mgrs_y / 100);

    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << int_x;
    ss << std::setw(3) << std::setfill('0') << int_y;

    std::string mgrs_code = grid_code + ss.str();

    for (tinyxml2::XMLElement* tag = element->FirstChildElement("tag"); tag != NULL; tag = tag->NextSiblingElement("tag"))
    {
      if (std::string(tag->Attribute("k")) == std::string("mgrs_code")) tag->SetAttribute("v", mgrs_code.c_str());
    }
  }

  doc.SaveFile(output_osm.c_str());

  return 0;
}
