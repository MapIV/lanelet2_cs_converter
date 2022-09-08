#include "lanelet2_parser.hpp"
#include <kml_generator/kml_generator.hpp>

int main(int argc, char** argv)
{
  std::string input_osm = argv[1];
  std::string output_kml = argv[2];

  LaneletParser lanelet_parser;
  if (!lanelet_parser.loadOSM(input_osm))
  {
    std::cerr << "\033[31;1mError: Cannot load Lanelet2: " << input_osm << std::endl;
    exit(1);
  }

  KmlGenerator kml_gen(output_kml);

  kml_gen.setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_gen.setLineInterval(0.1);

  for (int i = 0; i < lanelet_parser.size(); i++)
  {
    std::vector<LL2Point> right_lane, left_lane;
    lanelet_parser.getLanePointVec(i, right_lane, left_lane);

    std::vector<kml_utils::Point> right_point_vec;

    for (int j = 0; j < right_lane.size(); j++)
    {
      kml_utils::Point p;
      p.latitude = right_lane[j].lat;
      p.longitude = right_lane[j].lon;
      p.altitude = right_lane[j].elevation;
      right_point_vec.push_back(p);
    }

    kml_gen.addPointVector2LineKML(right_point_vec, (std::string("right_lane_") + std::to_string(i)), 1, KmlGenerator::ColorType::GREEN);

    std::vector<kml_utils::Point> left_point_vec;

    for (int j = 0; j < right_lane.size(); j++)
    {
      kml_utils::Point p;
      p.latitude = left_lane[j].lat;
      p.longitude = left_lane[j].lon;
      p.altitude = left_lane[j].elevation;
      left_point_vec.push_back(p);
    }

    kml_gen.addPointVector2LineKML(left_point_vec, (std::string("left_lane_") + std::to_string(i)), 1, KmlGenerator::ColorType::GREEN);
  }

  kml_gen.outputKml();

  return 0;
}
