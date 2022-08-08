cmake_minimum_required(VERSION 2.8.3)
project(lanelet2_cs_converter)

# Enable support for C++14
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=c++14")
else ()
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  llh_converter
  grid_map_core
  grid_map_pcl
  nav_msgs
  # lanelet2_io
  # lanelet2_core
  # lanelet2_projection
)
# find_package(TinyXML2 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS llh_converter
  DEPENDS PCL
  # INCLUDE_DIRS include
  # DEPENDS TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(jprcs src/jprcs.cpp src/tinyxml2.cpp)
target_link_libraries(jprcs ${catkin_LIBRARIES})
add_dependencies(jprcs ${catkin_EXPORTED_TARGETS})

add_executable(elevation_map_checker
  src/elevation_map_checker.cpp
  src/lanelet2_parser.cpp
  src/tinyxml2.cpp)
target_link_libraries(elevation_map_checker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(elevation_map_checker ${catkin_EXPORTED_TARGETS})
