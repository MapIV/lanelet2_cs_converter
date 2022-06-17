cmake_minimum_required(VERSION 2.8.3)
project(lanelet2_cs_converter)

# Enable support for C++14
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=c++14")
else ()
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  llh_converter
)
# find_package(TinyXML2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS llh_converter
  # INCLUDE_DIRS include
  # DEPENDS TinyXML
)

# include_directories(${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(jprcs src/jprcs.cpp src/tinyxml2.cpp)
# target_link_libraries(jprcs ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
target_link_libraries(jprcs ${catkin_LIBRARIES})
add_dependencies(jprcs ${catkin_EXPORTED_TARGETS})
