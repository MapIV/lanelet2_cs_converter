# lanelet2_cs_converter

[![Build and Test](https://github.com/MapIV/lanelet2_cs_converter/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/MapIV/lanelet2_cs_converter/actions/workflows/build_and_test.yaml)

(Updated 2022/08/09)

## Install

```
$ mkdir -p lanelet2_ws/src
$ cd lanelet2_ws/src
$ git clone https://github.com/MapIV/lanelet2_cs_converter.git
$ git clone https://github.com/MapIV/llh_converter.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## jprcs

This process re-calculate the Lat/Lon from JPRCS coordinates.

### Usage

```
$ ./devel/lib/lanelet2_cs_converter/jprcs <INPUT_OSM> <OUTPUT_OSM> <PLANE_NUM>
```

## elevation_map_checker

Visualize elevation_map info with 2D OccupancyGrid map. Detailed configuration is in [sample.yaml](./config/sample.yaml).

### Usage

```
$ ./src/lanelet2_cs_converter/scripts/elevation_map.sh <PCD> <OSM>
```

## LICENSE

This repository contains copies from [tinyxml2](https://github.com/leethomason/tinyxml2) which is under Zlib license.

* [tinyxml2.cpp](src/tinyxml2.cpp)
* [tinyxml2.h](include/tinyxml2.h)

