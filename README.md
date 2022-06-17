# lanelet2_cs_converter

(Updated 2022/06/17)

## Install

```
$ mkdir -p lanelet2_ws/src
$ cd lanelet2_ws/src
$ git clone https://gitlab.com/MapIV/map4_engine.git
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

## LICENSE

This repository contains copies from [tinyxml2](https://github.com/leethomason/tinyxml2) which is under Zlib license.

* [tinyxml2.cpp](src/tinyxml2.cpp)
* [tinyxml2.h](include/tinyxml2.h)

