#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=${SCRIPT_DIR%/*}
CONFIG_FILE=$BASE_DIR"/config/sample.yaml"

function usage() {
cat <<_EOT_
Usage:
  $0 <PCD> <LANELET>

Description:
  Visualize elevation_map

Options:
  * -h: Show usage

_EOT_

exit 0
}

# Parse option
if [ "$OPTIND" = 1 ]; then
  while getopts h OPT
  do
    case $OPT in
      h)
        usage ;;
      \?)
        echo "Undefined option $OPT"
        usage ;;
    esac
  done
else
  echo "No installed getopts-command." 1>&2
  exit 1
fi
shift $(($OPTIND - 1))

PCD_FILE=$1
OSM_FILE=$2

rviz -d ${BASE_DIR}"/rviz/elevation_map.rviz" &

sleep1

rosrun lanelet2_cs_converter elevation_map_checker $PCD_FILE $OSM_FILE $CONFIG_FILE
