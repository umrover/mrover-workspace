#!/bin/bash

DIR=$(dirname $0)

source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(realpath ${DIR}/../lib)

gzserver --verbose "$1" &
GZSERVER_PID=$!
gzweb &
GZWEB_PID=$!

wait $GZSERVER_PID $GZWEB_PID
