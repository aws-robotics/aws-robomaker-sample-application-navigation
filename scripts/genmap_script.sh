#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

# This script generates map files for the input world

# Input args via command line:
#   <world-name>  [required] 

set -e

if [ $# -ne 1 ]; then
    echo "Expect 1 argument";
    exit 2;
fi
        
if [ $1 = "worldforge" ] && [ -z "$WORLD_ID" ]; then
    echo "For WorldForge world, please set WORLD_ID to your worldforge world as per the README instructions"
    exit 2;
fi

echo "Sudo password may be needed to install system dependencies"
sudo apt-get install ruby-dev libxml-xpath-perl libxml2-utils

cd simulation_ws
vcs import < .rosinstall
rosdep install --from-paths src --ignore-src -r -y
cd ..

set +e

OUTPUT=`python scripts/add_map_plugin.py default --world_name $1`

if [ $? -eq 2 ]
then
	echo $OUTPUT
	exit $?
else
	world_source_path=$OUTPUT
fi

set -e

cd simulation_ws
colcon build
source install/local_setup.sh
cd ..

map_output_path=$(dirname $(dirname $world_source_path))/maps/map

roslaunch navigation_simulation start_map_service.launch &

python << END
import rospy

rospy.wait_for_service('/gazebo_2Dmap_plugin/generate_map')
END

rosservice call /gazebo_2Dmap_plugin/generate_map
rosrun map_server map_saver -f $map_output_path /map:=/map2d

cd simulation_ws
colcon build

kill $!

echo "--- Map file generated at $map_output_path"
