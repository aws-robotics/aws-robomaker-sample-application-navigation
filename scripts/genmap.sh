#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

# This script adds map generation plugin to the input world and plugin config parameters

# Input args via command line:
#   <path-to-config>  [required] 
#   <path-to-world-file>  [required]
#   <path-to-output-file>  [required]

set -e

if [ $# -ne 3 ]; then
    echo "expects 3 arguments"
    exit 2
fi

worldfile=$2
config=$1
worldbody=`xpath -q -e '/sdf/world/*' $worldfile`
template="""
<%
%>
<sdf version='1.6'>
  <world name='default'>
    $worldbody
    <plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
         <map_resolution><%= @map_res %></map_resolution>
         <map_height>0.3</map_height>
         <map_size_x><%= @map_size_x %></map_size_x>
         <map_size_y><%= @map_size_y %></map_size_y>
         <init_robot_x><%= @robot_init_x %></init_robot_x>
         <init_robot_y><%= @robot_init_y %></init_robot_y>
     </plugin>
 </world>
</sdf>
"""
echo $template | erb -r "$config"  | xmllint --format - > $3
echo $3
