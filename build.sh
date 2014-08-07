#!/bin/bash

source env.sh
cd ~/smartfm/smart-ros-pkg/third_party/geometry/bullet
make

rosmake ped_momdp_sarsop ped_pathplan golfcar_ppc
