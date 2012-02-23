#!/bin/bash

. ${ROS_ROOT}/tools/rosbash/rosbash

gitrootdir=`git rev-parse --show-toplevel`
curdir=`pwd`

echo cleaning bookingUI/shared
cd $gitrootdir/bookingUI/shared
make clean 2>/dev/null
rm -Rf CMakeCache.txt  CMakeFiles  cmake_install.cmake  libbookingcommon.so  Makefile test_DBInterface  test_SvgPath

for package in `rospack list | grep smart-ros-pkg | cut -d' ' -f1 | xargs echo`
do
    echo cleaning package $package
    roscd $package
    make clean > /dev/null 2>&1
done

cd $curdir
