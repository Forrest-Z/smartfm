#!/bin/bash

echo $ROS_ROOT | grep $ROS_DISTRO >/dev/null 2>&1 \
&& . ${ROS_ROOT}/../rosbash/rosbash \
|| . ${ROS_ROOT}/tools/rosbash/rosbash

gitrootdir=`git rev-parse --show-toplevel`

echo cleaning bookingUI/shared
(
cd $gitrootdir/bookingUI/shared
make clean 2>/dev/null
/bin/rm -Rf CMakeCache.txt  CMakeFiles  cmake_install.cmake  libbookingcommon.so  Makefile test_DBInterface  test_SvgPath

for package in `rospack list | grep smartfm | cut -d' ' -f1 | xargs echo`
do
    echo cleaning package $package
    roscd $package
    rosmake --target=clean > /dev/null 2>&1
done

)
