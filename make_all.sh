#!/bin/bash

# This script will attempt to build all our packages (and bookingUI/shared)
# It does not necessary succeed in building all of them, as dependencies
# might be missing.

# top directory of the git repository
gitrootdir=`git rev-parse --show-toplevel`

# record the current dir so that we can come back to it later
curdir=`pwd`

# How many processors do we have? Will be used to speed up compilation.
# NOTE: there might be a more portable way of doing this...
ncpus=`cat /proc/cpuinfo | grep processor | wc -l`

# Leave one out though, so that we can continue to do some stuffs
ncpus=$(($ncpus-1))

# Build the bookingUI/shared library
cd $gitrootdir/bookingUI/shared
cmake .
# the -j flag allows to parallelize the process
if [ $ncpus -gt 2 ]; then
    make -j $ncpus
else
    make
fi

packages=`rospack list | grep smart-ros-pkg | cut -d' ' -f1 | grep -v Launch | xargs echo`
cmd="rosmake --robust"
if [ $ncpus -gt 2 ]; then
    cmd="$cmd --threads=$ncpus"
fi

logfile=`mktemp --tmpdir=/tmp make_all_log.XXXXXX`
$cmd $packages | tee $logfile

echo
echo
echo ===========================================================================
echo SUMMARY:
echo FAILED: `grep FAIL $logfile | cut -d'<' -f4 | cut -d' ' -f2 | xargs echo`
echo PASS: `grep PASS $logfile | cut -d'<' -f4 | cut -d' ' -f2 | xargs echo`
echo
echo logged in $logfile

cd $curdir
