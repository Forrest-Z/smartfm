#!/bin/bash

# This script will attempt to build all our packages (and bookingUI/shared)
# It does not necessary succeed in building all of them, as dependencies
# might be missing.

# top directory of the git repository
gitrootdir=`git rev-parse --show-toplevel`

# How many processors do we have? Will be used to speed up compilation.
# NOTE: there might be a more portable way of doing this...
ncpus=`cat /proc/cpuinfo | grep processor | wc -l`

# Leave one out though, so that we can continue to do some stuffs
ncpus=$(($ncpus-1))

logfile=`mktemp --tmpdir=/tmp make_all_log.XXXXXX`
echo logging to $logfile




tee $logfile <<EOF

Building the bookingUI/shared library
-------------------------------------

EOF


# Do the compilation in a subshell
(
cd $gitrootdir/bookingUI/shared
cmake .

makecmd="make"

# the -j flag allows to parallelize the process
if [ $ncpus -gt 2 ]; then
    makecmd="$makecmd -j $ncpus"
fi

$makecmd

) 2>&1 | tee $logfile


if [ ${PIPESTATUS[0]} -eq 0 ] ; then
    compiling_bookinUI_share=0
else
    compiling_bookinUI_share=1
    cat <<EOF


Error: Could not compile bookingUI/share
----------------------------------------

EOF

    read -p "Ignore and continue [y/n]?"
    if [ $REPLY != "y" ] ; then
        echo Aborting | tee $logfile
        exit 1
    else
        echo Ignoring | tee $logfile
    fi
fi




tee $logfile <<EOF

================================================================================

Building all ROS packages
-------------------------

EOF


packages=`rospack list | grep smart-ros-pkg | cut -d' ' -f1 | grep -v Launch | xargs echo`
cmd="rosmake --robust"
if [ $ncpus -gt 2 ]; then
    cmd="$cmd --threads=$ncpus"
fi

$cmd $packages | tee $logfile


# [rosmake-0] Finished <<< pkg_name [PASS/FAIL] [ x.xx seconds ]
failed=`grep rosmake $logfile | grep Finished | grep FAIL | cut -d'<' -f4 | cut -d' ' -f2 | xargs echo`
pass=`grep rosmake $logfile | grep Finished | grep PASS | cut -d'<' -f4 | cut -d' ' -f2 | xargs echo`

if [ $compiling_bookinUI_share -eq 0 ] ; then
    pass="bookingUI/share $pass"
else
    failed="bookingUI/share $failed"
fi


tee $logfile <<EOF


================================================================================
SUMMARY:
--------

FAILED: $failed

PASS: $pass


EOF

echo This was logged in $logfile
