#!/bin/bash
set -e

source env.sh

function build_bookingui {
    ln -s ~/smartfm/bookingUI/shared/SvgPath ~/
    cd ~/smartfm/bookingUI/shared
    cmake .
    make
}

cd ~/smartfm/smart-ros-pkg/third_party/geometry/bullet
make

[ -e ~/SvgPath ] || build_bookingui

rosmake ped_momdp_sarsop ped_pathplan golfcar_ppc velmixer

