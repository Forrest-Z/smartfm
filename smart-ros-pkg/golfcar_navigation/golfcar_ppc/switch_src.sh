#!/bin/bash

(
cd src
rm -f golfcar_purepursuit.cpp golfcar_purepursuit.h trajector_planner.cpp
rm build/CMakeFiles/golfcar_ppc.dir/src/*.o
if [ $# -lt 1 -o $1 == "new" ]
then
    echo "Using new code"
    ln -s golfcar_purepursuit.new.cpp golfcar_purepursuit.cpp
    ln -s golfcar_purepursuit.new.h golfcar_purepursuit.h
    ln -s trajector_planner.new.cpp trajector_planner.cpp
elif [ $1 == "bak" -o $1 == "old" ]
then
    echo "Using old code"
    ln -s golfcar_purepursuit.bak.cpp golfcar_purepursuit.cpp
    ln -s golfcar_purepursuit.bak.h golfcar_purepursuit.h
    ln -s trajector_planner.bak.cpp trajector_planner.cpp
else
    echo "Not switching"
fi
)

exit