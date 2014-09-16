#!/bin/bash
set -e

cd ~/smartfm
git pull

source env.sh 
cd ~/smartfm/smart-ros-pkg/utown_plaza/run_simulation
source py/bin/activate
source aws_access.sh 

while true; do
    tm-spawn --address=tcp://54.169.35.254:3050 s3bags $(nproc)
    sleep 30
done

