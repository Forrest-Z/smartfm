#!/bin/bash

package_path=`rospack find etherlab_master`
motor_port=1
bash $package_path/scripts/0x1600.bash $motor_port
bash $package_path/scripts/0x1601.bash $motor_port
bash $package_path/scripts/0x1a01.bash $motor_port
cmd="/opt/etherlab/bin/ethercat download --position=$motor_port --type "
#everything in 0.1 deg and secs
$cmd uint32 0x6093 1 8192
$cmd uint32 0x6093 2 25
$cmd uint32 0x6094 1 3
$cmd uint32 0x6094 2 10
$cmd uint32 0x6097 1 384
$cmd uint32 0x6097 2 5
$cmd uint32 0x2090 2 144000
$cmd uint32 0x2090 3 144000
$cmd uint32 0x2090 4 144000
$cmd uint32 0x2090 5 144000

$cmd uint32 0x60ff 0 0

