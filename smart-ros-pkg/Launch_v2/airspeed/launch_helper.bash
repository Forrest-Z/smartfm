#!/bin/bash
package_path=`rospack find Launch_v2`
package_path=$package_path/airspeed
template_file=$1
generated_xml_file=$package_path/launch_gen/$1
launch_flags_file=$package_path/launch_flags.yaml
parameters_file=$package_path/parameters.yaml
generated_parameters_file=$package_path/launch_gen/parameters.yaml
echo $package_path
echo $template_file
echo $generated_xml_file
echo $launch_flags_file
rosrun Launch_v2 launch_helper.py $package_path $template_file $generated_xml_file $launch_flags_file
rosrun Launch_v2 launch_helper.py $package_path $parameters_file $generated_parameters_file  $launch_flags_file
rosparam load $generated_parameters_file
roslaunch $generated_xml_file
