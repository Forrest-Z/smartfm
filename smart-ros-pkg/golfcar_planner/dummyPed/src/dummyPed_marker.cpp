/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <sensing_on_road/pedestrian_laser_batch.h>
using namespace visualization_msgs;
using namespace std;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;
interactive_markers::MenuHandler menu_handler;
ros::Publisher Pedpub_;
geometry_msgs::Point ped1, ped2, ped3;
// %EndTag(vars)%

void publish_ped()
{
    sensing_on_road::pedestrian_laser_batch psglaser_batch;
    sensing_on_road::pedestrian_laser psglaser;

    psglaser.pedestrian_laser.point = ped1;
    psglaser.object_label = 1;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    psglaser.pedestrian_laser.point = ped2;
    psglaser.object_label = 2;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    psglaser.pedestrian_laser.point = ped3;
    psglaser.object_label = 3;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    Pedpub_.publish(psglaser_batch);
}

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    ostringstream s;
    if(feedback->marker_name == "Ped1") ped1 = feedback->pose.position;
    else if(feedback->marker_name == "Ped2") ped2 = feedback->pose.position;
    else if(feedback->marker_name == "Ped3") ped3 = feedback->pose.position;

    publish_ped();

    server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg, std_msgs::ColorRGBA &color )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color = color;

    return marker;
}

// %EndTag(Box)%

void saveMarker( InteractiveMarker int_marker )
{
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////


// %Tag(ViewFacing)%
void makeViewFacingMarker(std_msgs::ColorRGBA color, string name, geometry_msgs::Point &start_pose)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.pose.position = start_pose;
    int_marker.scale = 1;

    int_marker.name = name;
    int_marker.description = name;

    InteractiveMarkerControl control;

    // make a control that rotates around the view axis
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    control.name = "rotate";

    int_marker.controls.push_back(control);

    // create a box in the center which should not be view facing,
    // but move in the camera plane.
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.independent_marker_orientation = true;
    control.name = "move";

    control.markers.push_back( makeBox(int_marker, color) );
    control.always_visible = true;

    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%

// %Tag(main)%
int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_controls");
    ros::NodeHandle n;

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
    Pedpub_ = n.advertise<sensing_on_road::pedestrian_laser_batch>("ped_map_laser_batch", 2);
    ros::Duration(0.1).sleep();
    std_msgs::ColorRGBA color;
    ped1.x = 1.0; ped1.y = 1.0;
    color.r = 1.0; color.a = 1.0;
    makeViewFacingMarker(color, string("Ped1"), ped1);
    ped2.x = 10.0; ped2.y = 1.0;
    color.r = 0.0; color.g = 1.0;
    makeViewFacingMarker(color, string("Ped2"), ped2);
    ped3.x = 10.0; ped3.y = 7.0;
    color.g = 0.0; color.b = 1.0;
    makeViewFacingMarker(color, string("Ped3"), ped3);

    server->applyChanges();

    ros::spin();

    server.reset();
}
// %EndTag(main)%
