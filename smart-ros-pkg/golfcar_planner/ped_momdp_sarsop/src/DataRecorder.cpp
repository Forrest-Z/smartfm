#include <ros/ros.h>
void pedestrianCallBack(ped_momdp_sarsop::ped_local_frame_vector lPedLocal)
{
	ped_momdp_sarsop::ped_local_frame ped=lPedLocal.ped_local[0];
	Car world_car;
	world_car.w=ped.rob_pose.x*ModelParams::map_rln;
	world_car.h=ped.rob_pose.y*ModelParams::map_rln;
	vector<Pedestrian> ped_list;
    for(int ii=0; ii< lPedLocal.ped_local.size(); ii++)
    {
		geometry_msgs::Point32 p;
		Pedestrian world_ped;
		ped_momdp_sarsop::ped_local_frame ped=lPedLocal.ped_local[ii];
		world_ped.id=ped.ped_id;
		world_ped.w = ped.ped_pose.x*ModelParams::map_rln;
		world_ped.h = ped.ped_pose.y*ModelParams::map_rln;
		//TODO : goal
		p.x=ped.ped_pose.x;
		p.y=ped.ped_pose.y;
		p.z=2.0;
		pc.points.push_back(p);

		//cout<<"ped pose "<<ped.ped_pose.x<<" "<<ped.ped_pose.y<<" "<<world_ped.id<<endl;
			
        /// search for proper pedestrian to update
        //bool foundPed = momdp->updatePedRobPose(lPedLocal.ped_local[ii]);;
		ped_list.push_back(world_ped);
        //if(!foundPed)
        //{
            ///if ped_id does not match the old one create a new pomdp problem.
            //ROS_INFO(" Creating  a new pedestrian problem #%d", lPedLocal.ped_local[ii].ped_id);
            //momdp->addNewPed(lPedLocal.ped_local[ii]);
			//ROS_INFO("Create a new pedestrian!", lPedLocal.ped_local[ii].ped_id);
			//world.addNewPed(lPedLocal.ped_local[ii]);                                                  }
    }
	std::sort(ped_list.begin(),ped_list.end(),sortFn);
	for(int i=0;i<ped_list.size();i++)
	{
		RealWorld.UpdatePedPoseReal(ped_list[i]);
		//cout<<ped_list[i].id<<" ";
	}
	//cout<<endl;
	pc_pub.publish(pc);

}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"data_recorder");	
	ros::NodeHandle nh;
	nh.subscribe("ped_local_frame_vector",1,&pedestrianCallBack);
	ros::spin();
	return 0;
}
