#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <ped_momdp_sarsop/ped_local_frame_vector.h>
#include <sensing_on_road/pedestrian_vision_batch.h>

using namespace std;
struct ped_transforms
{
    tf::Transform transform;
    int label;
    ros::Time last_update;
};

class local_frame
{
public:
    local_frame();
    ~local_frame(){};

private:
    void publishTransform(const ros::TimerEvent& event);
    void pedCallback(sensing_on_road::pedestrian_vision_batchConstPtr ped_batch);
    bool getObjectPose(string& target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const;
	void PublishLocalFrames();
    vector<ped_transforms> ped_transforms_;
    tf::TransformBroadcaster br_;
    tf::TransformListener tf_;
    ros::Timer timer_;
    ros::Subscriber ped_sub_;
    ros::Publisher local_pub_;
    string global_frame_;
    double threshold_, offsetx_, offsety_;
};

local_frame::local_frame()
{
    ros::NodeHandle nh;
    ped_sub_ = nh.subscribe("ped_data_assoc", 1, &local_frame::pedCallback, this);
    local_pub_ = nh.advertise<ped_momdp_sarsop::ped_local_frame_vector>("ped_local_frame_vector", 1);

    ros::NodeHandle n("~");
    //n.param("global_frame", global_frame_, string("/odom"));
	n.param("global_frame", global_frame_, string("/golfcart/map"));
    n.param("threshold", threshold_, 3.0);
    n.param("offsetx", offsetx_, 0.0);
    n.param("offsety", offsety_, 3.0);
    timer_ = nh.createTimer(ros::Duration(0.01), &local_frame::publishTransform, this);
    ros::spin();
}

void local_frame::pedCallback(sensing_on_road::pedestrian_vision_batchConstPtr ped_batch)
{
  //cout<<"Pedcallback size of ped_batch="<<ped_batch->pd_vector.size()<<endl;
    ped_momdp_sarsop::ped_local_frame_vector plf_vector;
    for(size_t i=0;i<ped_batch->pd_vector.size();i++)
    {
        //find if the transform is already available
		/*
        size_t matched_ped;
        bool matched=false;
        for(size_t j=0; j<ped_transforms_.size(); j++)
        {
            if(ped_batch->pd_vector[i].object_label == ped_transforms_[j].label)
            {
                matched = true;
                matched_ped = j;
                continue;
            }

        }*/
        //if(matched)
		if(true)
        {
            //transform found, just update the position
            if(ped_batch->pd_vector[i].confidence > 0.1)
            {
                ped_momdp_sarsop::ped_local_frame plf;
                plf.header.stamp = ped_batch->header.stamp;
				/*
                stringstream frame_id;
                frame_id<<"ped_"<<ped_transforms_[matched_ped].label;
                plf.header.frame_id = frame_id.str();
				*/
				plf.header.frame_id="/golfcart/map";

                tf::Stamped<tf::Pose> in_pose, out_pose;

                //start with pedestrian. no interest on the orientation of ped for now
                in_pose.setIdentity();
                in_pose.frame_id_ = ped_batch->header.frame_id;
                geometry_msgs::Point32 ped_point = ped_batch->pd_vector[i].cluster.centroid;
                in_pose.setOrigin(tf::Vector3(ped_point.x, ped_point.y, ped_point.z));
                if(!getObjectPose(plf.header.frame_id, in_pose, out_pose)) continue;
                plf.ped_id = ped_transforms_[matched_ped].label;
                plf.ped_pose.x = out_pose.getOrigin().getX();
                plf.ped_pose.y = out_pose.getOrigin().getY();
                plf.ped_pose.z = out_pose.getOrigin().getZ();

                //then update the robot pose with the same frame
                in_pose.setIdentity();
                in_pose.frame_id_ = "base_link";
                if(!getObjectPose(plf.header.frame_id, in_pose, out_pose)) continue;

                plf.rob_pose.x = out_pose.getOrigin().getX();
                plf.rob_pose.y = out_pose.getOrigin().getY();
                plf.rob_pose.z = out_pose.getOrigin().getZ();
                plf_vector.ped_local.push_back(plf);
                ped_transforms_[matched_ped].last_update = ped_batch->header.stamp;
            }
        }
        else
        {
	    //if(ped_batch->pd_vector[i].confidence > threshold_/100)
            {
				ROS_INFO("New pedestrian received, creating new transform");
				ROS_INFO("%d transformations", ped_transforms_.size());
				//transform not found, add a new transform
				tf::Transform transform;
				tf::Stamped<tf::Pose> in_pose, out_pose;
				//use the current base_link pose as the new frame
				in_pose.frame_id_ = "base_link";
				in_pose.setIdentity();
				in_pose.setOrigin(btVector3(offsetx_,offsety_,0.0));
				if(!getObjectPose(global_frame_, in_pose, out_pose)) continue;
				transform.setOrigin( out_pose.getOrigin() );
				transform.setRotation( out_pose.getRotation() );
				ped_transforms ped_tr;
				ped_tr.label = ped_batch->pd_vector[i].object_label;
				ped_tr.last_update = ped_batch->header.stamp;
				ped_tr.transform = transform;
				ped_transforms_.push_back(ped_tr);
			}
        }
    }
    //finally publish all the transformed points
    local_pub_.publish(plf_vector);
}
bool local_frame::getObjectPose(string& target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const
{
    out_pose.setIdentity();

    try {
        tf_.transformPose(target_frame, in_pose, out_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
    }
    return true;
}

void local_frame::publishTransform(const ros::TimerEvent& event)
{
    for(size_t i=0; i<ped_transforms_.size(); i++)
    {
        stringstream frame_id;
        frame_id<<"ped_"<<ped_transforms_[i].label;
        br_.sendTransform(tf::StampedTransform(ped_transforms_[i].transform, ros::Time::now(), global_frame_, frame_id.str()));
    }
}

/*
void local_frame::PublishLocalFrames()
{
	ped_momdp_sarsop::ped_local_frame_vector plf_vector;
	Car car=world.GetCarPose();
	for(int i=0;i<world.NumPedInView();i++)
	{
		Pedestrian ped=world.GetPedPose(i);	
		ped_momdp_sarsop::ped_local_frame plf;
		plf.ped_id=ped.id;
		plf.ped_pose.x=ped.w;
		plf.ped_pose.y=ped.h;
		plf.rob_pose.x=car.w;
		plf.rob_pose.y=car.h;
		plf_vector.push_back(plf);	
	}
	local_pub_.publish(plf_vector);
	world.OneStep();
	
}*/
int main(int argc, char** argv){
    ros::init(argc, argv, "local_frame");
    local_frame *lf = new local_frame();
	//PublishLocalFrames();   //this is for simulation
    //

    return 0;
};
