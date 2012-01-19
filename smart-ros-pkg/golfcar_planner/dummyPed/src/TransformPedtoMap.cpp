
#include <ros/ros.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <tf/transform_listener.h>

using namespace std;

class transform_ped {
public:
    transform_ped();
private:
    void PedPoseCallback(sensing_on_road::pedestrian_laser_batch ped);
    bool transformPoint(geometry_msgs::PointStamped& in, geometry_msgs::PointStamped& out);
    ros::Publisher transformed_ped_pub_;
    ros::Subscriber ped_sub_;
    tf::TransformListener tf_listener_;
    string ped_str_;
    vector<int> ped_id_;
    bool initialize_ped_;
    vector <geometry_msgs::PointStamped> ped_poses_;
};

vector<string> SplitString(const char *data, const char* delimiter)
{
    char *str = strdup(data);
    char *pch = strtok(str, delimiter);
    vector<string> data_s;
    while (pch != NULL)
    {
        data_s.push_back(string(pch));
        pch = strtok(NULL, delimiter);
    }
    free(str);
    return data_s;
}

transform_ped::transform_ped()
{
    ros::NodeHandle nh;
    transformed_ped_pub_ = nh.advertise<sensing_on_road::pedestrian_laser_batch>("ped_map_laser_batch", 2);
    ped_sub_ = nh.subscribe("ped_laser_batch", 1, &transform_ped::PedPoseCallback, this);
    ros::NodeHandle n("~");
    n.param("ped_id", ped_str_, string(""));
    cout<<"Ped id: "<<ped_str_<<endl;
    if(ped_str_.empty())
    {
        ROS_ERROR("Pedestrian ID not set, please set through parameter ped_id");
        return;
    }
    else
    {
        vector<string> data = SplitString(ped_str_.c_str(), ", ");
        stringstream ss;
        ROS_INFO_STREAM("Number of Ped: "<<data.size()<<" with id");
        for(size_t i=0; i<data.size();i++)
        {
            ped_id_.push_back(atoi(data[i].c_str()));
            ROS_INFO_STREAM(ped_id_[i]);

        }
        ped_poses_.resize(data.size());

    }
    initialize_ped_ = false;
    ros::spin();
}



bool transform_ped::transformPoint(geometry_msgs::PointStamped& in, geometry_msgs::PointStamped& out)
{
    try{
        tf_listener_.transformPoint("map", in, out);
        return true;
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"map\": %s",in.header.frame_id.c_str(), ex.what());
        return false;
    }
}

void transform_ped::PedPoseCallback(sensing_on_road::pedestrian_laser_batch ped)
{
    sensing_on_road::pedestrian_laser_batch ped_map_laser_batch;

    //ped_map_laser_batch = ped;
    sensing_on_road::pedestrian_laser ped_laser;
    ped_map_laser_batch.header = ped.header;
    size_t ped_number=0;
    for(size_t j=0; j<ped_id_.size();j++)
    {
        for(size_t i=0; i<ped.pedestrian_laser_features.size();i++)
        {
            if(ped.pedestrian_laser_features[i].object_label == ped_id_[j])
            {
                if(!transformPoint(ped.pedestrian_laser_features[i].pedestrian_laser, ped_poses_[j])) return;

            }

        }
        //increment the number as long as the position is non-zero,
        //which is a good measure to tell that pedestrian_id is locked
        if(ped_poses_[j].point.x>0 && ped_poses_[j].point.y>0) ped_number++;

        ped_laser.object_label = ped_id_[j];
        ped_laser.pedestrian_laser = ped_poses_[j];
        ped_map_laser_batch.pedestrian_laser_features.push_back(ped_laser);
    }
    if(ped_number == ped_id_.size())
    {
        if(!initialize_ped_) ROS_INFO("All pedestrian locked on, start transforming to global frame!");
        initialize_ped_ = true;

    }
    if(initialize_ped_) transformed_ped_pub_.publish(ped_map_laser_batch);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_ped");

    transform_ped transform_ped_node;


}

