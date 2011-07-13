#include <local_map.h>

#define MAP_RES     (0.05)
#define MAP_WIDTH   (5.0)
#define MAP_HEIGHT  (5.0)


// vehicle location is always (0,0)
void local_map::map_init()
{
    map = (unsigned char *) calloc(sizeof(unsigned char), (int)((MAP_WIDTH*MAP_HEIGHT)/MAP_RES/MAP_RES));;

}
void local_map::map_free()
{
    free map;
}

void local_map::on_sick(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    cout<<"got message with: " << msg.header.frame_id << endl;

    float *angles;
    float *ranges;
    int nranges = msg->ranges.size();
    angles = new float[nranges];
    ranges = new float[nranges];

    float curr_angle = 0;
    for(int i=0; i< nranges; i++)
    {
        ranges[i] = msg->ranges[i];
        angles[i] = curr_angle;
        curr_angle += msg->angle_increment;
    }

    // transform to base_link frame here

    // 

    free angles;
    free ranges;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map");

    ros::NodeHandle nh;

    ros::Subscriber sick1;
    sick1 = nh.subscribe(nh, "sick_scan", 2, on_sick);
    //ros::Subscriber sick2 = nh.subscribe(nh, "sick_scan2", 2, on_sick);

    ros::spin();

}

