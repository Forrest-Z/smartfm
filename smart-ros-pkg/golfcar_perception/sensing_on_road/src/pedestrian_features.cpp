#include "pedestrian_features.h"

#define ASSO_TIME_COEF 0.1
#define NEAREST_THRESH 1.0
#define MERGE_DIST 0.30
#define MERGE_TIME 0.20
#define MERGE_CENTR_DIS 1.0
#define SPLIT_DIST 0.30
#define SPLIT_TIME 0.2
#define OCCLU_DIST 0.8
#define OCCLU_TIME 3.0
#define SPEED_DURATION 1.0
#define HISTORY_SIZE 100
#define STALE_THRESH 3.0

#define MOVING_BELIEF 0.4
#define SIZE_BELIEF 0.2

#define PERSON_SENSE_PERSON         0.9
#define NOT_PERSON_SENSE_PERSON     0.4
#define PROBABILITY_THRESHOLD       0.9
#define ONCE_FOR_ALL_THRESHOLD      0.95
#define ONCE_FOR_ALL_RESTTIME       1.0

namespace sensing_on_road
{

pedestrian_features::pedestrian_features()
{
    ros::NodeHandle nh;
    segments_batch_sub_    = nh.subscribe("segment_processed", 1, &pedestrian_features::pedestrian_filtering, this);
    veri_vision_sub_       = nh.subscribe("veri_pd_vision", 1, &pedestrian_features::Bayesfilter, this);
    laser_pedestrians_pub_ = nh.advertise<sensing_on_road::pedestrian_laser_batch>("ped_laser_batch", 2);
    pedestrians_pcl_pub_   = nh.advertise<sensor_msgs::PointCloud>("ped_laser_pcl", 2);
    veri_show_batch_pub_   = nh.advertise<sensing_on_road::pedestrian_vision_batch>("veri_show_batch", 2);
    veri_show_pcl_pub_     = nh.advertise<sensor_msgs::PointCloud>("veri_show_pcl", 2);
    object_total_number_   = 0;
}

pedestrian_features::~pedestrian_features()
{

}

//Function "pedestrian_filtering" open to change;
//to filter out irralavant segments according to some criteria;
//input  variable  - batch_para;
//output variable  - filtered_segment_batch_;
void pedestrian_features::pedestrian_filtering(const sensing_on_road::segments_one_batch& batch_para)
{
    filtered_segment_batch_ = batch_para;

    /*
    for(unsigned is =0; is <filtered_segment_batch_.segments.size(); is++)
    {
        if(filtered_segment_batch_.segments[is].max_dimension < 0.30)
        filtered_segment_batch_.segments.erase(filtered_segment_batch_.segments.begin()+is);
    }
    */
    ROS_DEBUG("Before data_association");
    data_association();
    ROS_DEBUG("After data_association");
    history_processing();
    ROS_DEBUG("After history_processing");
    pedestrian_extraction();
    ROS_DEBUG("After pedestrian_extraction");
}


//Fundamental Function "data_association";
//to associate newly comming segments with history;
//input  variable  - filtered_segment_batch_;
//output variable  - history_pool_;
void pedestrian_features::data_association()
{
    //the same batch of segments has the same time stamp;
    ros::Time segment_time = filtered_segment_batch_.header.stamp;

    //history_pool initialization;
    if(history_pool_.size()==0)
    {
        for(unsigned is=0; is!=filtered_segment_batch_.segments.size(); is++)
        {
            object_total_number_++; //from "1" to label objects;
            pedestrian_laser_history history_temp;
            history_temp.object_label = object_total_number_;
            history_temp.merged_labels.push_back(object_total_number_);
            history_temp.latest_update_time = segment_time;
            history_temp.single_segment_history.push_back(filtered_segment_batch_.segments[is]);
            history_pool_.push_back(history_temp);
        }
        ROS_INFO("History Initialized: %d", (int)history_pool_.size());
    }
    else
    {
        //--------------------------------------------------
        // association 1st step: two way matching;
        // here only use "distance" to calculate "nearest";
        //--------------------------------------------------

        //1. "history" and "new_segment" find their nearest partner respectively;
        //1.a "history" try to find nearest "new segment";
        for(unsigned ih=0; ih!=history_pool_.size(); ih++)
        {
            history_pool_[ih].nearest_segment_serial = -1;
            history_pool_[ih].history_status = 0;
            float last_history_x = (float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.x;
            float last_history_y = (float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.y;
            ros::Time history_time = history_pool_[ih].latest_update_time;
            float dis2d = 10000.0;
            float disSpace = 10000.0;
            float time_distance = (float)(segment_time.toSec()-history_time.toSec());

            for(unsigned is=0; is!=filtered_segment_batch_.segments.size(); is++)
            {
                float dis_x, dis_y, dis2d_tmp;
                dis_x = (float)filtered_segment_batch_.segments[is].segment_centroid_odom.point.x-last_history_x;
                dis_y = (float)filtered_segment_batch_.segments[is].segment_centroid_odom.point.y-last_history_y;


                dis2d_tmp = sqrtf(dis_x*dis_x+dis_y*dis_y)+fabsf(time_distance)*ASSO_TIME_COEF;

                if(dis2d_tmp<dis2d){dis2d = dis2d_tmp; disSpace = sqrtf(dis_x*dis_x+dis_y*dis_y); history_pool_[ih].nearest_segment_serial = is;}
            }
            //refer to "if(history_pool_[ih].nearest_distance >= NEAREST_THRESH)"
            //when the history is occluded for too long time, to compensate this effect;
            float compensate_dis = 0.0;
            if(history_pool_[ih].fastest_velocity>0.3) compensate_dis = time_distance*1.0;
            history_pool_[ih].nearest_distance = disSpace-compensate_dis;

            /*
            ROS_INFO("1--history obj_label %d, serial %d, %3f, %3f, merged size %d; segment %d, %3f, %3f, shortest dis2d_tmp, distance: %3f, %3f",
                    history_pool_[ih].object_label, ih, last_history_x, last_history_y, (int)history_pool_[ih].merged_labels.size(),
                    history_pool_[ih].nearest_segment_serial,
                    (float)filtered_segment_batch_.segments[history_pool_[ih].nearest_segment_serial].segment_centroid_odom.point.x,
                    (float)filtered_segment_batch_.segments[history_pool_[ih].nearest_segment_serial].segment_centroid_odom.point.y,
                    dis2d, disSpace
                    );
                    */ 

        }

        //1.b "new segment" try to find nearest "history";
        for(unsigned is=0; is!=filtered_segment_batch_.segments.size(); is++)
        {
            filtered_segment_batch_.segments[is].nearest_history_serial =-1;
            filtered_segment_batch_.segments[is].segment_status = 0;
            float segment_x = (float)filtered_segment_batch_.segments[is].segment_centroid_odom.point.x;
            float segment_y = (float)filtered_segment_batch_.segments[is].segment_centroid_odom.point.y;
            float dis2d = 10000.0;
            float disSpace = 10000.0;
            for(unsigned ih=0; ih!=history_pool_.size(); ih++)
            {
                float dis_x, dis_y, dis2d_tmp;
                dis_x = segment_x-(float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.x;
                dis_y = segment_y-(float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.y;
                ros::Time history_time = history_pool_[ih].latest_update_time;
                float time_distance = (float)(segment_time.toSec()-history_time.toSec());
                dis2d_tmp = sqrtf(dis_x*dis_x+dis_y*dis_y)+fabsf(time_distance)*ASSO_TIME_COEF;
                if(dis2d_tmp<dis2d)
                {
                    dis2d = dis2d_tmp;
                    filtered_segment_batch_.segments[is].nearest_history_serial =(int)ih;
                    disSpace=sqrtf(dis_x*dis_x+dis_y*dis_y);
                }
            }
            
            /*
            ROS_INFO("2--segment %d, %3f, %3f; history %d, %3f, %3f, shortest dis2d_tmp, distance: %3f, %3f",
                    is,
                    segment_x, segment_y,
                    filtered_segment_batch_.segments[is].nearest_history_serial,
                    (float)history_pool_[filtered_segment_batch_.segments[is].nearest_history_serial].single_segment_history.back().segment_centroid_odom.point.x,
                    (float)history_pool_[filtered_segment_batch_.segments[is].nearest_history_serial].single_segment_history.back().segment_centroid_odom.point.y,
                    dis2d, disSpace
                    );
                    */ 
        }

        //2 mutual matching is given highest priority;
        //2.a check from view of "history_pool";
        for(unsigned ih=0; ih!=history_pool_.size(); ih++)
        {
            int segment_serial = history_pool_[ih].nearest_segment_serial;
            if( segment_serial < 0 || segment_serial >= (int)(filtered_segment_batch_.segments.size()))
            {ROS_ERROR("Unexpected Overflow");return;}
            //check if mutual nearest;
            if(filtered_segment_batch_.segments[segment_serial].nearest_history_serial == (int)ih)
            {
                if(history_pool_[ih].nearest_distance >= NEAREST_THRESH)
                {
                    //"mutual matching" is given tightest binging;
                    history_pool_[ih].history_status = 10;
                    filtered_segment_batch_.segments[segment_serial].segment_status = 10;
                }
                else
                {
                    history_pool_[ih].history_status = 1;
                    filtered_segment_batch_.segments[segment_serial].segment_status = 1;
                    //ROS_INFO("Real Mutual Match ---- history %d, segment %d",ih,segment_serial);
                }
            }
        }

        for(unsigned ih=0; ih!=history_pool_.size(); ih++)
        {
            int segment_serial = history_pool_[ih].nearest_segment_serial;
            if(history_pool_[ih].history_status == 0)
            {
                //try to find "merge" chance from its nearest segment;
                //1.distance criterion;
                geometry_msgs::Point32 his_pt1  = history_pool_[ih].single_segment_history.back().rightPoint;
                geometry_msgs::Point32 his_pt2  = history_pool_[ih].single_segment_history.back().leftPoint;
                geometry_msgs::Point32 seg_pt1  = filtered_segment_batch_.segments[segment_serial].rightPoint;
                geometry_msgs::Point32 seg_pt2  = filtered_segment_batch_.segments[segment_serial].leftPoint;
                float nearest_dis = his_seg_dis(his_pt1, his_pt2, seg_pt1, seg_pt2);

                bool dis_flag = (nearest_dis < MERGE_DIST && history_pool_[ih].nearest_distance < MERGE_CENTR_DIS);
                //2. time criterion
                ros::Time history_time = history_pool_[ih].latest_update_time;
                float time_distance = (float)(segment_time.toSec()-history_time.toSec());
                bool  time_flag = (fabsf(time_distance)< MERGE_TIME);

                //2 for "merge"; 4 for "rest"; 3 is left for "split", to keep according with segment_status;
                if(dis_flag && time_flag)
                {   //"2" means merge, need special operation;
                    //it may be classified into "mutual matching" in Part 2.b;
                    //if(filtered_segment_batch_.segments[segment_serial].segment_status==1 || filtered_segment_batch_.segments[segment_serial].segment_status==10)
                    if(filtered_segment_batch_.segments[segment_serial].segment_status==1)
                    {
                        history_pool_[ih].history_status = 2;
                    }
                    else
                    {
                        history_pool_[ih].history_status = 1;
                        filtered_segment_batch_.segments[segment_serial].segment_status=1;
                        //ROS_INFO("Add Mutual Match ---- history: %d, segment %d, dis %3f", ih, segment_serial, nearest_dis);
                    }
                }
                else {history_pool_[ih].history_status = 4;}
            }
        }

        //2.b check from view of "filtered_segment_batch_";
        for(unsigned is=0; is!=filtered_segment_batch_.segments.size(); is++)
        {
            if(filtered_segment_batch_.segments[is].segment_status ==0 || filtered_segment_batch_.segments[is].segment_status ==10)
            {
                int history_serial = filtered_segment_batch_.segments[is].nearest_history_serial;
                bool history_moving = (history_pool_[history_serial].fastest_velocity>0.3);

                //try to find "split" chance from its nearest history;
                //1.distance criterion;
                geometry_msgs::Point32 his_pt1  = history_pool_[history_serial].single_segment_history.back().rightPoint;
                geometry_msgs::Point32 his_pt2  = history_pool_[history_serial].single_segment_history.back().leftPoint;
                geometry_msgs::Point32 seg_pt1  = filtered_segment_batch_.segments[is].rightPoint;
                geometry_msgs::Point32 seg_pt2  = filtered_segment_batch_.segments[is].leftPoint;
                float nearest_dis = his_seg_dis(his_pt1, his_pt2, seg_pt1, seg_pt2);
                bool dis_flag = (nearest_dis < SPLIT_DIST);

                //2. time criterion
                ros::Time history_time = history_pool_[history_serial].latest_update_time;
                float time_distance = (float)(segment_time.toSec()-history_time.toSec());
                bool  time_flag = (fabsf(time_distance)< SPLIT_TIME);

                float compensate_dis = 0.0;
                compensate_dis = time_distance*1.0;
                float nearest_dis2 = his_seg_dis(his_pt1, his_pt2, seg_pt1, seg_pt2)-compensate_dis;
                bool dis_flag2 = (nearest_dis2 < OCCLU_DIST);
                bool  time_flag2 = (fabsf(time_distance)< OCCLU_TIME);

                if(dis_flag && time_flag && history_moving)
                {
                    if(history_pool_[history_serial].history_status == 1)
                    {
                        //"3" means split, need special operation;
                        filtered_segment_batch_.segments[is].segment_status=3;
                        //ROS_INFO("Split ---- history: %d, segment %d", history_serial, is);
                    }
                    else if(history_pool_[history_serial].history_status == 2)
                    {
                        //ROS_INFO("Add2 Mutual Match ---- history: %d, segment %d", history_serial, is);
                        history_pool_[history_serial].history_status = 1;
                        filtered_segment_batch_.segments[is].segment_status=1;
                    }
                    else if(history_pool_[history_serial].history_status == 4 )
                    {
                        //ROS_INFO("Add3 Mutual Match ---- history: %d, segment %d", history_serial, is);
                        history_pool_[history_serial].history_status = 1;
                        filtered_segment_batch_.segments[is].segment_status=1;
                    }
                    else if( history_pool_[history_serial].history_status == 10)
                    {
                        if(filtered_segment_batch_.segments[is].segment_status ==10)
                        {
                            //ROS_INFO("Add3 Mutual Match ---- history: %d, segment %d", history_serial, is);
                            history_pool_[history_serial].history_status = 1;
                            filtered_segment_batch_.segments[is].segment_status=1;
                        }
                        else
                        {
                            filtered_segment_batch_.segments[is].segment_status=3;
                            //ROS_INFO("Split ---- history: %d, segment %d", history_serial, is);
                        }
                    }
                }
                else
                {
                    //take partial occlusion into consideration, in a more relaxed way;
                    bool partial_occlusion = false;
                    if(history_moving)
                    {
                        if(filtered_segment_batch_.segments[is].segment_status ==10 && history_pool_[history_serial].history_status == 10)
                        {
                            if(dis_flag2&&time_flag2) partial_occlusion = true;
                        }
                    }

                    if(partial_occlusion)
                    {
                        //ROS_INFO("Add OCCLU Mutual Match ---- history: %d, segment %d", history_serial, is);
                        history_pool_[history_serial].history_status = 1;
                        filtered_segment_batch_.segments[is].segment_status=1;
                    }
                    else
                    {
                        //ROS_INFO("Segment Generate new history; segment: %d", is);
                        //"4" will initiate new history.
                        filtered_segment_batch_.segments[is].segment_status=4;
                    }
                }
            }
        }

        //3. final check and update;

        //3.a check history;
        //define to erase merged history later, from end to beginning;
        std::vector<unsigned> merged_history_label;
        //ROS_INFO("3.a check history");
        for(unsigned ih=0; ih!=history_pool_.size(); ih++)
        {
            int segment_serial = history_pool_[ih].nearest_segment_serial;
            if(history_pool_[ih].history_status==1)
            {
                history_pool_[ih].latest_update_time = filtered_segment_batch_.header.stamp;
                history_pool_[ih].single_segment_history.push_back(filtered_segment_batch_.segments[segment_serial]);
            }
            else if(history_pool_[ih].history_status==2)
            {
                //ROS_INFO("Merge ---- history: %d, segment %d", ih, segment_serial);

                int history_serial = filtered_segment_batch_.segments[segment_serial].nearest_history_serial;

                if(history_pool_[ih].fastest_velocity > history_pool_[history_serial].fastest_velocity)
                {history_pool_[history_serial].fastest_velocity = history_pool_[ih].fastest_velocity;}

                if(history_pool_[ih].pedestrian_belief > history_pool_[history_serial].pedestrian_belief)
                {history_pool_[history_serial].pedestrian_belief = history_pool_[ih].pedestrian_belief;}

                //prepare to detele merged history;
                merged_history_label.push_back(history_pool_[ih].object_label);

                //-----------------------------------------------------------
                //--------deal with labeling problem when merging-----------;
                //-----------------------------------------------------------
                //1. recording all possible object_label;
                //ROS_INFO("Record all possible label");
                for(unsigned io=0; io < history_pool_[ih].merged_labels.size(); io++)
                {
                    int object_label = history_pool_[ih].merged_labels[io];
                    int label_serial = -1;
                    if( history_pool_[history_serial].merged_labels.size()>5) break;
                    for(unsigned ioo=0; ioo < history_pool_[history_serial].merged_labels.size(); ioo++)
                    {
                        if(object_label==history_pool_[history_serial].merged_labels[ioo])
                        {label_serial= ioo; break;}
                    }
                    if(label_serial==-1){history_pool_[history_serial].merged_labels.push_back(object_label);}
                }

                //2. bubble-sorting for "object_label", from small to large;
                //ROS_INFO("Bubble sorting %d", history_pool_[history_serial].merged_labels.size());
                for(unsigned y = 0; y < history_pool_[history_serial].merged_labels.size(); y++)
                {
                    for(unsigned k = 0; k < history_pool_[history_serial].merged_labels.size()-1-y; k++)
                    {
                        if(history_pool_[history_serial].merged_labels[k] > history_pool_[history_serial].merged_labels[k+1])
                        {
                            int object_label = history_pool_[history_serial].merged_labels[k+1];
                            history_pool_[history_serial].merged_labels[k+1]  = history_pool_[history_serial].merged_labels[k];
                            history_pool_[history_serial].merged_labels[k]  = object_label;
                        }
                    }
                }


            }
            else if(history_pool_[ih].history_status==4)
            {
                //not updated;
            }
            else if(history_pool_[ih].history_status==10)
            {
                //not updated;
            }
        }

        //3.b check segments
        //ROS_INFO("3.b check segments");
        for(unsigned is=0; is!=filtered_segment_batch_.segments.size(); is++)
        {
            int history_serial = filtered_segment_batch_.segments[is].nearest_history_serial;
            if(filtered_segment_batch_.segments[is].segment_status==3)
            {
                pedestrian_laser_history history_tmp = history_pool_[history_serial];
                history_tmp.once_for_all = false;
                history_tmp.single_segment_history.pop_back();
                history_tmp.single_segment_history.push_back(filtered_segment_batch_.segments[is]);
                unsigned label_position_tmp = 0;
                for(unsigned il=0; il<history_tmp.merged_labels.size(); il++)
                {
                    if(history_tmp.object_label==history_tmp.merged_labels[il])
                    {label_position_tmp = il; break;}
                }
                //if existing name is not enough, meaning new person splitted, give it a new label;
                if(label_position_tmp==history_tmp.merged_labels.size()-1)
                {
                    object_total_number_++;
                    history_tmp.object_label=object_total_number_;
                    history_tmp.merged_labels.push_back(object_total_number_);
                }
                else{history_tmp.object_label= history_tmp.merged_labels[label_position_tmp+1];}
                history_pool_.push_back(history_tmp);
            }
            else if(filtered_segment_batch_.segments[is].segment_status==4 ||filtered_segment_batch_.segments[is].segment_status==10)
            {
                object_total_number_++;
                pedestrian_laser_history history_tmp;
                history_tmp.object_label = object_total_number_;
                history_tmp.merged_labels.push_back(object_total_number_);
                history_tmp.latest_update_time = segment_time;
                history_tmp.single_segment_history.push_back(filtered_segment_batch_.segments[is]);
                history_pool_.push_back(history_tmp);
            }
        }

        //3.c delete merged history, pay attention the sequence: use "object_label" rather than "vector serial" to delete here;
        //ROS_INFO("3.c delete merged history");
        for(unsigned im=0; im!=merged_history_label.size(); im++)
        {
            for(unsigned ih=0; ih!=history_pool_.size(); ih++)
            {
                if(history_pool_[ih].object_label ==(int) merged_history_label[im])
                {history_pool_.erase(history_pool_.begin()+ih);break;}
            }
        }
    }
    ROS_DEBUG("data association finished");
}

float pedestrian_features::his_seg_dis(geometry_msgs::Point32 &pt11, geometry_msgs::Point32 &pt12, geometry_msgs::Point32 &pt21, geometry_msgs::Point32 &pt22)
{
    float dis_11_21 = sqrtf((pt11.x-pt21.x)*(pt11.x-pt21.x)+(pt11.y-pt21.y)*(pt11.y-pt21.y));
    float dis_11_22 = sqrtf((pt11.x-pt22.x)*(pt11.x-pt22.x)+(pt11.y-pt22.y)*(pt11.y-pt22.y));
    float dis_12_21 = sqrtf((pt12.x-pt21.x)*(pt12.x-pt21.x)+(pt12.y-pt21.y)*(pt12.y-pt21.y));
    float dis_12_22 = sqrtf((pt12.x-pt22.x)*(pt12.x-pt22.x)+(pt12.y-pt22.y)*(pt12.y-pt22.y));

    float smallest_dis = dis_11_21;
    if(dis_11_22<smallest_dis) smallest_dis=dis_11_22;
    if(dis_12_21<smallest_dis) smallest_dis=dis_12_21;
    if(dis_12_22<smallest_dis) smallest_dis=dis_12_22;

    return smallest_dis;
}

void pedestrian_features::history_processing()
{
    ros::Time segment_time = filtered_segment_batch_.header.stamp;
    std::vector<unsigned> stale_history;
    for(unsigned ih=0; ih!=history_pool_.size(); ih++)
    {
        //1st: maintain size;
        if(history_pool_[ih].single_segment_history.size()>=HISTORY_SIZE)
        {history_pool_[ih].single_segment_history.erase(history_pool_[ih].single_segment_history.begin());}

        //2nd: delete stale history_branch
        ros::Time history_time = history_pool_[ih].latest_update_time;
        if(segment_time.toSec()-history_time.toSec()> STALE_THRESH)
        {stale_history.push_back(ih);ROS_DEBUG("Delete History %d", ih);continue;}

        //3rd: check move or not;
        if(history_pool_[ih].history_status==4){continue;}
        float track_duration = (float)(history_pool_[ih].latest_update_time.toSec() - history_pool_[ih].single_segment_history.front().segment_centroid_laser.header.stamp.toSec());
        if(track_duration> SPEED_DURATION)
        {
            float front_x = (float)history_pool_[ih].single_segment_history.front().segment_centroid_odom.point.x;
            float front_y = (float)history_pool_[ih].single_segment_history.front().segment_centroid_odom.point.y;
            float back_x  = (float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.x;
            float back_y  = (float)history_pool_[ih].single_segment_history.back().segment_centroid_odom.point.y;
            float dis_front_back = sqrtf((front_x-back_x)*(front_x-back_x)+(front_y-back_y)*(front_y-back_y));
            history_pool_[ih].velocity = dis_front_back/track_duration;
            if(history_pool_[ih].velocity >history_pool_[ih].fastest_velocity)
            {history_pool_[ih].fastest_velocity = history_pool_[ih].velocity;}
        }
    }

    if(stale_history.size()>0)
    {
        for(unsigned is = stale_history.size(); is!=0; is--)
        {
            unsigned delete_history_serial = stale_history[is-1];
            history_pool_.erase(history_pool_.begin()+delete_history_serial);

            //tricky bug @_@ takes much time;
            //history_pool_.erase(history_pool_.begin()+stale_history[is-1]);
        }
    }
}

void pedestrian_features::pedestrian_extraction()
{
    new_laser_pedestrians_.header = filtered_segment_batch_.header;
    new_laser_pedestrians_.pedestrian_laser_features.clear();

    pedestrians_cloud_.header = filtered_segment_batch_.header;
    pedestrians_cloud_.points.clear();

    for(unsigned ih=0; ih!=history_pool_.size(); ih++)
    {
        if(history_pool_[ih].history_status==4){continue;}
        else
        {
            sensing_on_road::pedestrian_laser ped_laser;
            //size criteria && speed criteria
            if(history_pool_[ih].fastest_velocity<3.0 && history_pool_[ih].single_segment_history.back().max_dimension<3.0 )
            {
                ped_laser.object_label       =  history_pool_[ih].object_label;
                ped_laser.size               =  history_pool_[ih].single_segment_history.back().max_dimension;
                ped_laser.pedestrian_laser   =   history_pool_[ih].single_segment_history.back().segment_centroid_laser;

                bool ped_pub = false;
                if(history_pool_[ih].fastest_velocity>0.5)
                {
                    ped_pub = true;
                    if(history_pool_[ih].pedestrian_belief < MOVING_BELIEF) history_pool_[ih].pedestrian_belief = MOVING_BELIEF;
                }
                else if(history_pool_[ih].single_segment_history.back().max_dimension <1.0 &&history_pool_[ih].single_segment_history.back().max_dimension >0.3)
                {
                    //optional choice...
                    //ped_pub = true;
                    if(history_pool_[ih].pedestrian_belief < SIZE_BELIEF) history_pool_[ih].pedestrian_belief = SIZE_BELIEF;
                }

                if(ped_pub)
                {
                    geometry_msgs::Point32 ped_point_temp;
                    ped_point_temp.x = (float)ped_laser.pedestrian_laser.point.x;
                    ped_point_temp.y = (float)ped_laser.pedestrian_laser.point.y;
                    ped_point_temp.z = (float)ped_laser.pedestrian_laser.point.z;
                    pedestrians_cloud_.points.push_back(ped_point_temp);
                    new_laser_pedestrians_.pedestrian_laser_features.push_back(ped_laser);
                    ROS_DEBUG("-------------pedestrian history object label %d, serial %d--------------", history_pool_[ih].object_label, ih);
                }
            }
            else
            {
                //belong to other features;
            }
        }
    }

    laser_pedestrians_pub_.publish(new_laser_pedestrians_);
    pedestrians_pcl_pub_.publish(pedestrians_cloud_);
}

void pedestrian_features::Bayesfilter(const sensing_on_road::pedestrian_vision_batch& veri_pd_para)
{
    veri_batch_ = veri_pd_para;
    for(unsigned ih=0; ih!=history_pool_.size(); ih++)
    {
        for(unsigned i=0; i<veri_pd_para.pd_vector.size(); i++)
        {
            if(history_pool_[ih].object_label==veri_pd_para.pd_vector[i].object_label)
            {
                if(veri_pd_para.pd_vector[i].complete_flag==true)
                {
                    //2011-12-31: need Demian to check the verification result, which all shows "false"...
                    if(veri_pd_para.pd_vector[i].decision_flag==true)
                    {
                        float sense_person;
                        sense_person=(history_pool_[ih].pedestrian_belief)*PERSON_SENSE_PERSON+(1-history_pool_[ih].pedestrian_belief)*NOT_PERSON_SENSE_PERSON;
                        history_pool_[ih].pedestrian_belief=((history_pool_[ih].pedestrian_belief)*PERSON_SENSE_PERSON)/sense_person;
                    }
                    else
                    {
                        float sense_not_person;
                        sense_not_person=(history_pool_[ih].pedestrian_belief)*(1-PERSON_SENSE_PERSON)+(1-history_pool_[ih].pedestrian_belief)*(1-NOT_PERSON_SENSE_PERSON);
                        history_pool_[ih].pedestrian_belief=(history_pool_[ih].pedestrian_belief)*(1-PERSON_SENSE_PERSON)/sense_not_person;
                    }
                }
            }
        }
    }
    
    ProbabilityCheck();
}

void pedestrian_features::ProbabilityCheck()
{
    veri_show_pcl_.header   = veri_batch_.header;
    veri_show_pcl_.header.frame_id   = filtered_segment_batch_.header.frame_id;
    veri_show_pcl_.points.clear();
    veri_show_batch_.header = veri_batch_.header;
    veri_show_batch_.pd_vector.clear();
    ros::Time segment_time = filtered_segment_batch_.header.stamp;

    for(unsigned ih=0; ih!=history_pool_.size(); ih++)
    {
        //add highest probility...like fastest_velocity...to make verification more stable...if necessary...
        if(history_pool_[ih].pedestrian_belief > PROBABILITY_THRESHOLD || history_pool_[ih].once_for_all)
        {
            if(history_pool_[ih].pedestrian_belief > ONCE_FOR_ALL_THRESHOLD){history_pool_[ih].once_for_all=true;}


            ros::Time history_time = history_pool_[ih].latest_update_time;
            if(segment_time.toSec()-history_time.toSec()> ONCE_FOR_ALL_RESTTIME) {history_pool_[ih].once_for_all=false; continue;}


            geometry_msgs::Point32 veri_point;
            veri_point.x = (float)history_pool_[ih].single_segment_history.back().segment_centroid_laser.point.x;
            veri_point.y = (float)history_pool_[ih].single_segment_history.back().segment_centroid_laser.point.y;
            veri_point.z = (float)history_pool_[ih].single_segment_history.back().segment_centroid_laser.point.z;
            veri_show_pcl_.points.push_back(veri_point);

            for(unsigned ib=0; ib<veri_batch_.pd_vector.size(); ib++)
            {
                if(history_pool_[ih].object_label==veri_batch_.pd_vector[ib].object_label)
                {
                    veri_show_batch_.pd_vector.push_back(veri_batch_.pd_vector[ib]);
                    break;
                }
            }
        }
    }

    veri_show_batch_pub_.publish(veri_show_batch_);
    veri_show_pcl_pub_.publish(veri_show_pcl_);
}

} //namespace sensing_on_road



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pedestrian_features");
    sensing_on_road::pedestrian_features pedestrian;
    ros::spin();
    return 0;
}
