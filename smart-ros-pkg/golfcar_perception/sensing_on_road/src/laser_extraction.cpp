#include <vector>

#include <fmutil/fm_math.h>

#include "laser_extraction.h"

using std::vector;

#define DIS_RANGE 15
#define ASSOCIATE_WITHIN_RANGE_THRESH 1.0
#define ASSOCIATE_OVER_RANGE_MULTI 7

/*
#define ASSOCIATE_WITHIN_5_THRESH 0.5
#define ASSOCIATE_OVER_5_MULTI 10
*/

namespace sensing_on_road
{

laser_extraction::laser_extraction()
{
    ros::NodeHandle nh;
    nh.param("laser_frame_id",     ldmrs_single_id_,      std::string("ldmrsAssemb"));
    nh.param("odom_frame_id",      odom_frame_id_,        std::string("odom"));

    laser_sub_.subscribe(nh, "sickldmrs/assembled", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, ldmrs_single_id_, 10);
    tf_filter_->registerCallback(boost::bind(&laser_extraction::scan_callback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));
    
    laser_pub_ = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
    segment_border_pub_ = nh.advertise<sensor_msgs::PointCloud>("segment_border", 2);
    segment_processed_pub_ = nh.advertise<sensing_on_road::segments_one_batch>("segment_processed", 2);
    perpendicular_pub_ = nh.advertise<sensor_msgs::PointCloud>("perpendicular", 2);
    segment_centroids_pub_ = nh.advertise<sensor_msgs::PointCloud>("segments_centroids", 2);
}

laser_extraction::~laser_extraction()
{
    
}

void laser_extraction::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    //ROS_INFO("--------scan_callback--------");
    laser_scan_ = *scan_in;
    angle_resolution_ = fabsf(scan_in->angle_increment);
    /*
    ROS_INFO("angle_increment %lf", scan_in->angle_increment);
    if(scan_in->angle_increment < 0.00437 && scan_in->angle_increment > 0.00436)        serial_multiple_=4;
    else if(scan_in->angle_increment < 0.00873 && scan_in->angle_increment > 0.00872)   serial_multiple_ =2;
    else if(scan_in->angle_increment < 0.01746 && scan_in->angle_increment > 0.01745)   serial_multiple_ =1;
    else {ROS_ERROR("LIDAR Angle Resolution Not In List."); return;}
    */
    try{projector_.transformLaserScanToPointCloud(ldmrs_single_id_, *scan_in, laser_cloud_laser_, tf_);}
    catch (tf::TransformException& e){ROS_INFO("transform wrong");std::cout << e.what();return;}
    try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *scan_in, laser_cloud_odom_, tf_);}
    catch (tf::TransformException& e){ROS_INFO("transform wrong");std::cout << e.what();return;}

    laser_pub_.publish(laser_cloud_laser_);

    segmentation();
    segments_processing();
}

bool laser_extraction::isOnSameLine(const sensing_on_road::scan_segment & segment_temp,
    const geometry_msgs::Point32 & testPoint ) const
{
    unsigned size_temp = segment_temp.point_serials.size();
    if(size_temp<2)
        return false;

    unsigned serial1_temp = segment_temp.point_serials[size_temp-1];
    unsigned serial2_temp = segment_temp.point_serials[size_temp-2];
    float x1_temp = laser_cloud_laser_.points[serial1_temp].x;
    float y1_temp = laser_cloud_laser_.points[serial1_temp].y;
    float x2_temp = laser_cloud_laser_.points[serial2_temp].x;
    float y2_temp = laser_cloud_laser_.points[serial2_temp].y;

    float x3_temp = testPoint.x;
    float y3_temp = testPoint.y;

    float delt_x32_temp = x3_temp-x2_temp;
    float delt_x21_temp = x2_temp-x1_temp;
    float delt_y32_temp = y3_temp-y2_temp;
    float delt_y21_temp = y2_temp-y1_temp;

    // we only want the result on [0,pi], hence fabs(y)
    float thetha32_temp = atan2f(fabsf(delt_y32_temp),delt_x32_temp);
    float thetha21_temp = atan2f(fabsf(delt_y21_temp),delt_x21_temp);

    //check if the point is on the same line, with angle difference +-5 degree;
    if( fabs(thetha21_temp-thetha32_temp) > fmutil::d2r(5) )
        return false;

    //ROS_INFO("connect!!!");
    return true;
}

/// distance between 2 geometry_msgs::Point32 points
float pt_distance(const geometry_msgs::Point32 & p1, const geometry_msgs::Point32 & p2)
{
    return fmutil::distance(p1.x, p1.y, p2.x, p2.y);
}

/// Connection criteria: points must either be close from one another,
/// or form a line.
bool laser_extraction::isConnected(const sensing_on_road::scan_segment & segment_temp,
                                    const unsigned ip) const
{
    assert( ip>=1 );
    unsigned last_serial_temp = (unsigned)(laser_cloud_laser_.channels.front().values[ip-1]);
    float range_temp = laser_scan_.ranges[last_serial_temp];
    float dis_threshold = 0;
    if(range_temp< DIS_RANGE){dis_threshold = ASSOCIATE_WITHIN_RANGE_THRESH;}
    //else{ dis_threshold = ASSOCIATE_OVER_5_THRESH;}
    else{dis_threshold = ASSOCIATE_OVER_RANGE_MULTI * range_temp * fabsf(laser_scan_.angle_increment);}

    float d = pt_distance(laser_cloud_laser_.points[ip], laser_cloud_laser_.points[ip-1]);
    if(d < dis_threshold)
        return true;

    return isOnSameLine(segment_temp, laser_cloud_laser_.points[ip]);
}

/// Search the laser scan for segments: sets of points that
/// are connected. On output, extracted_segments_laser_ is generated.
bool laser_extraction::extract_segments(sensor_msgs::PointCloud *segment_border)
{
    extracted_segments_laser_.header = laser_cloud_laser_.header;
    extracted_segments_laser_.segments.clear();
    sensing_on_road::scan_segment segment_temp;

    for(unsigned ip=0; ip!=laser_cloud_laser_.points.size(); ip++)
    {
        //channel values are index already;
        unsigned serial_temp = (unsigned)(laser_cloud_laser_.channels.front().values[ip]);
        if(serial_temp>laser_scan_.ranges.size()-1)
        {
            ROS_ERROR("Unexpect Overflow");
            return false;
        }

        if(ip==0)
        {
            segment_temp.beam_serials.push_back(serial_temp);
            segment_temp.point_serials.push_back(ip);
            //segment_border->points.push_back(laser_cloud_laser_.points[ip]);
            continue;
        }

        if( isConnected(segment_temp, ip) )
        {
            //ROS_INFO("Connect");
            segment_temp.beam_serials.push_back(serial_temp);
            segment_temp.point_serials.push_back(ip);
            if(ip==(laser_cloud_laser_.points.size()-1))
                extracted_segments_laser_.segments.push_back(segment_temp);
        }
        else
        {
            //ROS_INFO("segment %d", extracted_segments_laser_.segments.size()-1+1);
            //ROS_INFO("----------------New Segment----------");
            extracted_segments_laser_.segments.push_back(segment_temp);
            segment_temp.beam_serials.clear();
            segment_temp.point_serials.clear();
            segment_temp.beam_serials.push_back(serial_temp);
            segment_temp.point_serials.push_back(ip);
            //segment_border->points.push_back(laser_cloud_laser_.points[ip]);

            if(ip==(laser_cloud_laser_.points.size()-1))
            {
                extracted_segments_laser_.segments.push_back(segment_temp);
                //segment_border->points.pop_back();
            }
        }

        //ROS_INFO("x, y, dis_pxy: %5f, %5f, %5f", laser_cloud_laser_.points[ip].x, laser_cloud_laser_.points[ip].y, dis_pxy);
    }

    return true;
}
using namespace std;
void laser_extraction::connect_segments()
{

    vector<sensing_on_road::scan_segment> & segments = extracted_segments_laser_.segments;

    for(unsigned is=0; is< segments.size(); is++)
    {
        segments[is].connect_front_segment = -1;
        segments[is].connect_back_segment  = -1;
    }

    if(segments.size()==0) return;
    for(unsigned is=0; is<segments.size()-1; is++)
    {
        for(unsigned ic = is+1; ic<segments.size(); ic++)
        {
            unsigned last_serial_temp = segments[is].beam_serials.back();
            float range_temp = laser_scan_.ranges[last_serial_temp];
            float dis_threshold = 0;
            if(range_temp<DIS_RANGE){dis_threshold = ASSOCIATE_WITHIN_RANGE_THRESH;}
            else{dis_threshold = ASSOCIATE_OVER_RANGE_MULTI * range_temp * fabsf(laser_scan_.angle_increment);}

            unsigned serial_number1 = segments[is].point_serials.back();
            unsigned serial_number2 = segments[ic].point_serials.front();
            const geometry_msgs::Point32 & right_seg_last = laser_cloud_laser_.points[serial_number1];
            const geometry_msgs::Point32 & left_seg_first = laser_cloud_laser_.points[serial_number2];
            float dis_pxy = pt_distance(right_seg_last, left_seg_first);
            
            if(dis_pxy < dis_threshold)
            {                
                segments[is].connect_back_segment = ic; 
                    
                if(segments[ic].connect_front_segment  == -1)
                {
                    if(segments[is].connect_front_segment ==-1)
                    {
                        segments[ic].connect_front_segment = is;
                    }
                    else
                    {
                       segments[ic].connect_front_segment = segments[is].connect_front_segment;
                    }
                } 
                else
                {
                    //meaning already get connected by front segments;
                    int most_front_serial = segments[ic].connect_front_segment;
                    //remember to set process its connected predecessor;
                    //record its front connected segment before chaning;
                    int front_serial      = segments[is].connect_front_segment;
                    //set the connect_front_segment to the most front one, serving as type label in later processing;
                    segments[is].connect_front_segment = most_front_serial;
                    
                    while (front_serial != -1)
                    {
                        int current_serial = front_serial;
                        front_serial = segments[front_serial].connect_front_segment;
                        segments[current_serial].connect_front_segment = most_front_serial; 
                    }
                }
                break;
            }
        }
    }
}

void laser_extraction::merge_segments()
{
    vector<sensing_on_road::scan_segment> & segments = extracted_segments_laser_.segments;
    delete_connected_segments_.clear();

    for(unsigned int is=0; is< segments.size(); is++)
    {
        if(segments[is].connect_front_segment == -1)
        {
            std::vector<int> connected_segments;
            for(unsigned int iss=0; iss< segments.size(); iss++)
            {
                if(segments[iss].connect_front_segment == (int) is)
                {
                    connected_segments.push_back(iss);
                    delete_connected_segments_.push_back(iss);
                }
            }
            
            for(unsigned int cs=0; cs<connected_segments.size(); cs++)
            {
                unsigned int connect_seg_serial = connected_segments[cs];
                for(unsigned int ipp=0; ipp< segments[connect_seg_serial].point_serials.size(); ipp++)
                {
                    unsigned int P_serial_tmp = segments[connect_seg_serial].point_serials[ipp];
                    unsigned int B_serial_tmp = segments[connect_seg_serial].beam_serials[ipp];
                    segments[is].point_serials.push_back(P_serial_tmp);
                    segments[is].beam_serials.push_back(B_serial_tmp);
                }
            }
        }
        else {continue;}
    }
}

void laser_extraction::delete_segments()
{
    //bubble_sorting, from large to small; preparing for "delete" operation;
    for(unsigned int y = 0; y < delete_connected_segments_.size(); y++)
    {
        for(unsigned int k = 0; k < delete_connected_segments_.size()-1-y; k++)
        {
            if(delete_connected_segments_[k] < delete_connected_segments_[k+1])
            {
                int seg_serial = delete_connected_segments_[k+1];
                delete_connected_segments_[k+1]  = delete_connected_segments_[k];
                delete_connected_segments_[k]  = seg_serial;
            }
        }
    }
    
    for(unsigned int dc=0; dc<delete_connected_segments_.size(); dc++)
    {
        ROS_DEBUG("delete segment: %d", delete_connected_segments_[dc]);
        if(delete_connected_segments_[dc]<0){ROS_DEBUG("Unexpected Scenario");}
        unsigned int delete_seg_serial = (unsigned int)delete_connected_segments_[dc];
        
        //erase...
        // Note: erasing from a vector is very inefficient !
        // Fixing this would require using a list.
        // Since we cannot modify the class (it's a message), we can convert
        // to list first, then convert back to vector.
        // However, if this is not executed too much, it won't affect the speed
        // so much...
        
        extracted_segments_laser_.segments.erase(extracted_segments_laser_.segments.begin()+delete_seg_serial);
    }
}

//segment one single scan into multiple segments;
//two steps: (1)largest distance; (2)angles of lines;
void laser_extraction::segmentation()
{
    //ROS_INFO("segmentation");
    sensor_msgs::PointCloud segment_border;
    segment_border.header = laser_cloud_laser_.header;

    if( !extract_segments(&segment_border) ) return;


    //----------------------------------------------------------------------------------------
    //-------------try to connect segments that are segmented by holes, if any;---------------
    //----------------------------------------------------------------------------------------
    //ROS_INFO("1.Try to connect segments");
    connect_segments();

    //ROS_INFO("2. to merge segments");
    merge_segments();

    //ROS_INFO("3. to delete segments");
    delete_segments();


    for(unsigned int is=0; is<extracted_segments_laser_.segments.size(); is++)
    {
        unsigned i = extracted_segments_laser_.segments[is].point_serials.front();
        segment_border.points.push_back(laser_cloud_laser_.points[i]);
    }


    //int segment_number_temp = extracted_segments_laser_.segments.size();
    //ROS_INFO("segments number: %d", segment_number_temp);
    segment_border_pub_.publish(segment_border);
}

// condition definition of segments:
#define SEG_TYPE_BORDER 0
#define SEG_TYPE_FOREGROUND 1
#define SEG_TYPE_BACKGROUND 2
#define SEG_TYPE_MAX_READING 3

void laser_extraction::check_right_condition(unsigned is)
{
    sensing_on_road::scan_segment & segment = extracted_segments_laser_.segments[is];
    if(is==0)
    {
        if(segment.beam_serials.front()==0)
            segment.right_condition = SEG_TYPE_BORDER;
        else
            segment.right_condition = SEG_TYPE_MAX_READING;
    }
    else
    {
        unsigned serial1_temp = segment.beam_serials.front();
        unsigned serial1_p_temp = segment.point_serials.front();
        unsigned serial2_temp = (unsigned) laser_cloud_laser_.channels.front().values[serial1_p_temp-1];

        if(serial1_temp==serial2_temp-1)
        {
            if(laser_scan_.ranges[serial1_temp]<=laser_scan_.ranges[serial2_temp])
                segment.right_condition = SEG_TYPE_FOREGROUND;
            else
                segment.right_condition = SEG_TYPE_BACKGROUND;
        }
        else
            segment.right_condition = SEG_TYPE_MAX_READING;
    }
}

void laser_extraction::check_left_condition(unsigned is)
{
    sensing_on_road::scan_segment & segment = extracted_segments_laser_.segments[is];
    // pay attention to usigned int substraction, which may lead to extreme big numbers;
    if(is==extracted_segments_laser_.segments.size()-1)
    {
        if(segment.beam_serials.back() == laser_scan_.ranges.size()-1)
            segment.left_condition = SEG_TYPE_BORDER;
        else
            segment.left_condition = SEG_TYPE_MAX_READING;
    }
    else
    {
        unsigned serial1_temp = segment.beam_serials.back();
        unsigned serial1_p_temp = segment.point_serials.back();
        unsigned serial2_temp = (unsigned) laser_cloud_laser_.channels.front().values[serial1_p_temp+1];
        
        if(serial1_temp+1==serial2_temp)
        {
            if(laser_scan_.ranges[serial1_temp]<=laser_scan_.ranges[serial2_temp])
                segment.left_condition = SEG_TYPE_FOREGROUND;
            else
                segment.left_condition = SEG_TYPE_BACKGROUND;
        }
        else
            segment.left_condition = SEG_TYPE_MAX_READING;
    }
}

//process segments for more meanings, with minimum information loss;
void laser_extraction::segments_processing()
{
    segment_centroids_.header = laser_cloud_laser_.header;
    segment_centroids_.points.clear();

    //ROS_INFO("segments_processing");
    for(unsigned is=0; is<extracted_segments_laser_.segments.size(); is++)
    {
        sensing_on_road::scan_segment & segment = extracted_segments_laser_.segments[is];
        segment.rightPoint = laser_cloud_odom_.points[segment.point_serials.front()];
        segment.leftPoint = laser_cloud_odom_.points[segment.point_serials.back()];

        // first step: condition definition of segments
        check_right_condition(is);
        check_left_condition(is);

        /* 2nd step:
        * further extract line pieces;
        * generate static features;
        */
        single_segment_processing(segment);
    }
    //ROS_INFO("Publish Begin");
    segment_processed_pub_.publish(extracted_segments_laser_);
    segment_centroids_pub_.publish(segment_centroids_);
    //ROS_INFO("Publish Over");
}

/// returns orientation of vector p1-p2
float orientation(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
{
    float delt_x_temp = p2.x - p1.x;
    float delt_y_temp = p2.y - p1.y;
    return atan2f(fabsf(delt_y_temp),delt_x_temp);
}

/// returns orientation of line
float orientation(const sensing_on_road::line_piece & line)
{
    return atan2f(fabsf(line.para_A),line.para_B);
}

bool check_perpendicular(const sensing_on_road::scan_segment & segment,
                         const vector<unsigned> & long_lines)
{
    if(long_lines.size()<2)
        return false;

    const sensing_on_road::line_piece & line_temp0 = segment.line_pieces[long_lines[0]];
    const sensing_on_road::line_piece & line_temp1 = segment.line_pieces[long_lines[1]];
    float thetha0 = orientation(line_temp0);
    float thetha1 = orientation(line_temp1);
        
    if( fabsf(thetha0-thetha1-M_PI_2) > fmutil::d2r(10) )
        return false;

    //to deal with the situation where two far-away line segments perpendicular to each other;
    if( line_temp0.point_serials[0] <= line_temp1.point_serials[0] )
    {
        if( line_temp1.point_serials.front() - line_temp0.point_serials.back() > 5 )
            return false;
    }
    else
    {
        if( line_temp0.point_serials.front() - line_temp1.point_serials.back() > 5 )
            return false;
    }

    return true;
}

std::vector<unsigned> getLongLines(const sensing_on_road::scan_segment &segment)
{
    std::vector<unsigned> long_lines;

    for(unsigned il=0; il<segment.line_pieces.size(); il++)
        if(segment.line_pieces[il].point_serials.size() >=3 )
            long_lines.push_back(il);

    if( ! long_lines.empty() )
    {
        //bubble-sorting;
        for(unsigned iz=0; iz<long_lines.size()-1; iz++)
        {
            for(unsigned ip=0; ip<long_lines.size()-1-iz; ip++)
            {
                unsigned s1 = segment.line_pieces[long_lines[ip]].point_serials.size();
                unsigned s2 = segment.line_pieces[long_lines[ip+1]].point_serials.size();
                if( s1<s2 )
                {
                    unsigned s_temp = long_lines[ip+1];
                    long_lines[ip+1] = long_lines[ip];
                    long_lines[ip] = s_temp;
                }
            }
        }
    }

    return long_lines;
}

class SegmentDimensions
{
public:
    float max_x, min_x, max_y, min_y, x_sum, y_sum;
    SegmentDimensions() :
        max_x(-1000.0), min_x(1000.0), max_y(-1000.0), min_y(1000.0),
        x_sum(0.0), y_sum(0.0)
    {
    }

    void add(const geometry_msgs::Point32 & p)
    {
        if(p.x > max_x)  max_x = p.x;
        if(p.y > max_y)  max_y = p.y;
        if(p.x <= min_x) min_x = p.x;
        if(p.y <= min_y) min_y = p.y;
        x_sum += p.x;
        y_sum += p.y;
    }
};


/// TODO: find a better name (ask baoxing)
void laser_extraction::segment_dimensions(sensing_on_road::scan_segment &segment_para,
    float *x_sum, float *y_sum) const
{
    SegmentDimensions sDim;
    
    sensing_on_road::line_piece line_temp;
    bool new_line_flag_temp = true;
    
    for(unsigned ip=0; ip!=segment_para.point_serials.size(); ip++)
    {
        if(ip==0)
        {
            unsigned p = segment_para.point_serials[ip];
            line_temp.point_serials.push_back(p);
            sDim.x_sum += laser_cloud_laser_.points[p].x;
            sDim.y_sum += laser_cloud_laser_.points[p].y;
            continue;
        }
        
        //float serial_temp0 = segment_para.point_serials[ip-1];
        float serial_temp0 = line_temp.point_serials.front();
        float serial_temp1 = segment_para.point_serials[ip];
        geometry_msgs::Point32 point_temp0  = laser_cloud_laser_.points[serial_temp0];
        geometry_msgs::Point32 point_temp1  = laser_cloud_laser_.points[serial_temp1];
        
        sDim.add(point_temp1);
        
        if(new_line_flag_temp)
        {
            line_temp.point_serials.push_back(serial_temp1);
            new_line_flag_temp = false;
            continue;
        }
        
        float serial_temp2 = line_temp.point_serials.back();
        geometry_msgs::Point32 point_temp2  = laser_cloud_laser_.points[serial_temp2];
        float new_angle_temp = orientation(point_temp0, point_temp1);
        float last_angle_temp = orientation(point_temp0, point_temp2);
        
        if( fabs(new_angle_temp-last_angle_temp) < fmutil::d2r(5) )
        {
            line_temp.point_serials.push_back(serial_temp1);
        }
        else
        {
            line_piece_calculation(line_temp);
            segment_para.line_pieces.push_back(line_temp);
            line_temp.point_serials.clear();
            line_temp.point_serials.push_back(serial_temp1);
            new_line_flag_temp = true;
        }
    }
    
    line_piece_calculation(line_temp);
    segment_para.line_pieces.push_back(line_temp);

    segment_para.max_dimension = fmutil::mag(sDim.max_x-sDim.min_x, sDim.max_y-sDim.min_y);
    *x_sum = sDim.x_sum;
    *y_sum = sDim.y_sum;
}

void laser_extraction::single_segment_processing(sensing_on_road::scan_segment &segment_para)
{
    float x_sum=0, y_sum=0;
    segment_dimensions(segment_para, &x_sum, &y_sum);
    
    geometry_msgs::PointStamped centroid_laser, centroid_odom;
    centroid_laser.header = laser_cloud_laser_.header;
    centroid_laser.point.x = x_sum / ((float)segment_para.point_serials.size());
    centroid_laser.point.y = y_sum / ((float)segment_para.point_serials.size());
    centroid_laser.point.z = laser_cloud_laser_.points[segment_para.point_serials[0]].z;
    tf_.transformPoint("odom", centroid_laser, centroid_odom);

    geometry_msgs::Point32 centroid_tmp;
    centroid_tmp.x = x_sum / ((float)segment_para.point_serials.size());
    centroid_tmp.y = y_sum / ((float)segment_para.point_serials.size());
    centroid_tmp.z = laser_cloud_laser_.points[segment_para.point_serials[0]].z;
    segment_centroids_.points.push_back(centroid_tmp);

    segment_para.segment_centroid_laser = centroid_laser;
    segment_para.segment_centroid_odom = centroid_odom;
    segment_para.centroid_range_to_laser = sqrtf(
        powf(centroid_laser.point.x,2) + powf(centroid_laser.point.y,2) );


    std::vector<unsigned> long_lines = getLongLines(segment_para);

    if( check_perpendicular(segment_para, long_lines) )
    {
        segment_para.perpendicular_lines.push_back(long_lines[0]);
        segment_para.perpendicular_lines.push_back(long_lines[1]);

        //debugging-visualization purposes;
        sensor_msgs::PointCloud perpendiculer_lines_pts;
        perpendiculer_lines_pts.header = laser_cloud_laser_.header;
        for(unsigned ill=0; ill<2; ill++)
        {
            unsigned line_serial = segment_para.perpendicular_lines[ill];
            const vector<unsigned> & pts = segment_para.line_pieces[line_serial].point_serials;
            for(unsigned ipp=0; ipp<pts.size(); ipp++)
                perpendiculer_lines_pts.points.push_back(laser_cloud_laser_.points[pts[ipp]]);
        }
        perpendicular_pub_.publish(perpendiculer_lines_pts);
    }
}

// caculate line information from "point_serials";
// be able to take care of single point;
void laser_extraction::line_piece_calculation(sensing_on_road::line_piece &line) const
{
    if(line.point_serials.size()==1) {
        line.length = 0.0;
        return;
    }

    float x1_temp = laser_cloud_laser_.points[line.point_serials.front()].x;
    float y1_temp = laser_cloud_laser_.points[line.point_serials.front()].y;
    float x2_temp = laser_cloud_laser_.points[line.point_serials.back()].x;
    float y2_temp = laser_cloud_laser_.points[line.point_serials.back()].y;
    line.para_A = y2_temp-y1_temp;
    line.para_B = x1_temp-x2_temp;
    line.para_C = line.para_A*x1_temp+line.para_B*y1_temp;
    line.length = fmutil::mag(line.para_A, line.para_B);
}

} //namespace



int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_extractor");
    sensing_on_road::laser_extraction extractor;
    ros::spin();
    return 0;
}
