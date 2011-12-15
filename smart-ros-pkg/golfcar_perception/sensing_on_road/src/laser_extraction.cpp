#include "laser_extraction.h"

#define DIS_RANGE 10
#define ASSOCIATE_WITHIN_RNAGE_THRESH 0.6
#define ASSOCIATE_OVER_RANGE_MULTI 7

/*
#define ASSOCIATE_WITHIN_5_THRESH 0.5
#define ASSOCIATE_OVER_5_MULTI 10
*/

namespace sensing_on_road{
    
    laser_extraction::laser_extraction()
    {
        
        private_nh_.param("laser_frame_id",     ldmrs_single_id_,      std::string("ldmrsAssemb"));
        private_nh_.param("odom_frame_id",      odom_frame_id_,        std::string("odom"));
        
        laser_sub_.subscribe(nh_, "sickldmrs/assembled", 10);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, ldmrs_single_id_, 10);
        tf_filter_->registerCallback(boost::bind(&laser_extraction::scan_callback, this, _1));
        tf_filter_->setTolerance(ros::Duration(0.05));
        laser_pub_ = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 2);
        segment_border_pub_ = nh_.advertise<sensor_msgs::PointCloud>("segment_border", 2);
        segment_processed_pub_ = nh_.advertise<sensing_on_road::segments_one_batch>("segment_processed", 2);
        perpendicular_pub_ = nh_.advertise<sensor_msgs::PointCloud>("perpendicular", 2);
        segment_centroids_pub_ = nh_.advertise<sensor_msgs::PointCloud>("segments_centroids", 2);
    }   
    laser_extraction::~laser_extraction(){}

    void laser_extraction::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        //ROS_INFO("--------scan_callback--------");
        laser_scan_ = *scan_in;
        angle_resolution_ = scan_in->angle_increment;
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
    
    //segment one single scan into multiple segments;
    //two steps: (1)largest distance; (2)angles of lines;
    void laser_extraction::segmentation()
    {
        //ROS_INFO("segmentation");
        extracted_segments_laser_.header = laser_cloud_laser_.header;
        extracted_segments_laser_.segments.clear();
        
        sensor_msgs::PointCloud segment_border;
        segment_border.header = laser_cloud_laser_.header;
        
        unsigned int serial_temp, last_serial_temp;
        float range_temp, dis_threshold, dis_px, dis_py, dis_pxy;
        sensing_on_road::scan_segment segment_temp;
        
        for(std::vector<geometry_msgs::Point32>::size_type ip=0; ip!=laser_cloud_laser_.points.size(); ip++)
        {
            //channel values are index already;
            serial_temp = (unsigned int)(laser_cloud_laser_.channels.front().values[ip]);
            if(serial_temp>laser_scan_.ranges.size()-1){ROS_ERROR("Unexpect Overflow"); return;}
            
            if(ip==0)
            {
                segment_temp.beam_serials.push_back(serial_temp);
                segment_temp.point_serials.push_back(ip);
                //segment_border.points.push_back(laser_cloud_laser_.points[ip]);
            }
            else
            {
                last_serial_temp = (unsigned int)(laser_cloud_laser_.channels.front().values[ip-1]);
                range_temp = laser_scan_.ranges[last_serial_temp];
                if(range_temp< DIS_RANGE){dis_threshold = ASSOCIATE_WITHIN_RNAGE_THRESH;}
                //else{ dis_threshold = ASSOCIATE_OVER_5_THRESH;}
                else{dis_threshold = ASSOCIATE_OVER_RANGE_MULTI * range_temp * laser_scan_.angle_increment;}
                
                dis_px  =laser_cloud_laser_.points[ip].x-laser_cloud_laser_.points[ip-1].x;
                dis_py  =laser_cloud_laser_.points[ip].y-laser_cloud_laser_.points[ip-1].y;
                dis_pxy =sqrtf(dis_px*dis_px+dis_py*dis_py);
                
                bool connect_flag = true;
                if(dis_pxy < dis_threshold) {connect_flag = true; }
                //else{connect_flag = false;ROS_INFO("Disconnect");}
                else 
                {
                    unsigned int size_temp = segment_temp.point_serials.size();
                    if(size_temp<2) connect_flag = false;
                    else
                    {
                        unsigned int serial1_temp = segment_temp.point_serials[size_temp-1];
                        unsigned int serial2_temp = segment_temp.point_serials[size_temp-2];
                        float x1_temp = laser_cloud_laser_.points[serial1_temp].x;
                        float y1_temp = laser_cloud_laser_.points[serial1_temp].y;
                        float x2_temp = laser_cloud_laser_.points[serial2_temp].x;
                        float y2_temp = laser_cloud_laser_.points[serial2_temp].y;
                        
                        float x3_temp = laser_cloud_laser_.points[ip].x;
                        float y3_temp = laser_cloud_laser_.points[ip].y;
                        float thetha32_temp, thetha21_temp;
                        float delt_x32_temp = x3_temp-x2_temp;
                        float delt_x21_temp = x2_temp-x1_temp;
                        float delt_y32_temp = y3_temp-y2_temp;
                        float delt_y21_temp = y2_temp-y1_temp;
                        
                        if(delt_x32_temp==0.0){thetha32_temp = (float)M_PI_2;}
                        else {thetha32_temp=atan2f(delt_y32_temp,delt_x32_temp);}
                        if(delt_x21_temp==0.0){thetha21_temp = (float)M_PI_2;}
                        else {thetha21_temp=atan2f(delt_y21_temp,delt_x21_temp);}
                        if(thetha32_temp<0) thetha32_temp=thetha32_temp+(float)M_PI;
                        if(thetha21_temp<0) thetha21_temp=thetha21_temp+(float)M_PI;
                        
                        //check if the point is on the same line, with angle difference +-5 degree;
                        if(thetha21_temp-thetha32_temp>0.0872||thetha21_temp-thetha32_temp<-0.0872){connect_flag = false;}
                        else{ROS_INFO("connect!!!");}                                       
                    }
                }
                                                    
                if(connect_flag)
                {
                    //ROS_INFO("Connect");
                    segment_temp.beam_serials.push_back(serial_temp);
                    segment_temp.point_serials.push_back(ip);
                    if(ip==(laser_cloud_laser_.points.size()-1)) 
                    {
                        extracted_segments_laser_.segments.push_back(segment_temp);
                    }
                    
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
                    //segment_border.points.push_back(laser_cloud_laser_.points[ip]);
                    
                    if(ip==(laser_cloud_laser_.points.size()-1))
                    {
                        extracted_segments_laser_.segments.push_back(segment_temp);
                        //segment_border.points.pop_back();
                    }
                }
                
                //ROS_INFO("x, y, dis_pxy: %5f, %5f, %5f", laser_cloud_laser_.points[ip].x, laser_cloud_laser_.points[ip].y, dis_pxy);  
            }           
        }
        
        
        //----------------------------------------------------------------------------------------
        //-------------try to connect segments that are segmented by holes, if any;---------------
        //----------------------------------------------------------------------------------------
        //ROS_INFO("1.Try to connect segments");
        for(unsigned int is=0; is< extracted_segments_laser_.segments.size(); is++)
        {
            extracted_segments_laser_.segments[is].connect_front_segment =-1;
            extracted_segments_laser_.segments[is].connect_back_segment =-1;
        }
        
        for(unsigned int is=0; is< extracted_segments_laser_.segments.size()-1; is++)
        {
            for(unsigned int ic = is+1; ic< extracted_segments_laser_.segments.size(); ic++)
            {
                last_serial_temp = extracted_segments_laser_.segments[is].beam_serials.back();
                range_temp = laser_scan_.ranges[last_serial_temp];
                if(range_temp<DIS_RANGE){dis_threshold = ASSOCIATE_WITHIN_RNAGE_THRESH;}
                else{dis_threshold = ASSOCIATE_OVER_RANGE_MULTI * range_temp * laser_scan_.angle_increment;}
                
                unsigned int serial_number1 = extracted_segments_laser_.segments[is].point_serials.back();
                unsigned int serial_number2 = extracted_segments_laser_.segments[ic].point_serials.front();
                geometry_msgs::Point32 right_seg_last = laser_cloud_laser_.points[serial_number1];
                geometry_msgs::Point32 left_seg_first = laser_cloud_laser_.points[serial_number2];
                dis_px  =right_seg_last.x-left_seg_first.x;
                dis_py  =right_seg_last.y-left_seg_first.y;
                dis_pxy =sqrtf(dis_px*dis_px+dis_py*dis_py);
                if(dis_pxy < dis_threshold)
                {
                    extracted_segments_laser_.segments[is].connect_back_segment = ic; 
                    extracted_segments_laser_.segments[ic].connect_front_segment = is; 
                    ROS_INFO("Connect is %d: (%5f, %5f) and ic %d: (%5f, %5f)", is, right_seg_last.x, right_seg_last.y, ic, left_seg_first.x, left_seg_first.y);
                    break;
                }
            }
        }
        
        //ROS_INFO("2. to merge segments");
        std::vector<int> delete_connected_segments;
        for(unsigned int is=0; is< extracted_segments_laser_.segments.size(); is++)
        {
            if(extracted_segments_laser_.segments[is].connect_front_segment != -1) continue;
            else
            {
                std::vector<int> connected_segments;
                int front_seg = is;
                int back_seg =  extracted_segments_laser_.segments[is].connect_back_segment;
                while(back_seg !=-1)
                {
                    front_seg = back_seg;
                    back_seg  = extracted_segments_laser_.segments[front_seg].connect_back_segment;
                    connected_segments.push_back(front_seg);
                    delete_connected_segments.push_back(front_seg);
                }
                
                for(unsigned int cs=0; cs<connected_segments.size(); cs++)
                {
                    unsigned int connect_seg_serial = connected_segments[cs];
                    for(unsigned int ipp=0; ipp< extracted_segments_laser_.segments[connect_seg_serial].point_serials.size(); ipp++)
                    {
                        unsigned int P_serial_tmp = extracted_segments_laser_.segments[connect_seg_serial].point_serials[ipp];
                        unsigned int B_serial_tmp = extracted_segments_laser_.segments[connect_seg_serial].beam_serials[ipp];
                        extracted_segments_laser_.segments[is].point_serials.push_back(P_serial_tmp);
                        extracted_segments_laser_.segments[is].beam_serials.push_back(B_serial_tmp);
                    }
                }
            }
        }
        
        //ROS_INFO("3. to delete segments");
        //bubble_sorting, from large to small; preparing for "delete" operation;
        for(unsigned int y = 0; y < delete_connected_segments.size(); y++)
        {
            for(unsigned int k = 0; k < delete_connected_segments.size()-1-y; k++)
            {
                if(delete_connected_segments[k] < delete_connected_segments[k+1])
                {
                    int seg_serial = delete_connected_segments[k+1];
                    delete_connected_segments[k+1]  = delete_connected_segments[k];
                    delete_connected_segments[k]  = seg_serial;
                }
            }
        }
        for(unsigned int dc=0; dc<delete_connected_segments.size(); dc++)
        {
            //ROS_INFO("delete segment: %d", delete_connected_segments[dc]);
            if(delete_connected_segments[dc]<0){ROS_INFO("Unexpected Scenario");}
            unsigned int delete_seg_serial = (unsigned int)delete_connected_segments[dc];
            extracted_segments_laser_.segments.erase(extracted_segments_laser_.segments.begin()+delete_seg_serial);
        }
        
        
        for(unsigned int is=0; is< extracted_segments_laser_.segments.size(); is++)
        {
            segment_border.points.push_back(laser_cloud_laser_.points[extracted_segments_laser_.segments[is].point_serials.front()]);
        }
        
        
        //int segment_number_temp = extracted_segments_laser_.segments.size();
        //ROS_INFO("segments number: %d", segment_number_temp);
        segment_border_pub_.publish(segment_border);
    }

    //process segments for more meanings, with minimum information loss;
    void laser_extraction::segments_processing()
    {
        segment_centroids_.header = laser_cloud_laser_.header;
        segment_centroids_.points.clear();
        
        //ROS_INFO("segments_processing");
        for(std::vector<sensing_on_road::scan_segment>::size_type is=0; is!=extracted_segments_laser_.segments.size(); is++)
        {
            unsigned int temp_serial;
            temp_serial = extracted_segments_laser_.segments[is].point_serials.front();
            extracted_segments_laser_.segments[is].rightPoint = laser_cloud_odom_.points[temp_serial];
            temp_serial = extracted_segments_laser_.segments[is].point_serials.back();
            extracted_segments_laser_.segments[is].leftPoint = laser_cloud_odom_.points[temp_serial];
            
            /* 1st step:
             * condition definition of segments: 
             * 0 for field of view borders; 
             * 1 for foreground; 
             * 2 for background; 
             * 3 for maximum readings aside;
             * be careful to cover all situations and avoid bugs;
             */ 
            
            //1. check right_condition;
            if(is==0)
            {
                if(extracted_segments_laser_.segments[is].beam_serials.front()==0) extracted_segments_laser_.segments[is].right_condition = 0;
                else  extracted_segments_laser_.segments[is].right_condition =3;
            }
            else
            {
                unsigned int serial1_temp = extracted_segments_laser_.segments[is].beam_serials.front();
                unsigned int serial1_p_temp = extracted_segments_laser_.segments[is].point_serials.front();
                unsigned int serial2_temp = (unsigned int)(laser_cloud_laser_.channels.front().values[serial1_p_temp-1]);
                
                if(serial1_temp==serial2_temp-1)
                {
                    if(laser_scan_.ranges[serial1_temp]<=laser_scan_.ranges[serial2_temp])
                    {extracted_segments_laser_.segments[is].right_condition=1;}
                    else {extracted_segments_laser_.segments[is].right_condition =2;}
                }
                else{extracted_segments_laser_.segments[is].right_condition =3;}
            }
            
            //2. check left_condition;
            // pay attention to usigned int substraction, which may lead to extreme big numbers;
            if(is==extracted_segments_laser_.segments.size()-1)
            {
                if(extracted_segments_laser_.segments[is].beam_serials.back()== laser_scan_.ranges.size()-1) extracted_segments_laser_.segments[is].left_condition = 0;
                else  extracted_segments_laser_.segments[is].left_condition = 3;
            }
            else 
            {
                unsigned int serial1_temp = extracted_segments_laser_.segments[is].beam_serials.back();
                unsigned int serial1_p_temp = extracted_segments_laser_.segments[is].point_serials.back();
                unsigned int serial2_temp = (unsigned int)(laser_cloud_laser_.channels.front().values[serial1_p_temp+1]);
                
                if(serial1_temp==serial2_temp-1)
                {
                    if(laser_scan_.ranges[serial1_temp]<=laser_scan_.ranges[serial2_temp])
                    {extracted_segments_laser_.segments[is].left_condition=1;}
                    else {extracted_segments_laser_.segments[is].left_condition =2;}
                }
                else{extracted_segments_laser_.segments[is].left_condition =3;}
            }
            
            /* 2nd step: 
             * further extract line pieces;
             * generate static features;
             */ 
            single_segment_processing(extracted_segments_laser_.segments[is]);
        }
        //ROS_INFO("Publish Begin");
        segment_processed_pub_.publish(extracted_segments_laser_);
        segment_centroids_pub_.publish(segment_centroids_);
        //ROS_INFO("Publish Over");
    }
    

    
    
    inline void laser_extraction::single_segment_processing(sensing_on_road::scan_segment &segment_para)
    {   
        //for segment calculation;
        float max_x_temp        = -1000.0; 
        float min_x_temp        =  1000.0;
        float max_y_temp        = -1000.0; 
        float min_y_temp        =  1000.0;
        float x_sum_temp        =  0.0;
        float y_sum_temp        =  0.0;
        
        sensing_on_road::line_piece line_temp;
        
        bool new_line_flag_temp = true;
        
        geometry_msgs::Point32 point_temp0, point_temp1;
        float serial_temp0, serial_temp1;
        //newly revised @_@;
        geometry_msgs::Point32 point_temp2;
        float serial_temp2;
        
        float last_angle_temp, new_angle_temp;
        
        for(std::vector<unsigned int>::size_type ip=0; ip!=segment_para.point_serials.size(); ip++)
        {   
            if(ip==0)
            {
                line_temp.point_serials.push_back(segment_para.point_serials[ip]);
                x_sum_temp = x_sum_temp + laser_cloud_laser_.points[segment_para.point_serials[ip]].x;
                y_sum_temp = y_sum_temp + laser_cloud_laser_.points[segment_para.point_serials[ip]].y;
                continue;
			}
            
            //serial_temp0 = segment_para.point_serials[ip-1];
            serial_temp0 = line_temp.point_serials.front();
            serial_temp1 = segment_para.point_serials[ip];
            point_temp0  = laser_cloud_laser_.points[serial_temp0];
            point_temp1  = laser_cloud_laser_.points[serial_temp1];
            
            if(point_temp1.x > max_x_temp)  max_x_temp=point_temp1.x;
            if(point_temp1.y > max_y_temp)  max_y_temp=point_temp1.y;
            if(point_temp1.x <= min_x_temp) min_x_temp=point_temp1.x;
            if(point_temp1.y <= min_y_temp) min_y_temp=point_temp1.y;
            x_sum_temp = x_sum_temp + point_temp1.x;
            y_sum_temp = y_sum_temp + point_temp1.y;
            
            float delt_x_temp = point_temp1.x - point_temp0.x;
            float delt_y_temp = point_temp1.y - point_temp0.y;
            if(delt_x_temp==0.0){new_angle_temp = (float)M_PI_2;}
            else {new_angle_temp=atan2f(delt_y_temp,delt_x_temp);}
            if(new_angle_temp<0) new_angle_temp=new_angle_temp+(float)M_PI;
            
            if(new_line_flag_temp)
            {
                line_temp.point_serials.push_back(serial_temp1);
                new_line_flag_temp=false;
                continue;
            }
            
            //newly revised @_@;
            serial_temp2 = line_temp.point_serials.back();
            point_temp2  = laser_cloud_laser_.points[serial_temp2];
            float delt_x2_temp = point_temp2.x - point_temp0.x;
            float delt_y2_temp = point_temp2.y - point_temp0.y;
            if(delt_x2_temp==0.0){last_angle_temp = (float)M_PI_2;}
            else {last_angle_temp=atan2f(delt_y2_temp,delt_x2_temp);}
            if(last_angle_temp<0) last_angle_temp=last_angle_temp+(float)M_PI;

                    
            if(new_angle_temp-last_angle_temp<0.0872 && new_angle_temp-last_angle_temp>-0.0872)
            {
                line_temp.point_serials.push_back(serial_temp1);
                last_angle_temp = new_angle_temp;
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
        
        float x_dimension_temp = max_x_temp - min_x_temp;
        float y_dimension_temp = max_y_temp - min_y_temp; 
        //segment_para.max_dimension = (x_dimension_temp >y_dimension_temp)? x_dimension_temp : y_dimension_temp;
        segment_para.max_dimension = sqrtf(x_dimension_temp *x_dimension_temp+ y_dimension_temp*y_dimension_temp);
        
        geometry_msgs::PointStamped centroid_laser, centroid_odom;
        centroid_laser.header = laser_cloud_laser_.header;
        centroid_laser.point.x = x_sum_temp/((float)segment_para.point_serials.size());
        centroid_laser.point.y = y_sum_temp/((float)segment_para.point_serials.size());
        centroid_laser.point.z = laser_cloud_laser_.points[segment_para.point_serials[0]].z;
        tf_.transformPoint("odom", centroid_laser, centroid_odom);

        geometry_msgs::Point32 centroid_tmp;
        centroid_tmp.x = x_sum_temp/((float)segment_para.point_serials.size());
        centroid_tmp.y = y_sum_temp/((float)segment_para.point_serials.size());
        centroid_tmp.z = laser_cloud_laser_.points[segment_para.point_serials[0]].z;
        segment_centroids_.points.push_back(centroid_tmp);
        
        segment_para.segment_centroid_laser = centroid_laser;
        segment_para.segment_centroid_odom = centroid_odom;
        segment_para.centroid_range_to_laser=sqrtf(centroid_laser.point.x*centroid_laser.point.x +centroid_laser.point.y*centroid_laser.point.y);
        
        
        std::vector<unsigned int>long_lines;
        
        for(std::vector<unsigned int>::size_type il=0; il!=segment_para.line_pieces.size(); il++)
        {   
            if(segment_para.line_pieces[il].point_serials.size() >=3 )
            {
                long_lines.push_back(il);
            }
        }
        if(long_lines.size()==0){return;}
        
        //bubble-sorting;
        for(std::vector<int>::size_type iz=0; iz<long_lines.size()-1; iz++)
        {
            for(std::vector<int>::size_type ip=0; ip<long_lines.size()-1-iz; ip++) 
             { 
                 unsigned int line1 = long_lines[ip];
                 unsigned int line2 = long_lines[ip+1];
                 
                 if(segment_para.line_pieces[line1].point_serials.size() < segment_para.line_pieces[line2].point_serials.size())
                 {
                     unsigned int s_temp = long_lines[ip+1];
                     long_lines[ip+1] = long_lines[ip];
                     long_lines[ip] = s_temp;
                 }
             }                        
         }
         
         //check if perpedicular;
         if(long_lines.size()>=2)
         {
             sensing_on_road::line_piece line_temp0 = segment_para.line_pieces[long_lines[0]];
             sensing_on_road::line_piece line_temp1 = segment_para.line_pieces[long_lines[1]];
             float thetha0, thetha1;
             
             if(line_temp0.para_B == 0.0){thetha0 = (float)M_PI_2;}
             else{thetha0=atan2f(-line_temp0.para_A/line_temp0.para_B, 1.0);if(thetha0<0)thetha0=thetha0+(float)M_PI;}
             if(line_temp1.para_B == 0.0){thetha1 = (float)M_PI_2;}
             else{thetha1=atan2f(-line_temp1.para_A/line_temp1.para_B, 1.0);if(thetha1<0)thetha1=thetha1+(float)M_PI;}
             if((fabsf(thetha0-thetha1)<M_PI_2+0.174)&&(fabsf(thetha0-thetha1)>M_PI_2-0.174))
             {
                 //to deal with the situation where two far-away line segments perpendicular to each other;
                 if(segment_para.line_pieces[long_lines[0]].point_serials[0]<=segment_para.line_pieces[long_lines[1]].point_serials[0])
                 {
                     if(segment_para.line_pieces[long_lines[1]].point_serials.front()-segment_para.line_pieces[long_lines[0]].point_serials.back()>5)
                     {return;}
                 }
                 else
                 {
                     if(segment_para.line_pieces[long_lines[0]].point_serials.front()-segment_para.line_pieces[long_lines[1]].point_serials.back()>5)
                     {return;}
                 }
                 
                 
                 segment_para.perpendicular_lines.push_back(long_lines[0]);
                 segment_para.perpendicular_lines.push_back(long_lines[1]);
                 
                 //debugging-visualization purposes;
                 sensor_msgs::PointCloud perpendiculer_lines_pts;
                 perpendiculer_lines_pts.header = laser_cloud_laser_.header;
                 unsigned int line_serial, point_serial;
                 for(unsigned int ill=0; ill<2;ill++)
                 {
                    line_serial = segment_para.perpendicular_lines[ill];
                    for(unsigned int ipp=0; ipp<segment_para.line_pieces[line_serial].point_serials.size(); ipp++)
                    {
                        point_serial = segment_para.line_pieces[line_serial].point_serials[ipp];
                        perpendiculer_lines_pts.points.push_back(laser_cloud_laser_.points[point_serial]);
                    }
                 }
                 perpendicular_pub_.publish(perpendiculer_lines_pts);
             }
         }       
    }
    
    /* caculate line information from "point_serials"; 
     * be able to take care of single point;
     */ 
   inline void laser_extraction::line_piece_calculation(sensing_on_road::line_piece &line_piece_para)
    {
        if(line_piece_para.point_serials.size()==1){line_piece_para.length=0.0;}
        else
        {
            float x1_temp = laser_cloud_laser_.points[line_piece_para.point_serials.front()].x;
            float y1_temp = laser_cloud_laser_.points[line_piece_para.point_serials.front()].y;
            float x2_temp = laser_cloud_laser_.points[line_piece_para.point_serials.back()].x;
            float y2_temp = laser_cloud_laser_.points[line_piece_para.point_serials.back()].y;
            line_piece_para.para_A = y2_temp-y1_temp;
            line_piece_para.para_B = x1_temp-x2_temp;
            line_piece_para.para_C = line_piece_para.para_A*x1_temp+line_piece_para.para_B*y1_temp;
            line_piece_para.length = sqrtf((x2_temp-x1_temp)*(x2_temp-x1_temp)+(y2_temp-y1_temp)*(y2_temp-y1_temp));
        }
    }
}

int main(int argc, char** argv)
{
     ros::init(argc, argv, "laser_extractor");
     sensing_on_road::laser_extraction extractor;
     ros::spin();
     return 0;
}
