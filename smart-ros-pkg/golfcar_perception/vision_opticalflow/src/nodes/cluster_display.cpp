/**
 * Displays cluster. Subscribes to "cluster" and "image" topics.
*/
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>

#include <vision_opticalflow/Feature.h>
#include <vision_opticalflow/Cluster.h>
#include <vision_opticalflow/Clusters.h>

class ClusterDisplayNode
{
public:
    ClusterDisplayNode();
    void spin();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_;
    message_filters::Subscriber<vision_opticalflow::Clusters> clusters_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, vision_opticalflow::Clusters
    > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    std::string displayWindowName_;

    void displayCallback(const sensor_msgs::Image::ConstPtr & frame,
        const vision_opticalflow::Clusters::ConstPtr & clusters_msg);
};

ClusterDisplayNode::ClusterDisplayNode()
: it_(nh_),
  frame_sub_(it_, "image", 20),
  clusters_sub_(nh_, "clusters", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, clusters_sub_),
  displayWindowName_("")
{
    synchronizer.registerCallback( boost::bind(&ClusterDisplayNode::displayCallback, this, _1, _2) );
}

void ClusterDisplayNode::displayCallback(const sensor_msgs::Image::ConstPtr & frame,
        const vision_opticalflow::Clusters::ConstPtr & clusters_msg)
{
    cv::Point2f cluster_pos;
    cv::Point2f cluster_end;
    
    cv::Scalar color[] = {
        cv::Scalar(250,0,0), cv::Scalar(0,250,0), cv::Scalar(0,0,250), cv::Scalar(250,250,0), cv::Scalar(250,0,250), cv::Scalar(0,250,250),
        cv::Scalar(50,0,0), cv::Scalar(0,50,0), cv::Scalar(0,0,50), cv::Scalar(50,50,0), cv::Scalar(50,0,50), cv::Scalar(0,50,50),
        cv::Scalar(100,0,0), cv::Scalar(0,100,0), cv::Scalar(0,0,100), cv::Scalar(100,100,0), cv::Scalar(100,0,100), cv::Scalar(0,100,100),
        cv::Scalar(150,0,0), cv::Scalar(0,150,0), cv::Scalar(0,0,150), cv::Scalar(150,150,0), cv::Scalar(150,0,150), cv::Scalar(0,150,150),
        cv::Scalar(200,0,0), cv::Scalar(0,200,0), cv::Scalar(0,0,200), cv::Scalar(200,200,0), cv::Scalar(200,0,200), cv::Scalar(0,200,200)
    };
//     std::cout << " displayCallback " << std::endl;
    
    if( displayWindowName_.length()==0 ) {
        displayWindowName_ = clusters_sub_.getTopic();
        cv::namedWindow(displayWindowName_, CV_WINDOW_NORMAL);
    }
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    cv::Mat img = cvImgFrame->image;

    cv::rectangle(img,cv::Point(0,380),cv::Point(960,480),cv::Scalar(0,255,0));
    if(clusters_msg->clusters_info.size() > 0)
    {
        for( unsigned int i = 0; i < clusters_msg->clusters_info.size(); i++ )
        {
            cluster_pos.x = clusters_msg->clusters_info[i].centroid.x;
            cluster_pos.y = clusters_msg->clusters_info[i].centroid.y;
            cv::circle(img, cluster_pos, 4, cv::Scalar(0,0,250));
            //draw line
            cluster_end.x = clusters_msg->clusters_info[i].centroid.x + 10*((clusters_msg->clusters_info[i].centroid_speed_mag)*cos(clusters_msg->clusters_info[i].centroid_speed_dir));
            cluster_end.y = clusters_msg->clusters_info[i].centroid.y + 10*((clusters_msg->clusters_info[i].centroid_speed_mag)*sin(clusters_msg->clusters_info[i].centroid_speed_dir));
            cv::line(img, cluster_pos, cluster_end, cv::Scalar(0,0,250));

            for(unsigned int j = 0; j < clusters_msg->clusters_info[i].members.size(); j++)
            {
                cluster_pos.x = clusters_msg->clusters_info[i].members[j].x;
                cluster_pos.y = clusters_msg->clusters_info[i].members[j].y;
                cv::circle(img, cluster_pos, 2, color[clusters_msg->clusters_info[i].id%30]);
    //             cv::circle(img, cluster_pos, 2, color[i%30]);
            }
            for(unsigned int k = 0; k < clusters_msg->clusters_info[i].members.size()-1; k++)
            {
                cv::Point2f start_line,end_line;
                start_line.x = clusters_msg->clusters_info[i].members[k].x;
                start_line.y = clusters_msg->clusters_info[i].members[k].y;
                end_line.x = clusters_msg->clusters_info[i].members[k+1].x;
                end_line.y = clusters_msg->clusters_info[i].members[k+1].y;
                cv::line(img, start_line, end_line, color[clusters_msg->clusters_info[i].id%30]);
                cv::circle(img, cluster_pos, 2, color[clusters_msg->clusters_info[i].id%30]);
            }
        }
    }else{
        //what you can do?
    }
    cv::imshow(displayWindowName_, img);
}

void ClusterDisplayNode::spin()
{
    while( ros::ok() )
    {
        if( displayWindowName_.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(10)=='q' )
                return;
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_display", ros::init_options::AnonymousName);
    ClusterDisplayNode node;
    node.spin();
    return 0;
}