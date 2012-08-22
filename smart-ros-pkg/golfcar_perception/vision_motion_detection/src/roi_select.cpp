/**
 * Display video and allow tuning the ROI (polygon)
 */


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <fmutil/fm_math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vision_motion_detection/Polygon.h>

using namespace std;



class RoiSelectNode
{
public:
    RoiSelectNode();
    void spin();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber frame_sub_;
    ros::Timer timer_;
    string poly_param_; // the name of the parameter to set

    Polygon roi_;
    cv::Mat frame_;
    std::string window_name_;
    Polygon::iterator selected_point_;
    bool drag_;
    bool contour_changed_;

    void img_callback(const sensor_msgs::Image::ConstPtr&);
    void timer_callback(const ros::TimerEvent&);

    Polygon::iterator find_closest_point(int x, int y);
    float dist_to_segment(int x, int y, const Polygon::iterator & it, const Polygon::iterator & jt);
    Polygon::iterator add_to_countour(int x, int y);

    void init();
    void on_mouse(int event, int x, int y, int flags);
    static void mouse_callback(int event, int x, int y, int flags, void *obj);
};


RoiSelectNode::RoiSelectNode()
: it_(nh_), drag_(false), contour_changed_(false)
{
    // retrieve the name of the parameter where we can read and set the poly definition
    ROS_ASSERT(ros::param::get("~poly", poly_param_));

    frame_sub_ = it_.subscribe("image", 1, boost::bind(&RoiSelectNode::img_callback, this, _1));
}

void RoiSelectNode::img_callback(const sensor_msgs::Image::ConstPtr & frame)
{
    if( window_name_.length()==0 )
    {
        init();
        for( unsigned i=0; i<roi_.size(); i++ )
        {
            fmutil::bound<int>(0, &(roi_[i].x), frame->width);
            fmutil::bound<int>(0, &(roi_[i].y), frame->height);
        }
    }

    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    frame_ = cvImgFrame->image;

    roi_.draw(frame_, CV_RGB(255,0,0));
    cv::imshow(window_name_, frame_);
}



void RoiSelectNode::timer_callback(const ros::TimerEvent & e)
{
    if( contour_changed_ )
    {
        // convert the contour into an XmlRpcValue
        XmlRpc::XmlRpcValue xmlrpc = roi_.to_XmlRpc();

        // send to parameter server
        ROS_INFO_STREAM("Setting poly " <<poly_param_ <<" to " <<roi_.to_str());
        ros::param::set(poly_param_, xmlrpc);
        contour_changed_ = false;
    }
}

void RoiSelectNode::init()
{
    // retrieve any existing definition to use as starting polygon
    ROS_INFO_STREAM("Checking for initial polygon definition in " << ros::names::resolve(poly_param_));
    XmlRpc::XmlRpcValue xmlrpc;
    if( ros::param::get(poly_param_, xmlrpc) )
    {
        ROS_INFO_STREAM("Got " << xmlrpc);
        try
        {
            roi_.from_XmlRpc(xmlrpc);
        }
        catch (runtime_error & e)
        {
            ROS_ERROR("Initial polygon definition: wrong format");
        }
    }
    else
    {
        ROS_INFO("No initial polygon definition found.");
    }

    window_name_ = frame_sub_.getTopic();
    cv::namedWindow(window_name_, CV_WINDOW_NORMAL);
    cv::setMouseCallback(window_name_, RoiSelectNode::mouse_callback, this);

    timer_ = nh_.createTimer(ros::Duration(0.1),
            boost::bind(&RoiSelectNode::timer_callback, this, _1));
}

void RoiSelectNode::spin()
{
    while( ros::ok() )
    {
        if( window_name_.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(100)=='q' )
                return;
        ros::spinOnce();
    }
}

void RoiSelectNode::mouse_callback(int event, int x, int y, int flags, void *obj)
{
	( (RoiSelectNode *) obj ) -> on_mouse(event, x, y, flags);
}

void RoiSelectNode::on_mouse(int event, int x, int y, int flags)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag_)
    {
        //cout <<"button down" <<endl;
        Polygon::iterator closest = roi_.find_closest_point(x,y);
        if( closest!=roi_.end() )
        {
            //cout <<"Selected point " <<closest->x <<"," <<closest->y <<endl;
            selected_point_ = closest;
            drag_ = 1;
        }
    }

    if (event == CV_EVENT_MOUSEMOVE && drag_)
    {
        //cout <<"dragging" <<endl;
        selected_point_->x = fmutil::bound<int>(0, x, frame_.size().width);
        selected_point_->y = fmutil::bound<int>(0, y, frame_.size().height);
    }

    if (event == CV_EVENT_LBUTTONUP && drag_)
    {
        //cout <<"button up" <<endl;
        drag_ = 0;
        contour_changed_ = true;
    }

    if( event == CV_EVENT_LBUTTONDBLCLK )
    {
        //cout <<"double click" <<endl;
        Polygon::iterator closest = roi_.find_closest_point(x,y);
        if( closest==roi_.end() )
        {
            //cout <<"adding new point" <<endl;
            roi_.add_to_countour(x,y);
            contour_changed_ = true;
        }
        else
        {
            //cout <<"erasing point" <<endl;
            roi_.erase(closest);
            contour_changed_ = true;
        }
    }

    roi_.draw(frame_, CV_RGB(255,0,0));
    cv::imshow(window_name_, frame_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roi_select", ros::init_options::AnonymousName);
    RoiSelectNode node;
    node.spin();
    return 0;
}
