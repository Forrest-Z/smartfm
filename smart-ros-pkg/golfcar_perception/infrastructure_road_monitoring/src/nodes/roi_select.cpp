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

using namespace std;

typedef vector<cv::Point>::iterator PIT;

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

    vector<cv::Point> contour_;
    cv::Mat frame_;
    std::string window_name_;
    PIT selected_point_;
    bool drag_;
    bool contour_changed_;

    void img_callback(const sensor_msgs::Image::ConstPtr&);
    void timer_callback(const ros::TimerEvent&);

    PIT find_closest_point(int x, int y);
    float dist_to_segment(int x, int y, const PIT & it, const PIT & jt);
    PIT add_to_countour(int x, int y);
    string contour_to_str() const;

    void init();
    void draw_roi();
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
        for( unsigned i=0; i<contour_.size(); i++ )
        {
            fmutil::bound<int>(0, &(contour_[i].x), frame->width);
            fmutil::bound<int>(0, &(contour_[i].y), frame->height);
        }
    }

    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    frame_ = cvImgFrame->image;

    draw_roi();
}

string RoiSelectNode::contour_to_str() const
{
    stringstream ss;
    ss << '[';
    for( unsigned i=0; i<contour_.size(); i++ )
    {
        if( i!=0 ) ss <<", ";
        ss <<'[' <<contour_[i].x <<',' <<contour_[i].y <<']';
    }
    ss <<']';
    return ss.str();
}

void RoiSelectNode::timer_callback(const ros::TimerEvent & e)
{
    if( contour_changed_ )
    {
        // convert the countour into an XmlRpcValue
        XmlRpc::XmlRpcValue my_list;
        my_list.setSize(contour_.size());
        for( unsigned i=0; i<contour_.size(); i++ ) {
            my_list[i].setSize(2);
            my_list[i][0] = contour_[i].x;
            my_list[i][1] = contour_[i].y;
        }

        // send to parameter server
        ROS_INFO_STREAM("Setting poly " <<poly_param_ <<" to " <<contour_to_str());
        ros::param::set(poly_param_, my_list);
        contour_changed_ = false;
    }
}

void RoiSelectNode::init()
{
	// retrieve any existing definition to use as starting polygon
	ROS_INFO_STREAM("Checking for initial polygon definition in " << ros::names::resolve(poly_param_));
	XmlRpc::XmlRpcValue my_list;
	if( ros::param::get(poly_param_, my_list) )
	{
		ROS_INFO_STREAM("Got " << my_list);
		ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < my_list.size(); ++i)
		{
		  ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
		  ROS_ASSERT(my_list[i].size()==2);
		  contour_.push_back( cv::Point(static_cast<int>(my_list[i][0]),
				  static_cast<int>(my_list[i][1])));
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
		PIT closest = find_closest_point(x,y);
		if( closest!=contour_.end() )
		{
			//cout <<"Selected point " <<closest->x <<"," <<closest->y <<endl;
			selected_point_ = closest;
			drag_ = 1;
		}
	}

	if (event == CV_EVENT_MOUSEMOVE && drag_)
	{
		//cout <<"dragging" <<endl;
		selected_point_->x = x;
		selected_point_->y = y;
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
		PIT closest = find_closest_point(x,y);
		if( closest==contour_.end() ) {
			//cout <<"adding new point" <<endl;
			add_to_countour(x,y);
		    contour_changed_ = true;
		}
		else {
			//cout <<"erasing point" <<endl;
			contour_.erase(closest);
			contour_changed_ = true;
		}
	}

	draw_roi();
}

PIT RoiSelectNode::find_closest_point(int x, int y)
{
	PIT closest = contour_.end();
	float dmin = 10000;
	for( PIT it = contour_.begin(); it!=contour_.end(); ++it )
	{
		float d = fmutil::distance(it->x, it->y, x, y);
		if( d<dmin && d<30 )
		{
			closest = it;
			dmin = d;
		}
	}
	return closest;
}


/** Check all lines of the polygon for the one closest to (x,y)
 *
 * Returns an iterator to the point
*/
float RoiSelectNode::dist_to_segment(int x, int y, const PIT & it, const PIT & jt)
{
	float aa = fmutil::angle(it->x, it->y, jt->x, jt->y);
	float ab = fmutil::angle(it->x, it->y, x, y);
	float ad = fmutil::angDist(aa,ab);
	if( ad>M_PI_2 ) return -1;

	float da = fmutil::distance(it->x, it->y, jt->x, jt->y);
	float db = fmutil::distance(it->x, it->y, x, y);
	if( db>da ) return -1;

	return db * sin(ad);
}

PIT RoiSelectNode::add_to_countour(int x, int y)
{
	PIT closest = contour_.end();
	float dmin = 10000;
	for( PIT it = contour_.begin(); it!=contour_.end(); ++it )
	{
		PIT jt;
		if( it!=contour_.end()-1 )
			jt = it+1;
		else
			jt = contour_.begin();

		float d = dist_to_segment(x,y,it,jt);
		if( d>=0 && d<dmin )
		{
			dmin = d;
			closest = jt;
		}
	}

	if( closest==contour_.begin() ) closest = contour_.end();
	return contour_.insert(closest, cv::Point(x,y));
}


void RoiSelectNode::draw_roi()
{
	vector< vector<cv::Point> > v;
	v.push_back( contour_ );
	cv::drawContours( frame_, v, -1, CV_RGB(255,0,0), 2, 8 );
	for( unsigned i=0; i<contour_.size(); i++ )
		cv::circle(frame_, contour_[i], 5, CV_RGB(255,0,0), 2);
    cv::imshow(window_name_, frame_);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "roi_select", ros::init_options::AnonymousName);
    RoiSelectNode node;
    node.spin();
    return 0;
}
