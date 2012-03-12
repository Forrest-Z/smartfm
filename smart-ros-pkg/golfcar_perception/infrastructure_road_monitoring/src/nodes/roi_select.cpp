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

vector<cv::Point> g_contour;
cv::Mat g_frame;
std::string g_window_name;
PIT g_selected_point;


PIT find_closest_point(int x, int y)
{
	PIT closest = g_contour.end();
	float dmin = 10000;
	for( PIT it = g_contour.begin(); it!=g_contour.end(); ++it )
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

PIT find_closest_segment(int x, int y)
{
	PIT closest = g_contour.end();
	float dmin = 10000;
	for( PIT it = g_contour.begin(); it!=g_contour.end()-1; ++it )
	{
		PIT jt = it+1;
		float aa = fmutil::angle(it->x, it->y, jt->x, jt->y);
		float ab = fmutil::angle(it->x, it->y, x, y);
		float ad = fmutil::angDist(aa,ab);
		if( ad>M_PI_2 ) continue;

		float da = fmutil::distance(it->x, it->y, jt->x, jt->y);
		float db = fmutil::distance(it->x, it->y, x, y);
		if( db>da ) continue;

		float d = db * sin(ad);
		if( d<dmin )
		{
			dmin = d;
			closest = jt;
		}
	}

	return closest;
}

PIT add_to_countour(int x, int y)
{
	if( g_contour.size() < 2 ) {
		g_contour.push_back(cv::Point(x,y));
		return g_contour.end()-1;
	}

	//check if the new point is between 2 existing points
	PIT closest = find_closest_segment(x,y);
	if( closest == g_contour.end() )
	{
		g_contour.push_back(cv::Point(x,y));
		return g_contour.end()-1;
	}
	else
	{
		return g_contour.insert(closest, cv::Point(x,y));
	}

	return g_contour.begin(); //for syntax
}

string contour_to_string()
{
	stringstream ss;
	ss <<'[';
	for( PIT i = g_contour.begin(); i!=g_contour.end(); ++i )
	{
		if( i!=g_contour.begin() ) ss <<", ";
		ss <<'[' <<i->x <<',' <<i->y <<']';
	}
	ss <<']';
	return ss.str();
}

void draw_roi()
{
	std::vector< std::vector<cv::Point> > v;
	v.push_back( g_contour );
	cv::drawContours( g_frame, v, -1, CV_RGB(255,0,0), 2, 8 );
	for( unsigned i=0; i<g_contour.size(); i++ )
		cv::circle(g_frame, g_contour[i], 5, CV_RGB(255,0,0), 2);
    cv::imshow(g_window_name, g_frame);
}

int drag = 0;
void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == CV_EVENT_LBUTTONDOWN && !drag)
	{
		cout <<"button down" <<endl;
		PIT closest = find_closest_point(x,y);
		if( closest!=g_contour.end() )
		{
			cout <<"Selected point " <<closest->x <<"," <<closest->y <<endl;
			g_selected_point = closest;
			drag = 1;
		}
	}

	if (event == CV_EVENT_MOUSEMOVE && drag)
	{
		cout <<"dragging" <<endl;
		g_selected_point->x = x;
		g_selected_point->y = y;
	}

	if (event == CV_EVENT_LBUTTONUP && drag)
	{
		cout <<"button up" <<endl;
		drag = 0;
	}

	if( event == CV_EVENT_LBUTTONDBLCLK )
	{
		cout <<"double click" <<endl;
		PIT closest = find_closest_point(x,y);
		if( closest==g_contour.end() ) {
			cout <<"adding new point" <<endl;
			add_to_countour(x,y);
		}
		else {
			cout <<"erasing point" <<endl;
			g_contour.erase(closest);
		}
	}

	draw_roi();

	cout <<contour_to_string() <<endl;
}


class RoiSelectNode
{
public:
	RoiSelectNode();
    void spin();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber frame_sub_;
    void callback(const sensor_msgs::Image::ConstPtr & frame);
};


RoiSelectNode::RoiSelectNode()
: it_(nh_)
{
	frame_sub_ = it_.subscribe("image", 1, boost::bind(&RoiSelectNode::callback, this, _1));
}

void RoiSelectNode::callback(const sensor_msgs::Image::ConstPtr & frame)
{
    if( g_window_name.length()==0 ) {
    	g_window_name = frame_sub_.getTopic();
        cv::namedWindow(g_window_name, CV_WINDOW_NORMAL);
        cv::setMouseCallback(g_window_name, mouse_callback, NULL);
    }
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    g_frame = cvImgFrame->image;

    draw_roi();
}

void RoiSelectNode::spin()
{
    while( ros::ok() )
    {
        if( g_window_name.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(100)=='q' )
                return;
        ros::spinOnce();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_display", ros::init_options::AnonymousName);
    RoiSelectNode node;
    node.spin();
    return 0;
}
