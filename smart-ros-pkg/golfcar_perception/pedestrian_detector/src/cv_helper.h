#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

namespace Cv_helper
{
    bool fillRoiRectangle(Size img_size, Size* roi_size, Point* roi_point, sensing_on_road::pedestrian_vision& pd)
    {
        
        //added for bayesian filter

        
        int image_width = img_size.width;
        int image_height = img_size.height;
        int img_x = pd.x;
        int img_y = pd.y;
        //pd.width = 640;
        //pd.height = 360;
        if(img_x > image_width || img_y > image_height) return false;
        if(img_x<0)img_x=0;
        if(img_y<0)img_y=0;

        int img_width = ((pd.width+img_x)>image_width)?(image_width-img_x):pd.width;
        int img_height =  ((pd.height+img_y)>image_height)?(image_height-img_y):pd.height;
        
        roi_size->width = img_width;
        roi_size->height = img_height;
        roi_point->x = img_x;
        roi_point->y = img_y;
        
        return true;
    };

    void sensormsgsToCv(const sensor_msgs::ImageConstPtr& msg_ptr, Mat &img)
    {
        ROS_DEBUG("SensorMsgs to CV");
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(msg_ptr, "bgra8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        Mat l_img(cv_image->image);
        img = l_img;
    };
};
