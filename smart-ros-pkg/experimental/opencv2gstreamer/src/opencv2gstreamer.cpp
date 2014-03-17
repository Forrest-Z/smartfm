#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";


static GMainLoop * loop = NULL;
static GstElement * pipeline2 = NULL;

static IplImage * img;
static uchar * IMG_data;

static const char app_sink_name[] = "app-sink_01" ; //no sink, cv_bridge will take care!
static const char app_src_name[] = "app-src_01";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    img = new IplImage(cv_ptr->image);
    IMG_data = (uchar *) img->imageData;
    cvConvertImage(img,img,CV_CVTIMG_SWAP_RB);
    
    //TODO:how to decide the buffer size
    GstBuffer *buffer;
    memcpy(GST_BUFFER_DATA(buffer),IMG_data, GST_BUFFER_SIZE(buffer));
    
    //push
    gst_app_src_push_buffer(GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline2),app_src_name)), buffer);
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
	gchar *userdata = (gchar *) data;

	switch(GST_MESSAGE_TYPE(msg))
	{
		case GST_MESSAGE_EOS:
		{
			
			break;
		}

		case GST_MESSAGE_ERROR:
		{
			gchar *debug;
			GError *error;
			
			gst_message_parse_error(msg, &error, &debug);
			g_free(debug);

			g_printerr("Error in pipeline:%s\nError Message: %s\n", userdata, error->message);
			g_error_free(error);

			g_main_loop_quit(loop);
			break;
		}

		case GST_MESSAGE_STATE_CHANGED : 
		{
        	GstState oldstate;
        	GstState newstate;
        	GstState pending;

        	gst_message_parse_state_changed (msg,&oldstate,&newstate,&pending);

        	g_debug("pipeline:%s old:%s new:%s pending:%s", userdata,
												gst_element_state_get_name(oldstate),
        	                   	                gst_element_state_get_name(newstate),
        	                      	            gst_element_state_get_name(pending));
    	 	break;
		}

		case GST_MESSAGE_WARNING: 
		{
			gchar *debug;
			GError *error;

        	gst_message_parse_warning (msg,&error,&debug);
			g_warning("pipeline:%s",userdata);
        	g_warning("debug: %s", debug);
	        g_warning("error: %s", error->message);
    	    g_free (debug);
    	    g_error_free (error);
    		break;
		}

		default:
			break;
	}
	return TRUE;
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "image_converter");
  
  GError *error = NULL;
  GstBus *bus = NULL;
  GstAppSinkCallbacks callbacks;
    gchar pipeline1_str[256];
    gchar pipeline2_str[256];	
    
    g_print("Inicializando GStreamer.\n");
    gst_init(&argc, &argv);
    
    g_print("Creating Main Loop.\n");
    loop = g_main_loop_new(NULL,FALSE);
    
    int res = 0;
    //TODO:res = sprintf(pipeline2_str, "appsrc name=\"%s\" ! queue ! videoparse format=14 width=%d height=%d framerate=25/1 ! videorate ! videoscale ! ffmpegcolorspace ! video/x-raw-rgb, width=%d, height=%d ! xvidenc ! avimux ! filesink location=\"%s\"", app_src_name, IMG_width, IMG_height, IMG_width, IMG_height, output_filename); 
    if (res < 0)
    {
	    g_printerr("Error configuring pipeline2's string \n");
	    return -1;
    }
    
    
    //debugging
    g_print("%s\n",pipeline2_str);
    //creating pipeline2
    pipeline2 = gst_parse_launch(pipeline2_str, &error);

    if (error) 
    {
	    g_printerr("Error [%s]\n",error->message);
	    return -1;
    }
    
    if (!gst_bin_get_by_name( GST_BIN(pipeline2), app_src_name))
    {
	    g_printerr("error creating app-src\n");
	    return -1;
    }
    
  // Adding msg handler to Pipeline1
      g_print("Adding msg handler to %s\n",gst_element_get_name(pipeline2));
      bus = gst_pipeline_get_bus( GST_PIPELINE(pipeline2) );
      gst_bus_add_watch(bus, bus_call, gst_element_get_name(pipeline2));
      gst_object_unref(bus);
    
    //Set the pipeline2 to "playing" state 
      g_print("Setting pipeline2's state to \"playing\".\n");
      gst_element_set_state(pipeline2, GST_STATE_PLAYING);
      
      // Iterate
      g_print("Running...\n");
      g_main_loop_run(loop);
      
      g_print("Stopping playback - pipeline2\n");
      gst_element_set_state(pipeline2, GST_STATE_NULL);
      
      g_print("Deleting pipeline2.\n");
      gst_object_unref( GST_OBJECT(pipeline2) );
      g_main_loop_unref(loop);
      
  ImageConverter ic;
  ros::spin();
  return 0;
}
