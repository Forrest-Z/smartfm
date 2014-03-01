#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
GST_DEBUG_CATEGORY (appsrc_pipeline_debug);
#define GST_CAT_DEFAULT appsrc_pipeline_debug

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

typedef struct _App App;

struct _App
{
  GstElement *pipeline;
  GstElement *appsrc;

  GMainLoop *loop;
  guint sourceid;

  GTimer *timer;

};

cv::Mat image_;

App s_app;
GstBus *bus;
static IplImage* rgbImage_;
int img_width_, img_height_;
int frame_rate, bitrate;

cv::Mat sensorMsgsToCv(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  ROS_DEBUG("SensorMsgs to CV");
  cv_bridge::CvImagePtr cv_image;
  try {
      cv_image = cv_bridge::toCvCopy(msg_ptr, "rgb8");
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  int epochSec = msg_ptr->header.stamp.toSec();
  
  time_t raw_time = epochSec;
  struct tm* timeinfo;
  timeinfo = localtime(&raw_time);
  string time_str = asctime(timeinfo);
  stringstream ss;
  ss<<time_str<<" "<<(msg_ptr->header.stamp.toSec() - epochSec)*1000;
  
  cv::putText(cv_image->image, ss.str(), cv::Point(0, img_height_), 
	      cv::FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
  return cv_image->image;
};

int total_frame;
static gboolean
read_data (App * app)
{
  
    if(ros::ok())
    ros::spinOnce();
  else {
    gst_element_set_state (app->pipeline, GST_STATE_NULL);
    gst_object_unref (bus);
    g_main_loop_unref (app->loop);
    exit(0);
  }
  
    GstFlowReturn ret;
    gdouble ms;

    ms = g_timer_elapsed(app->timer, NULL);
    if (ms > 1.0/double(frame_rate)) {
        GstBuffer *buffer;
        gboolean ok = TRUE;
	  
	if(image_.data == NULL) return TRUE;
	
// 	cout<<total_frame<<endl;
// 	total_frame++;
// 	if(total_frame > 1000){
// 	  gst_element_set_state (app->pipeline, GST_STATE_NULL);
// 	  gst_object_unref (bus);
// 	  g_main_loop_unref (app->loop);
// 	  exit(0);
// 	}
	buffer = gst_buffer_new_and_alloc(img_width_*img_height_*3*sizeof(guchar));
	memcpy(GST_BUFFER_DATA(buffer),rgbImage_->imageData, GST_BUFFER_SIZE(buffer));
        GST_BUFFER_SIZE (buffer) = img_height_*img_width_*3*sizeof(guchar);
// 	cvShowImage("img", rgbImage_);
// 	cvWaitKey(1);
	
        GST_DEBUG ("feed buffer");
        g_signal_emit_by_name (app->appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref (buffer);

        if (ret != GST_FLOW_OK) {
            /* some error, stop sending data */
            GST_DEBUG ("some error");
            ok = FALSE;
        }

        g_timer_start(app->timer);

        return ok;
    }

    // g_signal_emit_by_name (app->appsrc, "end-of-stream", &ret);
    return TRUE;
}

/* This signal callback is called when appsrc needs data, we add an idle handler
* to the mainloop to start pushing data into the appsrc */
static void
start_feed (GstElement * pipeline, guint size, App * app)
{
  if (app->sourceid == 0) {
    GST_DEBUG ("start feeding");
    app->sourceid = g_idle_add ((GSourceFunc) read_data, app);
  }
}

/* This callback is called when appsrc has enough data and we can stop sending.
* We remove the idle handler from the mainloop */
static void
stop_feed (GstElement * pipeline, App * app)
{
  if (app->sourceid != 0) {
    GST_DEBUG ("stop feeding");
    g_source_remove (app->sourceid);
    app->sourceid = 0;
  }
}

static gboolean
bus_message (GstBus * bus, GstMessage * message, App * app)
{
  GST_DEBUG ("got message %s",
      gst_message_type_get_name (GST_MESSAGE_TYPE (message)));

  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_ERROR: {
        GError *err = NULL;
        gchar *dbg_info = NULL;

        gst_message_parse_error (message, &err, &dbg_info);
        g_printerr ("ERROR from element %s: %s\n",
            GST_OBJECT_NAME (message->src), err->message);
        g_printerr ("Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
        g_error_free (err);
        g_free (dbg_info);
        g_main_loop_quit (app->loop);
        break;
    }
    case GST_MESSAGE_EOS:
      g_main_loop_quit (app->loop);
      break;
    default:
      break;
  }
  return TRUE;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr){
//   std::cout<<"Image callback!"<<std::endl;
  image_ = sensorMsgsToCv(msg_ptr);
  rgbImage_ =  new IplImage(image_);
}

int
main (int argc, char *argv[])
{
  App *app = &s_app;
  GstCaps *caps;
  total_frame = 0;
  gst_init (&argc, &argv);
  ros::init(argc, argv, "GstAdapter");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("minoru_sync/image_raw", 1, &imageCallback);
  GST_DEBUG_CATEGORY_INIT (appsrc_pipeline_debug, "appsrc-pipeline", 0,
      "appsrc pipeline example");

  /* create a mainloop to get messages and to handle the idle handler that will
* feed data to appsrc. */
  app->loop = g_main_loop_new (NULL, TRUE);
  app->timer = g_timer_new();

  // Option 1: Display on screen via xvimagesink
  //app->pipeline = gst_parse_launch("appsrc name=mysource ! video/x-raw-rgb,width=640,height=240 ! ffmpegcolorspace ! videoscale method=1 ! xvimagesink", NULL);

  // Option 2: Encode using Theora and stream through UDP
  // NOTE: first launch receiver by executing:
  //       gst-launch udpsrc port=5000 ! theoradec ! ffmpegcolorspace ! xvimagesink
  //app->pipeline = gst_parse_launch("appsrc name=mysource ! videorate ! ffmpegcolorspace ! videoscale method=1 ! video/x-raw-yuv,width=640,height=240,framerate=\(fraction\)15/1 ! theoraenc bitrate=700 ! udpsink host=127.0.0.1 port=5000", NULL);

  ros::NodeHandle priv_n("~");
  string receiver_address;
  int port;
  priv_n.param("receiver_address", receiver_address, string("127.0.0.1"));
  priv_n.param("port", port, 1234);
  priv_n.param("img_width", img_width_, 640);
  priv_n.param("img_height", img_height_, 240);
  priv_n.param("frame_rate", frame_rate, 15);
  priv_n.param("bitrate", bitrate, 400);
  stringstream ss;
  ss<<"appsrc name=mysource ! videorate ! ffmpegcolorspace ! videoscale method=1 ! video/x-raw-yuv,";
  ss<<"width="<<img_width_<<",height="<<img_height_<<",framerate="<<frame_rate<<"/1 ! ";
  ss<<"x264enc bitrate="<<bitrate<<" pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! ";
  ss<<"udpsink host="<<receiver_address<<" port="<<port;
  cout<<"Configuration: "<<ss.str()<<endl;
  // Option 3: Encode using H.264 and stream through RTP
  app->pipeline = gst_parse_launch(ss.str().c_str(), NULL);
  g_assert (app->pipeline);

  bus = gst_pipeline_get_bus (GST_PIPELINE (app->pipeline));
  g_assert(bus);

  /* add watch for messages */
  gst_bus_add_watch (bus, (GstBusFunc) bus_message, app);

  /* get the appsrc */
    app->appsrc = gst_bin_get_by_name (GST_BIN(app->pipeline), "mysource");
    g_assert(app->appsrc);
    g_assert(GST_IS_APP_SRC(app->appsrc));
    g_signal_connect (app->appsrc, "need-data", G_CALLBACK (start_feed), app);
    g_signal_connect (app->appsrc, "enough-data", G_CALLBACK (stop_feed), app);

  /* set the caps on the source */
  caps = gst_video_format_new_caps(GST_VIDEO_FORMAT_RGB, img_width_, img_height_, 0, 1, 4, 3);
  gst_app_src_set_caps(GST_APP_SRC(app->appsrc), caps);

  /* enable time stamp */
  gst_base_src_set_do_timestamp(GST_BASE_SRC(app->appsrc), false);
  /* go to playing and wait in a mainloop. */
  gst_element_set_state (app->pipeline, GST_STATE_PLAYING);

  /* this mainloop is stopped when we receive an error or EOS */
  g_main_loop_run (app->loop);
  GST_DEBUG ("stopping");

  gst_element_set_state (app->pipeline, GST_STATE_NULL);

  gst_object_unref (bus);
  g_main_loop_unref (app->loop);

  return 0;
}