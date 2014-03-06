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


int total_frame;
static gboolean
read_data (App * app)
{
    ros::spin();
    g_main_loop_quit (app->loop);
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
int epochSec = msg_ptr->header.stamp.toSec();
  
//   time_t raw_time = epochSec;
//   struct tm* timeinfo;
//   timeinfo = localtime(&raw_time);
//   string time_str = asctime(timeinfo);
//   stringstream ss;
//   ss<<time_str<<" "<<(msg_ptr->header.stamp.toSec() - epochSec)*1000;
//   cv::putText(cv_image->image, ss.str(), cv::Point(0, img_height_), 
// 	      cv::FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
  
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  size = img_width_*img_height_*3;

  buffer = gst_buffer_new_allocate (NULL, size, NULL);

  /* this makes the image black/white */
//   gst_buffer_memset (buffer, 0, 0xff , size);
  gst_buffer_fill(buffer, 0, &(msg_ptr->data[0]), size);
  g_signal_emit_by_name (s_app.appsrc, "push-buffer", buffer, &ret);
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
  ss<<"appsrc name=mysource ! video/x-raw, format=\"BGR\", ";
  ss<<"width="<<img_width_<<",height="<<img_height_<<",framerate="<<frame_rate<<"/1 ! videoconvert ! ";
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

  /* configure the caps of the video */
  g_object_set (G_OBJECT (app->appsrc), "caps",
      gst_caps_new_simple ("video/x-raw",
          "format", G_TYPE_STRING, "BGR",
          "width", G_TYPE_INT, img_width_,
          "height", G_TYPE_INT, img_height_,
          "framerate", GST_TYPE_FRACTION, frame_rate, 1, NULL), NULL);
  /* enable time stamp */
  gst_base_src_set_do_timestamp(GST_BASE_SRC(app->appsrc), true);
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