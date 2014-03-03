#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

using namespace std;

int img_width_, img_height_;

GstElement *appsrc;
GMainLoop *loop;

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr){
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  size = img_width_*img_height_*3;

  buffer = gst_buffer_new_allocate (NULL, size, NULL);
  gst_buffer_fill(buffer, 0, &(msg_ptr->data[0]), size);
  g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);
}

static gboolean read_data(){
  ros::spin();
  g_main_loop_quit (loop);
  return TRUE;
}

static void start_feed(GstElement * pipeline, guint size){
  g_idle_add ((GSourceFunc) read_data, NULL);
}

int main(int argc, char** argv){
  gst_init(&argc, &argv);
  ros::init(argc, argv, "gst_opencv_streamer");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("minoru_sync/image_raw", 1, &imageCallback);
  ros::NodeHandle priv_n("~");
  string receiver_address;
  int port, frame_rate, bitrate;
  priv_n.param("receiver_address", receiver_address, string("127.0.0.1"));
  priv_n.param("port", port, 1234);
  priv_n.param("img_width", img_width_, 640);
  priv_n.param("img_height", img_height_, 360);
  priv_n.param("frame_rate", frame_rate, 30);
  priv_n.param("bitrate", bitrate, 400);
  stringstream ss;
  ss<<"appsrc name=testsource ! video/x-raw, format=\"BGR\", ";
  ss<<"width="<<img_width_<<",height="<<img_height_<<",framerate="<<frame_rate<<"/1 ! videoconvert ! ";
  ss<<"x264enc bitrate="<<bitrate<<" tune=zerolatency ! rtph264pay ! ";
  ss<<"udpsink host="<<receiver_address<<" port="<<port;
  
  loop = g_main_loop_new(NULL, FALSE);
  GstElement *sink = gst_parse_launch(ss.str().c_str(), NULL);
  g_assert (sink);
  
  appsrc = gst_bin_get_by_name (GST_BIN(sink), "testsource");
  g_assert(appsrc);
  g_assert(GST_IS_APP_SRC(appsrc));
  g_signal_connect (appsrc, "need-data", G_CALLBACK (start_feed), NULL);
  g_object_set (G_OBJECT (appsrc), "caps",
      gst_caps_new_simple ("video/x-raw",
          "format", G_TYPE_STRING, "BGR",
          "width", G_TYPE_INT, img_width_,
          "height", G_TYPE_INT, img_height_,
          "framerate", GST_TYPE_FRACTION, frame_rate, 1, NULL), NULL);
  /* enable time stamp */
  gst_base_src_set_do_timestamp(GST_BASE_SRC(appsrc), true);
  g_object_set (G_OBJECT (appsrc), "format", GST_FORMAT_TIME, NULL);
  gst_element_set_state (sink, GST_STATE_PLAYING);
  printf ("Let's run!\n");
  g_main_loop_run (loop);
  return 0;
}