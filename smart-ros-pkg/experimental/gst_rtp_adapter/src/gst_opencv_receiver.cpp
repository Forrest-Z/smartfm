#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

using namespace std;

int img_width_, img_height_;

GstElement *appsink;
GMainLoop *loop;
GstElement *source;

image_transport::Publisher *pub_;

static GstFlowReturn on_new_sample_from_sink (GstElement * elt){

  if(!ros::ok())
    g_main_loop_quit (loop);
  
  GstSample *sample;
  GstBuffer  *buffer;
  int width, height;
  sample = gst_app_sink_pull_sample (GST_APP_SINK (elt));
  buffer = gst_sample_get_buffer (sample);
  GstPad* pad = gst_element_get_static_pad(appsink, "sink");
  const GstCaps *caps = gst_pad_get_current_caps(pad);
  GstStructure *structure = gst_caps_get_structure(caps,0);
//   cout<<gst_structure_to_string(structure)<<endl;
  gst_structure_get_int(structure,"width",&width);
  gst_structure_get_int(structure,"height",&height);
  
  sensor_msgs::Image msg;
  msg.width = width; 
  msg.height = height;
  msg.encoding = "rgb8";
  msg.is_bigendian = false;
  msg.step = width*3;
  msg.data.resize(width*height*3);
  GstMapInfo info;
  gst_buffer_map(buffer, &info, GST_MAP_READ);
  std::copy(info.data, info.data+(width*height*3), msg.data.begin());
  pub_->publish(msg);
  gst_buffer_unmap (buffer, &info);
  gst_sample_unref (sample);
  ros::spinOnce();

  return GST_FLOW_OK;
}

static gboolean
on_source_message (GstBus * bus, GstMessage * message)
{

  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_EOS:
      g_print ("The source got dry\n");
      gst_app_src_end_of_stream (GST_APP_SRC (source));
      break;
    case GST_MESSAGE_ERROR:
      g_print ("Received error\n");
      GError *gerror;
      gchar *debug;
      gst_message_parse_error(message, &gerror, &debug);
      cout<<debug<<endl;
      g_main_loop_quit (loop);
      break;
    default:
      break;
  }
  return TRUE;
}

int main(int argc, char** argv){
  gst_init(&argc, &argv);
  ros::init(argc, argv, "gst_opencv_receiver");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("gst/image_raw", 1);
  pub_ = &pub;
  stringstream ss;
  ss<<"udpsrc port=1234 ! application/x-rtp, payload=127 ! rtph264depay ! avdec_h264 ! ";
  ss<<"videoconvert ! videoscale ! appsink name=testsink caps=\"video/x-raw, format=RGB\"";
  cout<<ss.str()<<endl;
  loop = g_main_loop_new(NULL, FALSE);
  source = gst_parse_launch(ss.str().c_str(), NULL);
  g_assert (source);
  
  GstBus *bus = gst_element_get_bus (source);
  gst_bus_add_watch (bus, (GstBusFunc) on_source_message, NULL);
  gst_object_unref (bus);
  
  appsink = gst_bin_get_by_name (GST_BIN(source), "testsink");
  g_assert(appsink);
  
  g_object_set(G_OBJECT(appsink), "emit-signals", TRUE, "sync", FALSE, NULL);
  g_signal_connect (appsink, "new-sample", G_CALLBACK (on_new_sample_from_sink), NULL);
  gst_object_unref (appsink);
  
  gst_element_set_state (source, GST_STATE_PLAYING);
  printf ("Let's receive!\n");
  g_main_loop_run (loop);
  return 0;
}