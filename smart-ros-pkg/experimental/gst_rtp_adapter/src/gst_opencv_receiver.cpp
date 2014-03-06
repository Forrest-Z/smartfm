#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include "addTimeStamp.h"

using namespace std;

int img_width_, img_height_;

GstElement *appsink;
GMainLoop *loop;
GstElement *source;

image_transport::Publisher *pub_;
ros::Publisher *delay_pub_;
static GstFlowReturn on_new_sample_from_sink (GstElement * elt){

  GstClockTime gst_time_now = gst_clock_get_time (gst_element_get_clock (source));
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
//   cout<<GST_BUFFER_TIMESTAMP (buffer)<<" "<<GST_BUFFER_TIMESTAMP_IS_VALID(buffer)<<endl;
  GstClockTime running_time = GST_BUFFER_TIMESTAMP(buffer);
  GstClockTime buffer_timestamp = gst_element_get_base_time(source)+running_time;
  double delay_ms = double(gst_time_now-buffer_timestamp)/1e6;
//   cout<<buffer->offset<<": Running time "<<running_time/1e9<<" Delay "<<delay_ms<<" ms"<<endl;
//   cout<<GST_BUFFER_DURATION (buffer)<<endl;
//   cout<<"DTS:"<<GST_BUFFER_DTS(buffer)<<" PTS:"<<GST_BUFFER_PTS(buffer)<<endl;
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
  cv_bridge::CvImagePtr cv_image = sensorMsgToMat(msg);
  cv::Mat final_img = cv_image->image;
  
  ros::Time time_now = ros::Time::now();
  //put time stamp both for text and QR
  cv::Mat time_img = addTimeStamp(time_now);
  cv::Mat roi = final_img(cv::Rect(msg.width-time_img.cols, 0, time_img.cols, time_img.rows));
  time_img.copyTo(roi);
  cv::Mat qr_img = getQRCode(time_now);
  roi = final_img(cv::Rect(msg.width-qr_img.cols, msg.height-qr_img.rows, qr_img.cols, qr_img.rows));
  qr_img.copyTo(roi);
  
  
  //get latency
  cv::Mat received_qr_roi = final_img(cv::Rect(0, msg.height-qr_img.rows, qr_img.cols, qr_img.rows));
  uint64_t capture_time_ns = retriveQrCode(received_qr_roi);
  if(capture_time_ns > 0){
    int64_t latency = time_now.toNSec() - capture_time_ns;
    std_msgs::Float64 delay_msg;
    delay_msg.data = (double)latency/1e6;
    delay_pub_->publish(delay_msg);
  }
  cv_bridge::CvImage cv_img_msg(msg.header, msg.encoding, final_img);
  pub_->publish(cv_img_msg.toImageMsg());
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
  ros::NodeHandle priv_nh("~");
  int port;
  priv_nh.param("port", port, 1234);
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("gst/image_raw", 1);
  pub_ = &pub;
  delay_pub_ = new ros::Publisher(n.advertise<std_msgs::Float64>("delay_ms", 1));
  stringstream ss;
  ss<<"udpsrc port="<<port<<" ! application/x-rtp, payload=127 ! rtpjitterbuffer latency=100 ! rtph264depay ! avdec_h264 ! ";
  ss<<"videoconvert ! videoscale ! appsink name=testsink caps=\"video/x-raw, format=BGR\"";
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
