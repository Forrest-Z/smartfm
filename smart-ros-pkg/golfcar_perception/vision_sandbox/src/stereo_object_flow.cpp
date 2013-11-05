#include "stereo_processor_base.h"
#include <opencv/cv.h>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv/highgui.h>

using namespace cv;

namespace golfcart_vision
{
	class stereo_object_flow: public StereoProcessor
	{
		public:

		tf::TransformListener* tf_;
		std::string odom_frameID_, baselink_frameID_;
		cv_bridge::CvImagePtr cv_image_;

		SiftFeatureDetector *feature_detector_;
		SiftDescriptorExtractor *extractor_;
		std::vector<KeyPoint> keypoints_0_;
		Mat descriptors_0_;

		stereo_object_flow()
		{
			ros::NodeHandle private_nh("~");

			//feature extraction from 2D image;
			int nfeatures;
			private_nh.param("nfeatures", nfeatures, 100);

			tf_ = new tf::TransformListener();
			feature_detector_ = new SiftFeatureDetector(100);
			extractor_ = new SiftDescriptorExtractor();
		}

		void imageCb(const ImageConstPtr& l_image_msg,
			  const CameraInfoConstPtr& l_info_msg,
			  const CameraInfoConstPtr& r_info_msg,
			  const DisparityImageConstPtr& disp_msg)
		{
			model_.fromCameraInfo(l_info_msg, r_info_msg);
		}

		void imageProc2D(const ImageConstPtr& l_image_msg)
		{
			try
			{
				cv_image_ = cv_bridge::toCvCopy(l_image_msg, "bgr8");
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}

		}

		void keypoint2Dto3D(KeyPoint & feature2D, cv::Point3d & feature3D)
		{

		}

		void calcFlowVector()
		{

		}

	};

} // end of namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_object_flow_node");
  golfcart_vision::stereo_object_flow stereo_object_flow_node;
  ros::spin();
  return 0;
}

