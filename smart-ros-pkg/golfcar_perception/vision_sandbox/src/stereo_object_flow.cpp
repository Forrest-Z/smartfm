#include "stereo_processor_base.h"
#include <opencv/cv.h>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv/highgui.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include "cv_bridge/cv_bridge.h"

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
		FlannBasedMatcher *matcher_;

		std::vector<KeyPoint> keypoints_0_, keypoints_1_;
		Mat descriptors_0_, descriptors_1_;
		sensor_msgs::PointCloud points3D_0_, points3D_1_;



		stereo_object_flow()
		{
			ros::NodeHandle private_nh("~");

			int nfeatures;
			private_nh.param("nfeatures", nfeatures, 100);
			private_nh.param("odom_frame", odom_frameID_, std::string("odom"));

			tf_ = new tf::TransformListener();
			feature_detector_ = new SiftFeatureDetector(100);
			extractor_ = new SiftDescriptorExtractor();
			matcher_   = new FlannBasedMatcher();
		}

		void imageCb(const ImageConstPtr& l_image_msg,
			  const CameraInfoConstPtr& l_info_msg,
			  const CameraInfoConstPtr& r_info_msg,
			  const DisparityImageConstPtr& disp_msg)
		{
			model_.fromCameraInfo(l_info_msg, r_info_msg);
			imageProc2D(l_image_msg, disp_msg);
		}

		void imageProc2D(const ImageConstPtr& l_image_msg, const DisparityImageConstPtr& disp_msg)
		{
			try
			{
				cv_image_ = cv_bridge::toCvCopy(l_image_msg, "bgr8");
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			Mat current_frame;
			cvtColor(cv_image_->image, current_frame, CV_BGR2GRAY);
			feature_detector_->detect( current_frame, keypoints_1_ );
			extractor_->compute( current_frame, keypoints_1_, descriptors_1_ );

			points3D_1_.header = l_image_msg->header;
			points3D_1_.points.clear();

			for(size_t i=0; i<keypoints_1_.size(); i++)
			{

			}

			if(keypoints_0_.size()==0)
			{

			}
			else
			{

			}

		}

		void keypoint2Dto3D(KeyPoint & feature2D, cv::Point3d & feature3D, const DisparityImageConstPtr& disp_msg)
		{
			cv::Point2d left_uv;
			left_uv.x = feature2D.pt.x;
			left_uv.y = feature2D.pt.y;
			double disparity_value;
			disparity_value = disp_msg->image.at<float>(left_uv.x,left_uv.y);
			model_.projectDisparityTo3d(left_uv, disparity_value, feature3D);
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

