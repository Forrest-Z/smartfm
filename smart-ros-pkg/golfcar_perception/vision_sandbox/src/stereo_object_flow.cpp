#include <opencv/cv.h>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv/highgui.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseArray.h"

#include "stereo_processor_base.h"

#define INVALID_Z_VALUE 1000.0

using namespace cv;

namespace golfcart_vision
{
	class stereo_object_flow: public StereoProcessor
	{
		public:

		tf::TransformListener* tf_;
		std::string odom_frame_id_, baselink_frame_id_;
		cv_bridge::CvImagePtr cv_image_;

		SiftFeatureDetector *feature_detector_;
		SiftDescriptorExtractor *extractor_;
		FlannBasedMatcher *matcher_;

		std::vector<KeyPoint> keypoints_0_, keypoints_1_;
		Mat descriptors_0_, descriptors_1_;
		sensor_msgs::PointCloud points3D_0_, points3D_1_;

		ros::Publisher objectflow_pub_;
		ros::NodeHandle nh_;

		stereo_object_flow()
		{
			ros::NodeHandle private_nh("~");

			int nfeatures;
			private_nh.param("nfeatures", nfeatures, 100);
			private_nh.param("odom_frame", odom_frame_id_, std::string("odom"));

			tf_ = new tf::TransformListener();
			feature_detector_ = new SiftFeatureDetector(100);
			extractor_ = new SiftDescriptorExtractor();
			matcher_   = new FlannBasedMatcher();

			objectflow_pub_ = nh_.advertise<geometry_msgs::PoseArray>("objectflow", 2);
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

			cv_bridge::CvImagePtr disparity_mat;
			//pay attention here;
			disparity_mat = cv_bridge::toCvCopy(disp_msg->image, disp_msg->image.encoding);

			for(size_t i=0; i<keypoints_1_.size(); i++)
			{
				geometry_msgs::Point32 point_tmp = keypoint2Dto3D(keypoints_1_[i], disparity_mat->image, disp_msg->min_disparity);
				if(point_tmp.z < DBL_MAX - 1.0) points3D_1_.points.push_back(point_tmp);
			}

			try
			{
				tf_->transformPointCloud(odom_frame_id_, points3D_1_, points3D_1_);
			}
			catch (tf::TransformException)
			{
				ROS_WARN("cannot transform into Odom frame");
				return;
			}


			if(keypoints_0_.size()!=0)
			{
				std::vector< DMatch > matches;
				matcher_->match( descriptors_1_, descriptors_0_, matches );

				double max_dist = 0; double min_dist = 100;
				for( int i = 0; i < descriptors_1_.rows; i++ )
				{
					double dist = matches[i].distance;
					if( dist < min_dist ) min_dist = dist;
					if( dist > max_dist ) max_dist = dist;
				}

				std::vector< DMatch > good_matches;
				geometry_msgs::PoseArray object_flow;
				object_flow.header = disp_msg->header;
				object_flow.header.frame_id = odom_frame_id_;

				for( int i = 0; i < descriptors_1_.rows; i++ )
				{
					if( matches[i].distance <= 10*min_dist )
					{
						good_matches.push_back( matches[i]);
						int current_index = matches[i].queryIdx;
						int ref_index = matches[i].trainIdx;
						object_flow.poses.push_back(calcFlowVector(current_index, ref_index));
					}
				}

				objectflow_pub_.publish(object_flow);
			}

			keypoints_0_ = keypoints_1_;
			descriptors_0_ = descriptors_1_;
			points3D_0_ = points3D_1_;
		}

		geometry_msgs::Point32 keypoint2Dto3D(KeyPoint & feature2D, Mat& disp_mat, float disp_min)
		{
			cv::Point3d feature3D;
			cv::Point2d left_uv;
			left_uv.x = feature2D.pt.x;
			left_uv.y = feature2D.pt.y;
			double disparity_value;


			disparity_value = disp_mat.at<float>(left_uv.x,left_uv.y);
			if(disparity_value <= disp_min){feature3D.x=0.0; feature3D.y=0.0; feature3D.z = DBL_MAX;}
			else model_.projectDisparityTo3d(left_uv, disparity_value, feature3D);

			geometry_msgs::Point32 point_tmp;
			point_tmp.x = (float)feature3D.x;
			point_tmp.y = (float)feature3D.y;
			point_tmp.z = (float)feature3D.z;
			return point_tmp;
		}

		geometry_msgs::Pose calcFlowVector(int current_index, int ref_index)
		{
			geometry_msgs::Pose object_flow;
			object_flow.position.z = INVALID_Z_VALUE + 0.001;
			if(points3D_1_.points[current_index].z > INVALID_Z_VALUE || points3D_0_.points[current_index].z > INVALID_Z_VALUE) return object_flow;

			double delt_x = points3D_1_.points[current_index].x - points3D_0_.points[current_index].x;
			double delt_y = points3D_1_.points[current_index].y - points3D_0_.points[current_index].y;
			//double delt_z = points3D_1_.points[current_index].z - points3D_0_.points[current_index].z;
			double yaw_tmp = atan2(delt_y, delt_x);

            tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(yaw_tmp),
                            btVector3(points3D_1_.points[current_index].x,points3D_1_.points[current_index].y, points3D_1_.points[current_index].z)),
                            object_flow);
            return object_flow;
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

