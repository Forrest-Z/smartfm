#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include "RasterMapImageOpt.h"
#include "renderMap.h"
#include <fstream>
#include <boost/thread/thread.hpp>
#include "pcl_downsample.h"
#include "mysql_helper.h"
class NormalCorrelativeMatching{
public:
  pcl::PointCloud<pcl::PointNormal> input_cloud;//, matching_cloud;
  double res_;
  double offset_x, offset_y, rotation;
  double best_score_;
  
  pcl::PointCloud<pcl::PointNormal> best_tf_pt;
  RasterMapImage rmi, rmi_ver;
  double range_cov_;
  NormalCorrelativeMatching(pcl::PointCloud<pcl::PointNormal> input_cl, double res, double range_cov): 
    input_cloud(input_cl), res_(res), rmi(res_, range_cov), rmi_ver(res_, range_cov), range_cov_(range_cov) {
    vector<cv::Point2f> input_pts, input_normals;
    input_pts.resize(input_cloud.points.size());
    input_normals.resize(input_cloud.points.size());
    for (size_t i = 0; i < input_cloud.points.size(); i++) {
        input_pts[i].x = input_cloud.points[i].x;
        input_pts[i].y = input_cloud.points[i].y;
        input_normals[i].x = input_cloud.points[i].normal_x;
        input_normals[i].y = input_cloud.points[i].normal_y;
    }
    rmi.getInputPoints(input_pts, input_normals);
    
  }
  
  double veriScore(pcl::PointCloud<pcl::PointNormal> &matching_clouds_j, vector<pcl::PointCloud<pcl::PointNormal> > &input_clouds, ScoreData &sd) {
    RasterMapImage rmi_ver(res_, range_cov_);
    pcl::PointCloud<pcl::PointNormal> matching_cloud = matching_clouds_j;
    double yaw_rotate = sd.t / 180. * M_PI;
    Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
	      -sin(yaw_rotate / 2.));
    Eigen::Vector3f bl_trans(sd.x, sd.y, 0.);
    pcl::transformPointCloudWithNormals<pcl::PointNormal>(
	      matching_cloud, matching_cloud, bl_trans, bl_rotation);
    vector<cv::Point2f> input_pts, input_normals;
    input_pts.resize(matching_cloud.points.size());
    input_normals.resize(matching_cloud.points.size());
    for (size_t k = 0; k < matching_cloud.points.size(); k++) {
	input_pts[k].x = matching_cloud.points[k].x;
	input_pts[k].y = matching_cloud.points[k].y;
	input_normals[k].x = matching_cloud.points[k].normal_x;
	input_normals[k].y = matching_cloud.points[k].normal_y;
    }
    rmi_ver.getInputPoints(input_pts, input_normals);  
    double ver_score = 0;

    for(size_t k=0; k<input_clouds.size(); k++)
    {
      pcl::PointCloud<pcl::PointNormal> input_cloud_single = input_clouds[k];
      input_cloud_single = pcl_downsample(input_cloud_single, res_, res_, res_);
      vector<cv::Point2f> search_pt;
      vector<double> normal_pt;
      search_pt.resize(input_cloud_single.points.size());
      normal_pt.resize(input_cloud_single.points.size());
      
      for (size_t l = 0; l < input_cloud_single.points.size(); l++) {
	  search_pt[l].x = input_cloud_single.points[l].x;
	  search_pt[l].y = input_cloud_single.points[l].y;
	  normal_pt[l] = atan2(input_cloud_single.points[l].normal_y,
		  input_cloud_single.points[l].normal_x);
      }
      ScoreDetails s_det;
      double best_score = rmi_ver.getScoreWithNormal(search_pt, normal_pt, s_det);
      if(ver_score < best_score)
	ver_score = best_score;
    }
    return ver_score;
  }



  void bruteForceSearch(ScoreDetails &sd, pcl::PointCloud<pcl::PointNormal> &matching_cloud, bool sorted=false) {
    double best_x, best_y;
    int best_rotation;
    double best_score = 0;
    ScoreDetails best_score_details;
    fmutil::Stopwatch sw("overall");
    fmutil::Stopwatch sw_detail, sw_tf, sw_normal;
    int rot_start = sd.best_rot-sd.rot_range;
    int rot_end = sd.best_rot+sd.rot_range;
    double trans_x_start = sd.best_x - sd.trans_x_range;
    double trans_x_end = sd.best_x + sd.trans_x_range;
    double trans_y_start = sd.best_y - sd.trans_y_range;
    double trans_y_end = sd.best_y + sd.trans_y_range;
    vector<cv::Point> best_tf_pt_int;
    pcl::PointCloud<pcl::PointNormal> best_norm_tf;
    //cout<<rot_start<<" "<<rot_end<<" "<<trans_x_start<<" "<<trans_x_end<<" "<<trans_y_start<<" "<<trans_y_end<<endl;
    int angle_inc = sd.angle_res;

    for (int i = rot_start; i < rot_end; i += angle_inc) {
      //cout<<"rotation "<<i<<":"<<endl;
        pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
        int rotate_idx = i;
        double yaw_rotate = rotate_idx / 180. * M_PI;
        Eigen::Vector3f bl_trans(0, 0, 0.);
        Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
                -sin(yaw_rotate / 2.));
        sw_tf.start("tf");
        pcl::transformPointCloudWithNormals<pcl::PointNormal>(
                matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
        sw_tf.end(false);
        //matching_cloud_tf = pcl_downsample(matching_cloud_tf, 0.5, 0.5, 0.5);
        vector<double> angular_normals;
        angular_normals.resize(matching_cloud_tf.size());
        //cout<<"Size of cloud= "<<angular_normals.size()<<endl;
        sw_normal.start("2");
        for (size_t idx = 0; idx < matching_cloud_tf.size(); idx++) {
            angular_normals[idx] = atan2(matching_cloud_tf.points[idx].normal_y,
                    matching_cloud_tf.points[idx].normal_x);
        }
        sw_normal.end(false);
        vector<cv::Point> search_pt_int;
        search_pt_int.resize(matching_cloud_tf.size());

        for (size_t j = 0; j < matching_cloud_tf.size(); j++) {
            search_pt_int[j] = rmi.imageCoordinate(matching_cloud_tf.points[j]);
        }
        for (double j = trans_x_start; j < trans_x_end; j += sd.trans_res)	//=res_)
                {
            vector<cv::Point> search_pt;
            search_pt.resize(search_pt_int.size());
            int offset_x = j / res_;
            for (size_t idx = 0; idx < search_pt_int.size(); idx++)
                search_pt[idx].x = search_pt_int[idx].x + offset_x;
            for (double k = trans_y_start; k < trans_y_end; k += sd.trans_res)	//res_)
                    {
                sw_detail.start("getscore time");
                int offset_y = k / res_;
                for (size_t idx = 0; idx < search_pt_int.size(); idx++)
                    search_pt[idx].y = search_pt_int[idx].y - offset_y;
                ScoreDetails sd;

                double score;
              if(sorted)score = rmi.getScoreWithNormalSorted(search_pt,
                        angular_normals, sd);
              else score = rmi.getScoreWithNormal(search_pt,
                        angular_normals, sd);
                
                sw_detail.end(false);
                //cout<<i<<" "<<j<<" "<<k<<" "<<score<<endl;
                if (score > best_score) {
                    best_x = j;
                    best_y = k;
                    best_rotation = rotate_idx;
                    best_score = score;
                    best_score_details = sd;
                    best_tf_pt_int = search_pt;
                    best_norm_tf = matching_cloud_tf;
                }
                //cout<<score<<" ";
            }
            //cout<<endl;
            //cout<<setiosflags (ios::fixed | ios::showpoint | ios::right) <<setprecision(3)<<setfill('0')<<setw(3)<<"At offset "<<j<<" "<<" "<<i<<", best tf found so far: "<<best_x<<" "<<best_y<<" "<<best_rotation<<
            	//" Distance:"<<best_score_details.dist_score<<" Norm:"<< best_score_details.normal_score<<" NNorm:"<<best_score_details.norm_norm_score<<
            //" WorstNorm:"<<best_score_details.worst_norm_score<<" Final:"<<best_score<<"      \xd"<<flush;
        }

    }
    sw.end(false);
    /*cout << "getscore: " << sw_detail.total_ / 1000 << " ms" << endl;
    cout << "tf: " << sw_tf.total_ / 1000 << " ms" << endl;
    cout << "getscore inner: " << rmi.scorepoint_sw_.total_ / 1000 << " ms"
            << endl;
    cout << "norm rotate: " << sw_normal.total_ / 1000 << " ms" << endl;
    cout << best_x <<" "<<best_y << " "<< best_rotation<<" "<<best_score<<endl;
    */
    offset_x = best_x;
    offset_y = best_y;
    rotation = best_rotation;
    sd.best_score = best_score;
    sd.best_rot =  best_rotation;
    sd.best_x = best_x;
    sd.best_y = best_y;
    /*best_tf_pt = best_norm_tf;
    assert(best_tf_pt.size() == best_tf_pt_int.size());
    for(size_t i=0; i<best_tf_pt_int.size(); i++)
    {
      cv::Point2f pt = rmi.realCoordinate(best_tf_pt_int[i]);
      best_tf_pt.points[i].x = pt.x;
      best_tf_pt.points[i].y = pt.y;
    }*/
  }
  double bruteForceSearch(pcl::PointCloud<pcl::PointNormal> &matching_cl, bool sorted = false){
    ScoreDetails sd;
    double score = bruteForceSearch(matching_cl, sd, sorted);
    return score;
  }
  double bruteForceSearch(pcl::PointCloud<pcl::PointNormal> &matching_cl, ScoreDetails& sd, bool sorted = false){
    
    ScoreDetails sd1, sd1b, sd2;
    sd1.best_rot = 0;
    sd1.best_x = 0;
    sd1.best_y = 0;
    sd1.rot_range = 180;
    sd1.angle_res = 10;
    sd1.trans_res = 1;
    sd1.trans_x_range = 20;
    sd1.trans_y_range = 20;
    //sd1b = sd1;
    //sd1b.best_rot = 0;
    //sd1b.rot_range = 12;    
    bruteForceSearch(sd1, matching_cl, sorted);
    //bruteForceSearch(sd1b, matching_cl, sorted);
    //if(sd1b.best_score > sd1.best_score) sd2 = sd1b;
    //else 
      sd2= sd1;
    /*sd2.rot_range = 5; sd2.angle_res = 1; sd2.trans_res = 0.1;
  //performing matching 100 and 1224 doesn't yield a good match, a 0.1 x direction mismatch 
  //this is due to rounding problem where the fast search rounded the value first then perform translation, resulting a misalignment where direct pcl tf only perform rounding at the very end right before getScore. Improved it by increasing the base resolution to 0.05
    sd2.trans_x_range = 1.0; sd2.trans_y_range = 1.0;
    //just to simulate the same work load with GPU
    //for(int i=0; i<10; i++)
      bruteForceSearch(sd2, matching_cl, sorted);*/
    best_score_ = sd2.best_score;
    sd = sd2;
    return best_score_;
  }
};
