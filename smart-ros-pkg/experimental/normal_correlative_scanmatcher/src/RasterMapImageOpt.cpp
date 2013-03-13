/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */


#include <pcl/visualization/pcl_visualizer.h>
#include "NormalCorrelativeMatching.h"
#include "mysql_helper.h"
#include "readfrontend.h"
pcl::PointCloud<pcl::PointNormal> input_cloud, matching_cloud;
double res_;
double offset_x, offset_y, rotation;
pcl::PointCloud<pcl::PointNormal> best_tf_pt;
NormalCorrelativeMatching *ncm;

bool matching_mode=true;
vector<ScoreData> score_data_vec;
string frontend_file;
boost::shared_ptr<pcl::visualization::PCLVisualizer> initVisualizer() {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return (viewer);
}

void addPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
        pcl::PointCloud<pcl::PointNormal> &cloud_in, string cloud_name,
        string normal_name, bool red_color) {
    cout << cloud_in.points.size() << " loaded." << endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> *color;
    if (red_color)
        color = new pcl::visualization::PointCloudColorHandlerCustom<
                pcl::PointNormal>(cloud_in.makeShared(), 255, 0, 0);
    else
        color = new pcl::visualization::PointCloudColorHandlerCustom<
                pcl::PointNormal>(cloud_in.makeShared(), 0, 255, 0);
    viewer->addPointCloud<pcl::PointNormal>(cloud_in.makeShared(), *color,
            cloud_name);
    viewer->addPointCloudNormals<pcl::PointNormal>(cloud_in.makeShared(), 2,
            0.1, normal_name);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}

int score_sql_idx = 0;
string folder;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
            *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
    if (event.keyDown()) {
        if(matching_mode)
        {
          cout<<"Matching mode"<<endl;
          viewer->removePointCloud("matching_cloud");
          viewer->removePointCloud("matching_normal_cloud");
          string keySym = event.getKeySym();

          if (keySym == "Down")
              offset_y -= res_;
          else if (keySym == "Up")
              offset_y += res_;
          else if (keySym == "Left")
              offset_x -= res_;
          else if (keySym == "Right")
              offset_x += res_;
          else if (keySym == "y")
              rotation++;
          else if (keySym == "p")
              rotation--;
          vector<cv::Point2f> search_pt;
          vector<double> normal_pt;
          pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
          double yaw_rotate = rotation / 180. * M_PI;
          Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
                  -sin(yaw_rotate / 2.));
          Eigen::Vector3f bl_trans(offset_x, offset_y, 0.);
          pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(
                  matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
          search_pt.resize(matching_cloud_tf.points.size());
          normal_pt.resize(matching_cloud_tf.points.size());
          //cout<<"Size of keyboard cloud: "<<matching_cloud_tf.points.size()<<endl;
          for (size_t i = 0; i < matching_cloud_tf.points.size(); i++) {
              search_pt[i].x = matching_cloud_tf.points[i].x;
              search_pt[i].y = matching_cloud_tf.points[i].y;
              normal_pt[i] = atan2(matching_cloud_tf.points[i].normal_y,
                      matching_cloud_tf.points[i].normal_x);
          }
          ScoreDetails sd;
          assert(ncm->rmi.image_.data != NULL);
          double best_score = ncm->rmi.getScoreWithNormal(search_pt, normal_pt, sd);

          cout << "Distance:" << sd.dist_score << " Norm:" << sd.normal_score
                  << " NNorm:" << sd.norm_norm_score << " WorstNorm:"
                  << sd.worst_norm_score << " Final:" << best_score << endl;

          addPointCloud(viewer, matching_cloud_tf, "matching_cloud",
                  "matching_normal_cloud", false);
          cout << offset_x << " " << offset_y << " " << rotation << endl;
      }
      else
      {
        cout<<"Continuous mode"<<endl;
        viewer->removePointCloud("input_cloud");
        viewer->removePointCloud("input_normal_cloud");
        viewer->removePointCloud("matching_cloud");
        viewer->removePointCloud("matching_normal_cloud");

        ScoreData sd;
        //do
        {
          sd = score_data_vec[score_sql_idx++];
        }//while (sd.node_src < 200 || (sd.node_src > 5420 && sd.node_src<5700));
          
        
        stringstream input_file;
        input_file <<folder<<setfill('0')<<setw(5)<<sd.node_src<<".pcd";    
        pcl::PointCloud<pcl::PointNormal> input_cloud;
        pcl::io::loadPCDFile(input_file.str(), input_cloud);
        vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, frontend_file.c_str(), input_file.str());

        stringstream matching_file;
        matching_file <<folder<<setfill('0')<<setw(5)<<sd.node_dst<<".pcd";
        pcl::PointCloud<pcl::PointNormal> matching_cloud;
        pcl::io::loadPCDFile(matching_file.str(), matching_cloud);
        cout<<input_file.str()<<endl;
        cout<<matching_file.str()<<endl;
        double yaw_rotate = sd.t / 180. * M_PI;
        Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
                  -sin(yaw_rotate / 2.));
        Eigen::Vector3f bl_trans(sd.x, sd.y, 0.);
        pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(
                  matching_cloud, matching_cloud, bl_trans, bl_rotation);
        addPointCloud(viewer, input_cloud, "input_cloud",
                  "input_normal_cloud", true);
        addPointCloud(viewer, matching_cloud, "matching_cloud",
                  "matching_normal_cloud", false);
        
      }

    }
}



map<int, int> getHistogram(pcl::PointCloud<pcl::PointNormal> &pcl) {
    map<int, int> histogram;
    for (size_t i = 0; i < pcl.points.size(); i++) {
        histogram[int(
                atan2(pcl.points[i].normal_y, pcl.points[i].normal_x) / M_PI
                        * 360)]++;
    }
    for (int i = -360; i <= 360; i++) {
        if (histogram.find(i) == histogram.end())
            histogram[i] = 0;
    }
    return histogram;

}

double getHistogramSSE(map<int, int> &hist1, map<int, int> &hist2) {
    assert(hist1.size() == hist2.size());
    double error = 0;
    for (map<int, int>::iterator it1 = hist1.begin(), it2 = hist2.begin();
            it1 != hist1.end(); it1++, it2++) {
        double temp = it1->second - it2->second;
        error += temp * temp;
    }
    return error;
}

void getBestRotationWithHistogram() {
    map<int, int> input_hist = getHistogram(input_cloud);
    vector<map<int, int> > histograms;
    double smallest_error = 1e999;
    int best_rotation = -360;
    for (int i = -180; i < 180; i += 2) {
        pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
        double yaw_rotate = i / 180. * M_PI;
        Eigen::Vector3f bl_trans(0, 0, 0.);
        Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
                -sin(yaw_rotate / 2.));
        pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(
                matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
        map<int, int> matching_hist = getHistogram(matching_cloud_tf);
        double error_now = getHistogramSSE(matching_hist, input_hist);
        cout << error_now << " ";
        if (error_now < smallest_error) {
            smallest_error = error_now;
            best_rotation = i;
        }
    }
    cout << endl;
    cout << "Best rotation found: " << best_rotation << " with error: "
            << smallest_error << endl;
}

int main(int argc, char **argv) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
            initVisualizer();
    if(argc == 5)
    {
      if(string(argv[1]) == "m")
      {
        cout<<"Matching mode on"<<endl; matching_mode = true;
        pcl::io::loadPCDFile(argv[3], input_cloud);
        append_input_cloud(input_cloud, string(argv[2]), string(argv[3]));
        pcl::io::loadPCDFile(argv[4], matching_cloud);
        res_ = 0.1;
        input_cloud = pcl_downsample(input_cloud, res_*2, res_*2, res_*2);
        
        matching_cloud = pcl_downsample(matching_cloud, res_*2, res_*2, res_*2);
        addPointCloud(viewer, input_cloud, "input_cloud", "input_normal_cloud",
                true);

        ncm = new NormalCorrelativeMatching(input_cloud, res_, 0.2);
        fmutil::Stopwatch sw("single matching not sorted");
        ncm->bruteForceSearch(matching_cloud);
        sw.end();
        fmutil::Stopwatch sw2("single matching sorted");
        ncm->bruteForceSearch(matching_cloud, true);
        sw2.end();
        best_tf_pt = ncm->best_tf_pt;
        rotation = ncm->rotation;
        offset_x = ncm->offset_x;
        offset_y = ncm->offset_y;
        
        cout<<"Best score = "<<ncm->best_score_<<" "<<ncm->offset_x<<" "<<ncm->offset_y<<" "<<ncm->rotation<<endl;
        addPointCloud(viewer, best_tf_pt, "matching_cloud",
                    "matching_normal_cloud", false);
      }
      else if(string(argv[1]) == "d")
      {
        cout<<"Assuming frontend data is given, reading from database"<<endl; 
        matching_mode = false;
        frontend_file = argv[2];
        int startfile_idx= frontend_file.find_last_of("/")+1;
        folder = frontend_file.substr(0, startfile_idx);
        string frontend = frontend_file.substr(startfile_idx, frontend_file.size()-startfile_idx-4);
        cout<<"Folder = "<<folder<<" frontend = "<<frontend<<endl;
        MySQLHelper mysql("normal_scanmatch", frontend);     
        score_data_vec = mysql.getListScoreConstraint(atof(argv[3]), atof(argv[4]), 10); 
      }
    }
    else
    {
      cout<<"Expected commands are:"<<endl;
      cout<<"$ RasterMapImageOpt m frontEndFile srcPCD matchingPCD"<<endl;
      cout<<"$ RasterMapImageOpt d frontEndFile scoreThresL scoreThresU"<<endl;
      return 1;
    } 
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
