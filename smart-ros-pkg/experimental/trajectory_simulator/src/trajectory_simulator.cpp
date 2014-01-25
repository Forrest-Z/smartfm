#include <stage.hh>
#include <golfcar_ppc/golfcar_purepursuit.h>
#include <nav_msgs/Path.h>
#include <sys/stat.h>
#include <StationPath.h>
#include <fmutil/fm_stopwatch.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
using namespace std;

struct poseVec{
    double x, y, a;
    poseVec(): x(0.0), y(0.0), a(0.0){}
    poseVec(double _x, double _y, double _a):
    x(_x), y(_y), a(_a){}
};

class CarModel{
public:
  
  double wheelbase_;
  
  double time_step_;
  double time_now_;
  poseVec vel_, pose_;
  CarModel(double wheelbase, double time_step): wheelbase_(wheelbase), 
  time_step_(time_step), time_now_(0.0){}
  CarModel(double wheelbase, double time_step, poseVec init_pose):
  wheelbase_(wheelbase), time_step_(time_step), time_now_(0.0), pose_(init_pose){}
    
  void SetSpeed(double x, double y, double a){
    
    vel_.x = x * cos(a);
    vel_.a = x * sin(a)/wheelbase_;
    
    
    double dx = vel_.x * time_step_;
    pose_.a = normalize(pose_.a + vel_.a * time_step_);
    double cosa = cos(pose_.a);
    double sina = sin(pose_.a);
    pose_.x += dx * cosa;
    pose_.y += dx * sina;
    time_now_+=time_step_;
  }
  
private:
  
  inline double normalize( double a )
  {
    while( a < -M_PI ) a += 2.0*M_PI;
    while( a >  M_PI ) a -= 2.0*M_PI;	 
    return a;
  }
  
};

class TrajectorySimulator{
  golfcar_purepursuit::PurePursuit *pp_;
  StationPaths sp_;
  string global_frame_;
  
  void setGlobalPath(ros::Publisher &pub){
    StationPath station_path = sp_.getPath(sp_.knownStations()(0),
					    sp_.knownStations()(1));
    nav_msgs::Path p;
    p.poses.resize(station_path.size());
    for(unsigned int i=0; i<station_path.size(); i++)
    {
	p.poses[i].header.frame_id = global_frame_;
	p.poses[i].header.stamp = ros::Time::now();
	p.poses[i].pose.position.x = station_path[i].x_;
	p.poses[i].pose.position.y = station_path[i].y_;
	p.poses[i].pose.orientation.w = 1.0;
    }
    ROS_INFO("Plan with %d points sent.", (int)p.poses.size());
    p.header.stamp = ros::Time();
    p.header.frame_id = global_frame_;
    pub.publish(p);
    
    pp_-> path_.poses = p.poses;
    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;
  }
public:
  TrajectorySimulator(int argc, char** argv){
    ros::init(argc, argv, "trajectory_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    double min_look_ahead_dist = 4.0;
    double forward_achor_pt_dist = 1.0;
    double car_length = 2.55;
    double time_step, max_lat_acc, max_speed, max_acc, max_jerk;
    priv_nh.param("time_step", time_step, 0.1);
    priv_nh.param("max_lat_acc", max_lat_acc, 1.0);
    priv_nh.param("max_speed", max_speed, 5.0);
    priv_nh.param("max_acc", max_acc, 0.5);
    priv_nh.param("max_jerk", max_jerk, 0.5);
    global_frame_ = "/robot_0/map";
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    
    sensor_msgs::PointCloud pc;
    ros::Publisher pub, global_path_pub, curvature_pub, max_speed_pub, speed_pub, acc_pub, jerk_pub;
    
    pub = nh.advertise<nav_msgs::Path>("simulated_path", 1, true);
    global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 1, true);
    curvature_pub = nh.advertise<sensor_msgs::PointCloud>("curvature_pt", 1, true);
    max_speed_pub = nh.advertise<sensor_msgs::PointCloud>("max_speed_pt", 1, true);
    speed_pub = nh.advertise<sensor_msgs::PointCloud>("speed_pt", 1, true);
    acc_pub = nh.advertise<sensor_msgs::PointCloud>("acc_pt", 1, true);
    jerk_pub = nh.advertise<sensor_msgs::PointCloud>("jerk_pt", 1, true);
    
    setGlobalPath(global_path_pub);
    nav_msgs::Path simulated_path;
    
    geometry_msgs::Point first_pt = pp_->path_.poses[0].pose.position;
    geometry_msgs::Point sec_pt = pp_->path_.poses[1].pose.position;
    double car_init_orientation = atan2(sec_pt.y-first_pt.y, sec_pt.x-first_pt.x);
    CarModel model(car_length, time_step, poseVec(pp_-> path_.poses[0].pose.position.x, 
						  pp_-> path_.poses[0].pose.position.y, car_init_orientation));
    
    
    bool path_exist = true;
    fmutil::Stopwatch sw("Simulation took");
    sensor_msgs::PointCloud curve_pc, max_speed_pc, speed_pc, acc_pc, jerk_pc;
    double speed_now = 0, acc_now = 0;
    while(path_exist && ros::ok()){
      geometry_msgs::PoseStamped p;
      p.header.frame_id =global_frame_;
      p.header.stamp = ros::Time::now();
      p.pose.position.x = model.pose_.x;
      p.pose.position.y = model.pose_.y;
      p.pose.orientation = tf::createQuaternionMsgFromYaw(model.pose_.a);
      simulated_path.poses.push_back(p);
      pp_->vehicle_base_ = p.pose;
      double steer_angle, dist_to_goal;
      path_exist = pp_->steering_control(&steer_angle, &dist_to_goal);
      //using low speed to obtain an over fitted curve for a tighter turn
      model.SetSpeed(3.0, 0.0, steer_angle);
      
      geometry_msgs::Point32 curve_pt, max_speed_pt, speed_pt, acc_pt, jerk_pt;
      curve_pt.x = model.pose_.x;
      curve_pt.y = model.pose_.y;
      max_speed_pt = speed_pt = acc_pt = jerk_pt = curve_pt;
      max_speed_pt.y+=100.0;
      speed_pt.y+=200.0;
      acc_pt.y+=300.0;
      jerk_pt.y+=400.0;
      double turning_rad = fabs(car_length / tan(steer_angle));
      curve_pt.z = turning_rad;
      max_speed_pt.z = sqrt(max_lat_acc*turning_rad);
      if(max_speed_pt.z > max_speed) max_speed_pt.z = max_speed;


      acc_pt.z = (max_speed_pt.z - speed_now)/time_step;
      double required_jerk = (acc_pt.z - acc_now)/time_step;
      if( required_jerk > max_jerk)	
	required_jerk = max_jerk;
      else if(required_jerk < -max_jerk)
	required_jerk = -max_jerk;
      acc_now += required_jerk * time_step;
      jerk_pt.z = required_jerk;
      speed_now += acc_now * time_step;
      speed_pt.z = speed_now;
      curve_pc.points.push_back(curve_pt);
      max_speed_pc.points.push_back(max_speed_pt);
      speed_pc.points.push_back(speed_pt);
      acc_pc.points.push_back(acc_pt);
      jerk_pc.points.push_back(jerk_pt);
      pp_->updateCommandedSpeed(5.0);
    }
    sw.end();
    simulated_path.header.frame_id = global_frame_;
    simulated_path.header.stamp = ros::Time::now();
    max_speed_pc.header = speed_pc.header = acc_pc.header = jerk_pc.header = curve_pc.header = simulated_path.header;
    curvature_pub.publish(curve_pc);
    max_speed_pub.publish(max_speed_pc);
    speed_pub.publish(speed_pc);
    acc_pub.publish(acc_pc);
    jerk_pub.publish(jerk_pc);
    pub.publish(simulated_path);
    
    
    
      
    ros::spin();
      
  }
  
  
};

int main(int argc, char** argv){
  TrajectorySimulator ts(argc, argv);
  return 0;
}