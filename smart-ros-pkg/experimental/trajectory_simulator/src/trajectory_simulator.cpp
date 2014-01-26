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
    //we want to maintain a fixed distance, not time fixed
    
    
    vel_.x = x * cos(a);
    vel_.a = x * sin(a)/wheelbase_;
    
    //figure out the time step for a fixed amount of distance
    //let's do a 5 cm resolution.
    double dx = 0.05;
    time_step_ = dx/vel_.x;
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
  
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

  double solveCubic(double a, double b, double c, double d){
    //http://www.1728.org/cubic2.htm
    if(fabs(a)<1e-9){
      cout<<fabs(a)<<endl;
      a=b;
      b=c;
      c=d;
      double temp = b*b - 4*a*c;
      cout <<a<<" "<<b<<" "<<c<<" "<< temp<<endl;
      return (-b + sqrt(temp))/(2*a);
    }
    double f = ((3*c/a)-((b*b)/(a*a)))/3;
    double g = ((2*pow(b,3)/pow(a,3))-(9*b*c/pow(a,2))+(27*d/a))/27.0;
    double h = pow(g,2)/4.0+pow(f,3)/27.0;
    cout<<"h = "<<h<<endl;
    if(h > 0){
      double r = -g/2.0+sqrt(h);
      double s = pow(r, 1/3.0);
      double t = -g/2.0-sqrt(h);
      double u = sgn(t) * pow(fabs(t),1/3.0);
      return s+u-(b/(3*a));
    }
    else {
      double i = sqrt((pow(g,2)/4)-h);
      double j = pow(i,(1/3.0));
      double k = acos(-(g/(2*i)));
      double l = -j;
      double m = cos(k/3.0);
      double n = sqrt(3.0)*sin(k/3.0);
      double p = -(b/(3*a));
      double x1= 2*j*cos(k/3.0)-(b/(3*a));
      double x2= l*(m+n)+p;
      double x3= l*(m-n)+p;
      double minx = 999.0;
      bool match_criteria = false;
      cout<<x1<<" "<<x2<<" "<<x3<<endl;
      if(x1 < minx && x1>=0) {minx = x1; match_criteria=true;}
      if(x2 < minx && x2>=0) {minx = x2; match_criteria=true;}
      if(x3 < minx && x3>=0) {minx = x3; match_criteria=true;}
      assert(match_criteria);
      return minx;
    }
    
    
  }
  
  TrajectorySimulator(int argc, char** argv){
    double a=atof(argv[1]);
    double b=atof(argv[2]);
    double c=atof(argv[3]);
    double d=atof(argv[4]);
    cout<<solveCubic(a,b,c,d)<<endl;
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
    vector<int> speed_intervals;
    double speed_now = 3.0, acc_now = 0;
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
      
      model.SetSpeed(speed_now, 0.0, steer_angle);
      pp_->updateCommandedSpeed(speed_now);
      
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
      
      curve_pc.points.push_back(curve_pt);
      max_speed_pc.points.push_back(max_speed_pt);
      speed_pc.points.push_back(speed_pt);
      acc_pc.points.push_back(acc_pt);
      jerk_pc.points.push_back(jerk_pt);
    }
    for(int i=0; i<1; i++){
      int min_speed_idx=0;
      double min_speed=max_speed_pc.points[0].z;
      for(size_t j=1; j<max_speed_pc.points.size(); j++){
	double speed_temp = max_speed_pc.points[j].z ;
	if(speed_temp< min_speed){
	  min_speed = speed_temp;
	  min_speed_idx=j;
	}
      }
      
      double v1 = 0 + 0.5 * max_acc*max_acc/max_jerk;
      double v2 = min_speed - max_acc*max_acc*0.5*max_jerk;
      double v3 = min_speed;
      
      int interval_idx = 0;
      for(size_t j=1; j<max_speed_pc.points.size(); j++){
	//figure out the time for each segment
	double jerk = 0;
	double acc_pt_pre = acc_pc.points[j-1].z;
	double speed_pt_pre = speed_pc.points[j-1].z;
	double acc_pt = acc_pc.points[j].z;
	double speed_pt = speed_pc.points[j].z;
	double time_step;
	switch (interval_idx){
	  case 0:
	    jerk = max_jerk;
	    time_step = solveCubic(jerk, acc_pt_pre, speed_pt_pre, -0.05);
	    acc_pt = acc_pt_pre + time_step * jerk;
	    speed_pt = speed_pt_pre + time_step * acc_pt;
	    if(speed_pt > v1) interval_idx++;
	    break;
	  case 1:
	    jerk = 0.0;
	    time_step = solveCubic(jerk, acc_pt_pre, speed_pt_pre, -0.05);
	    acc_pt = acc_pt_pre + time_step * jerk;
	    speed_pt = speed_pt_pre + time_step * acc_pt;
	    if(speed_pt > v2) interval_idx++;
	    break;
	  case 2:
	    jerk = -max_jerk;
	    time_step = solveCubic(jerk, acc_pt_pre, speed_pt_pre, -0.05);
	    acc_pt = acc_pt_pre + time_step * jerk;
	    speed_pt = speed_pt_pre + time_step * acc_pt;
	    if(speed_pt > v3) {
	      interval_idx++;
	      acc_pt = 0.0;
	      speed_pt = v3;
	    }
	    break;
	  case 3:
	    speed_pt = v3;
	    break;
	}
	acc_pc.points[j].z = acc_pt;
	jerk_pc.points[j].z = jerk;
	speed_pc.points[j].z = speed_pt;
      }
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