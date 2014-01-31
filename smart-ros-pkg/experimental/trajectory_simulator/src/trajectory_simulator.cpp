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

struct PointInfo{
  geometry_msgs::Point position;
  double dist, curvature, max_speed, speed_profile;
  double acceleration, jerk, time;
  int idx;
};

bool compareByMaxSpeed(const PointInfo &a, const PointInfo &b){
  return a.max_speed < b.max_speed;
}

bool compareByIdx(const PointInfo &a, const PointInfo &b){
  return a.idx < b.idx;
}

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
    
  double SetSpeed(double x, double a){
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
    return dx;
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
  double max_acc_, max_jerk_;
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

  double getTimeStep(double jerk, double acc, double speed, double dist){
    return solveCubic(jerk/6.0, acc/2.0, speed, -dist);
  }
  
  double solveCubic(double a, double b, double c, double d){
    //http://www.1728.org/cubic2.htm
    //example: time_step = solveCubic(jerk, acc_pt_pre, speed_pt_pre, -0.05);
    if(fabs(a)<1e-9){
      //cout<<fabs(a)<<endl;
      a=b;
      b=c;
      c=d;
      double temp = b*b - 4*a*c;
      //cout <<a<<" "<<b<<" "<<c<<" "<< temp<<endl;
      return (-b + sqrt(temp))/(2*a);
    }
    double f = ((3*c/a)-((b*b)/(a*a)))/3;
    double g = ((2*pow(b,3)/pow(a,3))-(9*b*c/pow(a,2))+(27*d/a))/27.0;
    double h = pow(g,2)/4.0+pow(f,3)/27.0;
    
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
      //cout<<x1<<" "<<x2<<" "<<x3<<endl;
      if(x1 < minx && x1>=0) {minx = x1; match_criteria=true;}
      if(x2 < minx && x2>=0) {minx = x2; match_criteria=true;}
      if(x3 < minx && x3>=0) {minx = x3; match_criteria=true;}
      assert(match_criteria);
      return minx;
    }
    
    
  }
  
  vector<PointInfo> localMinimaSearch(vector<PointInfo> path_info){
    //simple local minima search
    //accelerate true, deccelerate false
    
    bool last_acc_sign;
    double cur_vel = 0.0;
    vector<PointInfo> minima_pts;
    
    //minima_pts.push_back(path_info[1]);
    last_acc_sign = (path_info[1].max_speed - path_info[0].max_speed) > 0;
    cout<<last_acc_sign<<endl;
    for(size_t i=2; i<path_info.size(); i++){
      double vel_speed = path_info[i].max_speed - path_info[i-1].max_speed;
      bool acc_sign = vel_speed > 0;
      if(acc_sign != last_acc_sign){
	//only find out the part where minimum pt occur
	if(acc_sign){
	  minima_pts.push_back(path_info[i]);
	}
	last_acc_sign = acc_sign;
      }
    }
    return minima_pts;
  }
  
  void dynamicAccelerationProfile(int end, double max_jerk, vector<PointInfo> &path_info,
			   double &acc_pt_pre, double &speed_pt_pre, int &last_acceleration_idx){
    for(int k=last_acceleration_idx+1; k<=end; k++){
      double time_step = getTimeStep(max_jerk, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre + max_jerk*time_step;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step + max_jerk*pow(time_step,2)*0.5;
      if(max_jerk>0){
      if(acc_now >= max_acc_) {
	  last_acceleration_idx = k;
	  break;
	}
      }
      else {
	if(acc_now <=0){
	  last_acceleration_idx = k;
	  break;
	}
      }
      path_info[k].speed_profile = speed_now;
      path_info[k].time = time_step;
      path_info[k].acceleration = acc_now;
      path_info[k].jerk = max_jerk;
      acc_pt_pre = acc_now;
      speed_pt_pre = speed_now;
      //cout<<"1: "<<time_step<<" "<<speed_pt_pre<<" "<<max_jerk<<" "<<acc_pt_pre<<endl;
    }
  }
  
  void constAccelerationProfile(int end, double v2, vector<PointInfo> &path_info, 
			   double &acc_pt_pre, double &speed_pt_pre, int &last_acceleration_idx){
    for(int k=last_acceleration_idx; k<=end; k++){
      double time_step = getTimeStep(0.0, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step;
      if(speed_now >=v2){
	last_acceleration_idx = k;
	break;
      }
      path_info[k].speed_profile = speed_now;
      path_info[k].time = time_step;
      path_info[k].acceleration = acc_now;
      path_info[k].jerk = 0.0;
      acc_pt_pre = acc_now;
      speed_pt_pre = speed_now;
      //cout<<"2: "<<time_step<<" "<<speed_pt_pre<<" "<<acc_pt_pre<<endl;
    }
  }
  
  TrajectorySimulator(int argc, char** argv){
    
    double a_pre=0.0;
    double v_pre=0.0;
    double jerk_now=1.0;
    double t_total = 0.0;
    for(int i=0; i<100; i++){
      if(i>=50) jerk_now = -1.0;
      double t_now = getTimeStep(jerk_now,a_pre,v_pre,0.05);
      double a_now = a_pre + jerk_now*t_now;
      double v_now = v_pre + a_pre*t_now + 0.5*jerk_now*t_now*t_now;
      v_pre = v_now;
      a_pre = a_now;
      t_total+=t_now;
      //cout<<t_total<<": "<<t_now<<" "<<a_now<<" "<<v_now<<endl;
    }
    //return;
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
    max_acc_ = max_acc;
    max_jerk_ = max_jerk;
    global_frame_ = "/robot_0/map";
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    
    sensor_msgs::PointCloud pc;
    ros::Publisher pub, global_path_pub, curvature_pub, dist_pub;
    ros::Publisher max_speed_pub, speed_pub, acc_pub, jerk_pub;
    
    global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 1, true);
    curvature_pub = nh.advertise<sensor_msgs::PointCloud>("curvature_pt", 1, true);
    max_speed_pub = nh.advertise<sensor_msgs::PointCloud>("max_speed_pt", 1, true);
    speed_pub = nh.advertise<sensor_msgs::PointCloud>("speed_pt", 1, true);
    acc_pub = nh.advertise<sensor_msgs::PointCloud>("acc_pt", 1, true);
    jerk_pub = nh.advertise<sensor_msgs::PointCloud>("jerk_pt", 1, true);
    dist_pub = nh.advertise<sensor_msgs::PointCloud>("dist_pt", 1, true);
    setGlobalPath(global_path_pub);
    
    vector<PointInfo> path_info;
    
    geometry_msgs::Point first_pt = pp_->path_.poses[0].pose.position;
    geometry_msgs::Point sec_pt = pp_->path_.poses[1].pose.position;
    double car_init_orientation = atan2(sec_pt.y-first_pt.y, sec_pt.x-first_pt.x);
    CarModel model(car_length, time_step, poseVec(pp_-> path_.poses[0].pose.position.x, 
						  pp_-> path_.poses[0].pose.position.y, car_init_orientation));
    
    
    bool path_exist = true;
    fmutil::Stopwatch sw("Simulation took");
    vector<int> speed_intervals;
    double speed_now = 3.0, acc_now = 0;
    double dist_travel = 0.0;
    int path_no = 0;
    while(path_exist && ros::ok()){
      double steer_angle, dist_to_goal;
      PointInfo point_info;
      point_info.position.x = model.pose_.x;
      point_info.position.y = model.pose_.y;
      point_info.position.z = model.pose_.a;
      geometry_msgs::PoseStamped p;
      p.header.frame_id =global_frame_;
      p.header.stamp = ros::Time::now();
      p.pose.position.x = model.pose_.x;
      p.pose.position.y = model.pose_.y;
      p.pose.orientation = tf::createQuaternionMsgFromYaw(model.pose_.a);
      pp_->vehicle_base_ = p.pose;
      path_exist = pp_->steering_control(&steer_angle, &dist_to_goal);
      
      dist_travel += model.SetSpeed(speed_now, steer_angle);
      point_info.dist = dist_travel;
      pp_->updateCommandedSpeed(speed_now);
      double turning_rad = fabs(car_length / tan(steer_angle));
      point_info.curvature = turning_rad;
      point_info.max_speed = sqrt(max_lat_acc*turning_rad);
      point_info.idx = path_no++;
      if(point_info.max_speed > max_speed) point_info.max_speed = max_speed;
      path_info.push_back(point_info);
    }
    
    cout<<path_info.size()<<" point size"<<endl;
    vector<PointInfo> local_minima_pts = localMinimaSearch(path_info);
    sort(local_minima_pts.begin(), local_minima_pts.end(), compareByMaxSpeed);
    cout<<local_minima_pts.size()<<" local minima found"<<endl;
    for(size_t i=0; i<1; i++){
      vector<PointInfo> split_sections;
      split_sections.push_back(path_info[0]);
      split_sections.push_back(path_info[path_info.size()-1]);
      split_sections.insert(split_sections.end(), local_minima_pts.begin(), local_minima_pts.begin()+i);
      sort(split_sections.begin(), split_sections.end(), compareByIdx);
      cout<<"*********************************"<<endl;
      cout<<"Max speed "<<local_minima_pts[i].max_speed<<endl;
      double max_speed = local_minima_pts[i].max_speed;
      for(size_t j=1; j<split_sections.size(); j++){
	cout<<j<<": "<<split_sections[j].idx<<"|"<<split_sections[j].dist<<endl;
	int end = split_sections[j].idx;
	int start = split_sections[j-1].idx;
	double length = path_info[end].dist - path_info[start].dist;
	if(fabs(path_info[start].acceleration) <= 1e-3 && fabs(path_info[end].acceleration) <= 1e-3){
	  double minimum_dist;
	  if(path_info[start].speed_profile >= max_acc*max_acc/max_jerk)
	    minimum_dist = path_info[start].speed_profile/2.0*(path_info[start].speed_profile/max_acc+max_acc/max_jerk);
	  else
	    minimum_dist = path_info[start].speed_profile*sqrt(path_info[start].speed_profile/max_jerk);
	  //check for case e
	  if(length > minimum_dist){
	    //check for case c
	    if(max_speed < path_info[start].speed_profile + max_acc*max_acc/max_jerk){
	      cout<<"In CASE C"<<endl;
	    }
	    else {
	      double full_profile_dist = max_speed/2.0*((max_speed-(2*path_info[start].speed_profile))/max_acc +
	      max_acc/max_jerk);
	      full_profile_dist *= 2.0;
	      //check for case b
	      if(length < full_profile_dist){
		cout<<"In CASE B"<<endl;
	      }
	      //case a
	      else{
		double v1 = path_info[start].speed_profile+max_acc*max_acc/(2.0*max_jerk);
		double v2 = max_speed - max_acc*max_acc/(2.0*max_jerk);
		//buggy, need correct formula both for solving the time step and calculating the speed
		cout<<"In CASE A v1="<<v1<<" v2="<<v2<<" v3="<<max_speed<<endl;
		double acc_pt_pre = 0.0;
		double speed_pt_pre = path_info[start].speed_profile;
		int last_acceleration_idx, last_decceleration_idx;
		last_acceleration_idx = start;
		//Perhaps ok??
		//The following algorithm produce acceleration takes 2.87 sec to go from 0 to 1.864 m/s within distance of 2.65 m
		//Spreadsheet claculation sohws acceleration takes 3.00 sec to go from 0 to 1.89 m/s within distance of 2.835 m
		dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, speed_pt_pre, last_acceleration_idx);
		constAccelerationProfile(end, v2, path_info, acc_pt_pre, speed_pt_pre, last_acceleration_idx);
		dynamicAccelerationProfile(end, -max_jerk, path_info, acc_pt_pre, speed_pt_pre, last_acceleration_idx);
		//just copy the acceleration profile and put it in reverse
		last_decceleration_idx = end;
		for(int j=start; j<=last_acceleration_idx; j++){
		  //cout<<last_decceleration_idx<<" "<<path_info[j].speed_profile<<endl;
		  path_info[last_decceleration_idx].acceleration = path_info[j].acceleration;
		  path_info[last_decceleration_idx].jerk = path_info[j].jerk;
		  path_info[last_decceleration_idx].speed_profile = path_info[j].speed_profile;
		  path_info[last_decceleration_idx].time = path_info[j+1].time;
		  //cout<<path_info[last_decceleration_idx].speed_profile<<endl;
		  last_decceleration_idx--;
		}
		for(int j=last_acceleration_idx+1; j<=last_decceleration_idx; j++){
		  path_info[j].acceleration = 0.0;
		  path_info[j].jerk = 0.0;
		  path_info[j].speed_profile = speed_pt_pre;
		  path_info[j].time = 0.05/speed_pt_pre;
		}
	      }
	      //cout<<full_profile_dist<<", ";
	    }
	  }
	  else {
	    cout<<"ERROR"<<endl;
	  }
	}
	else {
	  cout<<"Acceleration not zero"<<endl;
	}
      }    
    }
    sw.end();
    sensor_msgs::PointCloud curve_pc, max_speed_pc, speed_pc, acc_pc, jerk_pc, dist_pc;
    dist_pc.header.frame_id = global_frame_;
    dist_pc.header.stamp = ros::Time::now();
    max_speed_pc.header = speed_pc.header = acc_pc.header = jerk_pc.header = curve_pc.header = dist_pc.header;
    double time_t = 0.0;
    for(size_t i=0; i<path_info.size(); i++){
      time_t += path_info[i].time;
      geometry_msgs::Point32 p;
      p.x = path_info[i].position.x;
      p.y = path_info[i].position.y;
      p.z = path_info[i].curvature;
      curve_pc.points.push_back(p);
      p.y += 100;
      p.z = path_info[i].dist;
      dist_pc.points.push_back(p);
      p.x = time_t;
      p.y = path_info[i].max_speed + 100;;
      p.z = path_info[i].max_speed;
      max_speed_pc.points.push_back(p);
      p.y = path_info[i].speed_profile + 100;
      p.z = path_info[i].speed_profile;
      speed_pc.points.push_back(p);
      p.y = path_info[i].acceleration + 200;
      p.z = path_info[i].acceleration;
      acc_pc.points.push_back(p);
      p.y = path_info[i].jerk + 300;
      p.z = path_info[i].jerk;
      jerk_pc.points.push_back(p);
      //cout<<path_info[i].speed_profile<<" "<<path_info[i].max_speed<<" "<<path_info[i].acceleration<<endl;
    }
    curvature_pub.publish(curve_pc);
    max_speed_pub.publish(max_speed_pc);
    speed_pub.publish(speed_pc);
    acc_pub.publish(acc_pc);
    jerk_pub.publish(jerk_pc);
    dist_pub.publish(dist_pc);
    
    sensor_msgs::PointCloud local_pc;
    local_pc.header = max_speed_pc.header;
    ros::Publisher local_min_pub = nh.advertise<sensor_msgs::PointCloud>("local_min_puts", 1, true);
    for(size_t i=0; i<local_minima_pts.size(); i++){
      geometry_msgs::Point32 p;
      p.x = local_minima_pts[i].position.x;
      p.y = local_minima_pts[i].position.y+=100;
      p.z = local_minima_pts[i].max_speed;
      local_pc.points.push_back(p);
    }
    local_min_pub.publish(local_pc);
    
      
    ros::spin();
      
  }
  
  
};

int main(int argc, char** argv){
  TrajectorySimulator ts(argc, argv);
  return 0;
}