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
  double dist, curvature, max_speed, curve_max_speed, speed_profile;
  double min_dist;
  double acceleration, jerk, time;
  int idx;
  bool verified_ok;
  PointInfo(): jerk(0.0), verified_ok(false){};
  void copy(const PointInfo &p){
    this->speed_profile = p.speed_profile;
    this->acceleration = p.acceleration;
    this->jerk = p.jerk;
    this->time = p.time;
    //cout<<this->idx<<"\t"<<p.time<<"\t"<<p.speed_profile<<"\t"<<p.acceleration<<"\t"<<p.jerk<<endl;
  }
  
  bool operator<(const PointInfo &p) const { 
    return this->idx < p.idx; 
  }
};

bool compareByMaxSpeed(const PointInfo &a, const PointInfo &b){
  cout<<a.max_speed<<" "<<b.max_speed<<" "<<(a.max_speed < b.max_speed)<<endl;
  return a.max_speed < b.max_speed;
}

bool compareByIdx(const PointInfo &a, const PointInfo &b){
  return a.idx < b.idx;
}

class CarModel{
public:
  
  double wheelbase_;
  
  double dist_res_;
  double time_now_;
  
  poseVec vel_, pose_;
  CarModel(double wheelbase, double dist_res): wheelbase_(wheelbase), 
  dist_res_(dist_res), time_now_(0.0){}
  CarModel(double wheelbase, double dist_res, poseVec init_pose):
  wheelbase_(wheelbase), dist_res_(dist_res), time_now_(0.0), pose_(init_pose){}
    
  double SetSpeed(double x, double a){
    //we want to maintain a fixed distance, not time fixed
    
    
    vel_.x = x * cos(a);
    vel_.a = x * sin(a)/wheelbase_;
    
    //figure out the time step for a fixed amount of distance
    //let's do a 5 cm resolution.
    double time_step = dist_res_/vel_.x;
    pose_.a = normalize(pose_.a + vel_.a * time_step);
    double cosa = cos(pose_.a);
    double sina = sin(pose_.a);
    pose_.x += dist_res_ * cosa;
    pose_.y += dist_res_ * sina;
    time_now_+=time_step;
    return dist_res_;
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
  
  double max_lat_acc_, max_speed_;
  double max_acc_, max_jerk_;
  double dist_res_;
  ros::Publisher global_path_pub_;
  ros::Publisher pub, curvature_pub, dist_pub;
  ros::Publisher max_speed_pub, speed_pub, speed_xy_pub, acc_pub, jerk_pub;
  ros::Publisher local_min_pub;
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
  
    
  double getNewNewSpeed(double v0, double v1, double local_dist){
    double a = 1.0/max_acc_;
    double b = max_acc_/max_jerk_;
    double c = max_acc_/(2*max_jerk_)*(v0+v1) - 1.0/(2*max_acc_)*(v0*v0+v1*v1) - (local_dist - 2*0.05);
    double bsq_4ac = sqrt(b*b-4*a*c);
    double answer1 = (-b+bsq_4ac)/(2*a);
    double answer2 = (-b-bsq_4ac)/(2*a);
    //cout << " answer1 "<<answer1<<" answer2 "<<answer2<<" "<<a<<" "<<b<<" "<<c<<" ";
    if(answer1 > answer2 && answer2>0) answer1 = answer2;
    
    return answer1;
  }
  
public:
  
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

  double getTimeStep(double jerk, double acc, double speed, double dist){
    return solveCubic(jerk/6.0, acc/2.0, speed, -dist);
  }
  
  double getReversedTimeStep(double jerk, double acc, double speed, double dist){
    return solveCubic(jerk/6.0, -acc/2.0, speed, -dist);
  }
  
  double solveCubic(double a, double b, double c, double d, bool showOutput=false){
    //http://www.1728.org/cubic2.htm
    //example: time_step = solveCubic(jerk, acc_pt_pre, speed_pt_pre, -0.05);
    if(fabs(a)<1e-9){
      //cout<<fabs(a)<<endl;
      a=b;
      b=c;
      c=d;
      double temp = b*b - 4*a*c;
      //this may occur if decceleration overshoot given a distance
      if(showOutput)
	cout<<" a b c temp "<<a<<" "<<b<<" "<<c<<" "<<temp<<endl;
      if(temp < 0) return 1e9;
      else return (-b + sqrt(temp))/(2*a);
    }
    if(showOutput)
      cout <<a<<" "<<b<<" "<<c<<" "<< d<<endl;
    double f = ((3*c/a)-((b*b)/(a*a)))/3;
    double g = ((2*pow(b,3)/pow(a,3))-(9*b*c/pow(a,2))+(27*d/a))/27.0;
    double h = pow(g,2)/4.0+pow(f,3)/27.0;
    
    if(h > 0){
      double r = -g/2.0+sqrt(h);
      double s = sgn(r)*pow(fabs(r), 1/3.0);
      double t = -g/2.0-sqrt(h);
      double u = sgn(t) * pow(fabs(t),1/3.0);
      double answer = s+u-(b/(3*a));
      if(showOutput)
	cout<<" solveCubic "<<answer<<" r "<<r<<" s "<<s<<" t "<<t<<" u "<<u<<endl;
      return answer;
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
      if(showOutput)
	cout<<" solveCubic "<<x1<<" "<<x2<<" "<<x3<<endl;
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
    bool limit_exceeded = false;
    if(minima_pts.size()>0)
      minima_pts[0].verified_ok = true;
    for(size_t i=1; i<minima_pts.size(); i++){
      double speed_diff = minima_pts[i].max_speed - minima_pts[i-1].max_speed;
      double dist = minima_pts[i].dist - minima_pts[i-1].dist;
      double acc = speed_diff/(dist/max_speed_);
      cout<<i<<": acc="<<acc<<endl;
      bool cur_acc_sign = acc > 0;
      if(limit_exceeded){
	if(last_acc_sign == cur_acc_sign)
	  minima_pts[i-1].verified_ok =false;
	limit_exceeded = false;
      }
      if(fabs(acc) > max_acc_)
	limit_exceeded = true;
      minima_pts[i].verified_ok = true;
      last_acc_sign = cur_acc_sign;
    }
    for(size_t i=0; i<minima_pts.size();){
      if(!minima_pts[i].verified_ok)
	minima_pts.erase(minima_pts.begin()+i);
      else
	i++;
    }
    return minima_pts;
  }
  
  
  void publishPathInfo(vector<PointInfo> path_info, vector<PointInfo> local_minima_pts){
    sensor_msgs::PointCloud pc;
    
    
    sensor_msgs::PointCloud curve_pc, max_speed_pc, speed_pc, acc_pc, jerk_pc, dist_pc, speed_xy_pc;
    dist_pc.header.frame_id = global_frame_;
    dist_pc.header.stamp = ros::Time::now();
    speed_xy_pc.header = max_speed_pc.header = speed_pc.header = acc_pc.header = jerk_pc.header = curve_pc.header = dist_pc.header;
    double time_t = 0.0;
    path_info[0].time = 0.0;
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
      p.y += 100;
      p.z = path_info[i].speed_profile;
      speed_xy_pc.points.push_back(p);
      p.x = time_t;
      p.y = path_info[i].curve_max_speed + 100;;
      p.z = path_info[i].curve_max_speed;
      max_speed_pc.points.push_back(p);
      p.y = path_info[i].speed_profile + 100;
      p.z = path_info[i].speed_profile;
      //cout<<path_info[i].time<<" "<<p.z<<endl;
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
    speed_xy_pub.publish(speed_xy_pc);
    
    sensor_msgs::PointCloud local_pc;
    local_pc.header = max_speed_pc.header;
    for(size_t i=0; i<local_minima_pts.size(); i++){
      geometry_msgs::Point32 p;
      p.x = speed_pc.points[local_minima_pts[i].idx].x;
      p.y = local_minima_pts[i].max_speed+100;
      p.z = local_minima_pts[i].max_speed;
      local_pc.points.push_back(p);
    }
    cout<<"Publishing "<<local_minima_pts.size()<<" with "<<path_info.size()<<" paths"<<endl;
    local_min_pub.publish(local_pc);
  }
  
  void printLocalMinimaStatus(string msg, vector<PointInfo> &local_minima_pts){
    cout<<endl<<"********* start:"<<msg<<"**********"<<endl;
    cout<<"idx\tdist\tmax_speed\tprofile\tverified\tmin_dist"<<endl;
    for(size_t i=0; i<local_minima_pts.size(); i++){
      cout<<local_minima_pts[i].idx<<"\t"<<local_minima_pts[i].dist<<"\t"
      <<local_minima_pts[i].max_speed<<"\t"<<local_minima_pts[i].speed_profile<<"\t"
      <<local_minima_pts[i].verified_ok<<"\t"
      <<local_minima_pts[i].min_dist
      <<endl;
    }
    cout<<endl<<"********* end:"<<msg<<"**********"<<endl;
    cout<<endl;
  }
  
  
  TrajectorySimulator(double max_lat_acc, double max_speed, double max_acc, double max_jerk, double dist_res, 
		      ros::NodeHandle &nh):max_lat_acc_(max_lat_acc), max_speed_(max_speed), max_acc_(max_acc),
		      max_jerk_(max_jerk), dist_res_(dist_res){
      global_path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1, true);
      curvature_pub = nh.advertise<sensor_msgs::PointCloud>("curvature_pt", 1, true);
      max_speed_pub = nh.advertise<sensor_msgs::PointCloud>("max_speed_pt", 1, true);
      speed_pub = nh.advertise<sensor_msgs::PointCloud>("speed_pt", 1, true);
      speed_xy_pub = nh.advertise<sensor_msgs::PointCloud>("speed_xy_pt", 1, true);
      acc_pub = nh.advertise<sensor_msgs::PointCloud>("acc_pt", 1, true);
      jerk_pub = nh.advertise<sensor_msgs::PointCloud>("jerk_pt", 1, true);
      dist_pub = nh.advertise<sensor_msgs::PointCloud>("dist_pt", 1, true);
      local_min_pub = nh.advertise<sensor_msgs::PointCloud>("local_min_puts", 1, true);
    };
  
  vector<PointInfo> getSpeedProfile() {  
    double min_look_ahead_dist = 4.0;
    double forward_achor_pt_dist = 1.0;
    double car_length = 2.55;
    double speed_ini = 0.0;
    global_frame_ = "/robot_0/map";
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    
    setGlobalPath(global_path_pub_);
    
    vector<PointInfo> path_info;
    
    geometry_msgs::Point first_pt = pp_->path_.poses[0].pose.position;
    geometry_msgs::Point sec_pt = pp_->path_.poses[1].pose.position;
    double car_init_orientation = atan2(sec_pt.y-first_pt.y, sec_pt.x-first_pt.x);
    CarModel model(car_length, dist_res_, poseVec(pp_-> path_.poses[0].pose.position.x, 
						  pp_-> path_.poses[0].pose.position.y, car_init_orientation));
    
    
    bool path_exist = true;
    fmutil::Stopwatch sw("Simulation took");
    vector<int> speed_intervals;
    double speed_now = 3.0;
    double dist_travel = 0.0;
    int path_no = 0;
    geometry_msgs::Pose initial_pose;
    initial_pose.position = first_pt;
    initial_pose.orientation = tf::createQuaternionMsgFromYaw(car_init_orientation);
    pp_->vehicle_base_ = initial_pose;
    cout<<initial_pose<<endl;
    PointInfo initial_point;
    initial_point.position = initial_pose.position;
    initial_point.dist = 0.0;
    initial_point.max_speed = max_speed_;
    initial_point.curve_max_speed = max_speed_;
    initial_point.idx = path_no++;
    path_info.push_back(initial_point);
    while(path_exist && ros::ok()){
      double steer_angle, dist_to_goal;
      path_exist = pp_->steering_control(&steer_angle, &dist_to_goal);
      dist_travel += model.SetSpeed(speed_now, steer_angle);
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
      point_info.dist = dist_travel;
      pp_->updateCommandedSpeed(speed_now);
      double turning_rad = fabs(car_length / tan(steer_angle));
      point_info.curvature = turning_rad;
      point_info.max_speed = sqrt(max_lat_acc_*turning_rad);
      point_info.idx = path_no++;
      if(point_info.max_speed > max_speed_) point_info.max_speed = max_speed_;
      point_info.speed_profile = max_speed_;
      point_info.curve_max_speed = point_info.max_speed;
      point_info.time = dist_res_/max_speed_;
      path_info.push_back(point_info);
    }
    cout<<path_info.size()<<" point size"<<endl;
    
    vector<PointInfo> local_minima_pts = localMinimaSearch(path_info);
    PointInfo start_pt;
    start_pt.max_speed = speed_ini;
    start_pt.dist = 0.0;
    start_pt.idx = 0;
    local_minima_pts.push_back(start_pt);
    path_info[path_info.size()-1].max_speed = 0.0;
    local_minima_pts.push_back(path_info[path_info.size()-1]);
    sort(local_minima_pts.begin(), local_minima_pts.end());
    printLocalMinimaStatus("after sort 2", local_minima_pts);
    
    fmutil::Stopwatch sw2("connecting points");
//     speedProfileConnectingMinima(local_minima_pts, path_info);
    double percent=0.0;
    for(; percent<=1.05; percent+=0.05){
      vector<PointInfo> *path_copy= new vector<PointInfo>(path_info);
      vector<PointInfo> *minima_copy= new vector<PointInfo>(local_minima_pts);
      speedProfileProfilingMinimas(*minima_copy, *path_copy, percent);
      bool violated = false;
      for(size_t i=0; i<path_copy->size(); i++)
	if((*path_copy)[i].speed_profile - (*path_copy)[i].max_speed > 0.1){
	  cout<<(*path_copy)[i].speed_profile - (*path_copy)[i].max_speed<<endl;
	  violated = true;
	  break;
	}
      cout<<"Percent "<<percent<<" "<<violated<<endl;
      if(!violated) break;
    }
    speedProfileProfilingMinimas(local_minima_pts, path_info, percent);
    sw2.end();
    sw.end();
    publishPathInfo(path_info, local_minima_pts);
    return path_info;
  }
  
  void speedProfileProfilingMinimas(vector<PointInfo> &local_minima_pts, vector<PointInfo> &path_info, double allowable_percent){
    
    //for each local minima build decceleration and acceleration profile given a minima
    double req_speed_inc = max_acc_*max_acc_/max_jerk_;
    int path_size = path_info.size();
    vector<PointInfo> new_start_end_pts;
    for(size_t i=0; i<local_minima_pts.size(); i++){
      if(fabs(local_minima_pts[i].max_speed - max_speed_)<1e-5) continue;
      vector<PointInfo> new_profile_temp;
      if(max_speed_ - local_minima_pts[i].max_speed >= req_speed_inc)
	 new_profile_temp = completeAccelerationProfile(local_minima_pts[i].max_speed, max_speed_);
      else
	new_profile_temp = shortAccelerationProfile(local_minima_pts[i].max_speed, max_speed_, max_jerk_, dist_res_);
      //fill up the acc part
      bool speed_violated = false;
      bool stop_arguing = false;
      PointInfo constant_speed;
      constant_speed.speed_profile = local_minima_pts[i].max_speed;
      constant_speed.time = dist_res_/local_minima_pts[i].max_speed;
      
//       printLocalMinimaStatus("new profile", new_profile_temp);
     
//       cout<<i<<": "<<new_profile_temp.size()<<" "<<local_minima_pts[i].max_speed<<endl;
      PointInfo start_acc_pt_rec, end_acc_pt_rec, start_dec_pt_rec, end_dec_pt_rec;
      int path_idx, j;
      int last_continuous_seg = -1;
      for(j=0, path_idx = local_minima_pts[i].idx; j<new_profile_temp.size(); j++, path_idx++){
// 	cout<<local_minima_pts[i].idx<<" i "<<i<<" j "<<j<<" path_idx "<<path_idx<<" speed_violated "<<speed_violated<<" stop_arguing "<<stop_arguing<<endl;
	if(fabs(path_info[path_idx].max_speed / max_speed_)>allowable_percent) stop_arguing = true;
	if(path_idx > path_size) break;
	if(speed_violated){
// 	  cout<<"Speed violated. Assinging constant speed from "<<local_minima_pts[i].idx<<" to "<<path_idx<<endl;
	  //max speed exceeded. All acceleration will be delayed to until this idx
	  //profile before this will just be a constant speed
	  for(int k=local_minima_pts[i].idx; k<path_idx; k++){
	    if(path_info[k].speed_profile > constant_speed.speed_profile)
	      path_info[k].copy(constant_speed);
	  }
	  speed_violated = false;
	  j=0;
	}
	if(new_profile_temp[j].speed_profile > path_info[path_idx].max_speed && !stop_arguing){
	  speed_violated = true;
// 	  cout<<i<<": at "<<j<<" speed "<<new_profile_temp[j].speed_profile<<" larger than "<<path_idx<<" "<<path_info[path_idx].max_speed<<endl;
	  continue;
	}
	else {
	  if(path_info[path_idx].speed_profile > new_profile_temp[j].speed_profile){
	    path_info[path_idx].copy(new_profile_temp[j]);
// 	    path_info[path_idx].curve_max_speed = j;
	    if(last_continuous_seg==-1){
	      start_acc_pt_rec.max_speed = new_profile_temp[j].speed_profile;
	      start_acc_pt_rec.idx = path_idx;
	      last_continuous_seg = path_idx;
	    }
	    else {
	      if(path_idx - last_continuous_seg == 1){
		end_acc_pt_rec.max_speed = new_profile_temp[j].speed_profile;
		end_acc_pt_rec.idx = path_idx;
		last_continuous_seg = path_idx;
	      }
	      else{
		//consider only the last segment
		j--;
		path_idx--;
		last_continuous_seg = -1;
	      }
	    }
	  }
	}
      }
      path_idx = local_minima_pts[i].idx;
      speed_violated = false;
      stop_arguing = false;
//       cout<<i<<": "<<new_profile_temp.size()<<" "<<local_minima_pts[i].max_speed<<endl;
      last_continuous_seg = -1;
      //it could be the case where now acceleration occur and skipped to decceleration.
      //need to initialize to avoid crash
      start_dec_pt_rec.max_speed = new_profile_temp[0].speed_profile;
      start_dec_pt_rec.idx = 0;
      for(j=0, path_idx = local_minima_pts[i].idx; j<new_profile_temp.size(); j++, path_idx--){
// 	cout<<"j "<<j<<" path_idx "<<path_idx<<" speed_violated "<<speed_violated<<" stop_arguing "<<stop_arguing<<endl;
	if(path_idx < 0) break;
	if(fabs(path_info[path_idx].max_speed /max_speed_)>allowable_percent) stop_arguing = true;
	if(speed_violated || (!stop_arguing && path_idx == 0)){
// 	  cout<<"Speed violated. Assinging constant speed from "<<local_minima_pts[i].idx<<" to "<<path_idx<<endl;
	  for(int k=local_minima_pts[i].idx; k>=path_idx; k--){
	    if(path_info[k].speed_profile > constant_speed.speed_profile)
	      path_info[k].copy(constant_speed);
	  }
	  speed_violated = false;
	  j=0;
	}
	if(new_profile_temp[j].speed_profile > path_info[path_idx].max_speed && !stop_arguing){
	  speed_violated = true;
// 	  cout<<"Speed violated at "<<path_idx<<" with "<<j<<endl;
	  continue;
	}
	else {
	  if(path_info[path_idx].speed_profile > new_profile_temp[j].speed_profile){
	    path_info[path_idx].copy(new_profile_temp[j]);
	    if(last_continuous_seg==-1){
	      end_dec_pt_rec.max_speed = new_profile_temp[j].speed_profile;
	      end_dec_pt_rec.idx = path_idx;
	      last_continuous_seg = path_idx;
	    }
	    else {
	      if(last_continuous_seg - path_idx == 1){
		start_dec_pt_rec.max_speed = new_profile_temp[j].speed_profile;
		start_dec_pt_rec.idx = path_idx;
		last_continuous_seg = path_idx;
	      }
	      else {
		j--;
		path_idx++;
		last_continuous_seg = -1;
	      }
	    }
// 	    path_info[path_idx].curve_max_speed = j;
	  }
	}
// 	cout<<"path_idx and j "<<path_idx<<" "<<j<<endl;
      }
      new_start_end_pts.push_back((start_dec_pt_rec));
      new_start_end_pts.push_back((end_dec_pt_rec));
      new_start_end_pts.push_back((start_acc_pt_rec));
      new_start_end_pts.push_back((end_acc_pt_rec));
      cout<<"DEC start_idx "<<start_dec_pt_rec.idx<<"("<<start_dec_pt_rec.max_speed<<") ";
      cout<<"DEC end_idx "<<end_dec_pt_rec.idx<<"("<<end_dec_pt_rec.max_speed<<") ";
      cout<<"ACC start_idx "<<start_acc_pt_rec.idx<<"("<<start_acc_pt_rec.max_speed<<") ";
      cout<<"ACC end_idx "<<end_acc_pt_rec.idx<<"("<<end_acc_pt_rec.max_speed<<")"<<endl;
    }
    local_minima_pts.clear();// = new_start_end_pts;
    for(size_t i=0; i<new_start_end_pts.size(); i++){
      if(fabs(path_info[new_start_end_pts[i].idx].speed_profile - new_start_end_pts[i].max_speed) < 1e-5)
	local_minima_pts.push_back(new_start_end_pts[i]);
    }
  }
  PointInfo visualizeLocalMinimaPt(PointInfo pt){
    PointInfo pt_temp;
    pt_temp.idx = pt.idx;
    pt_temp.max_speed = pt.speed_profile;
    return pt_temp;
  }
    
  void speedProfileConnectingMinima(vector<PointInfo> &local_minima_pts, vector<PointInfo> &path_info){
    
    connectingPathGivenLocalMin(path_info, local_minima_pts);
    //compare again
    vector<PointInfo> speed_errors;
    for(size_t i=0; i<path_info.size(); i++){
      PointInfo err;
      err.position = path_info[i].position;
      err.speed_profile = path_info[i].max_speed;
      err.max_speed = path_info[i].max_speed - path_info[i].speed_profile;
      err.idx = path_info[i].idx;
      err.curvature = path_info[i].curvature;
      err.time = path_info[i].time;
      err.dist = path_info[i].dist;
      if(err.max_speed < -0.1)
	speed_errors.push_back(err);
    }
    
    vector<PointInfo> error_local_minima = localMinimaSearch(speed_errors);
    printLocalMinimaStatus("err to be added", error_local_minima);
    for(size_t i=0; i<error_local_minima.size(); i++){
      PointInfo new_local_minima = error_local_minima[i];
      new_local_minima.max_speed = new_local_minima.speed_profile;
      local_minima_pts.push_back(new_local_minima);
    }
    sort(local_minima_pts.begin(), local_minima_pts.end());
    printLocalMinimaStatus("after sort 3", local_minima_pts);
    connectingPathGivenLocalMin(path_info, local_minima_pts);
  }
  void connectingPathGivenLocalMin(vector<PointInfo> &path_info, vector<PointInfo> &local_minima_pts){
    for(size_t i=1; i<local_minima_pts.size(); i++){
      double v0 = local_minima_pts[i-1].max_speed;
      double v1 = local_minima_pts[i].max_speed;
      if(fabs(v1-v0)<1e-5){
	//trivial case just ok  
	cout<<i<<": Same speed OK!"<<endl;
	double constant_time_step = dist_res_/local_minima_pts[i].max_speed;
	for(int j=local_minima_pts[i-1].idx; j<=local_minima_pts[i].idx; j++){
	  path_info[j].speed_profile = local_minima_pts[i].max_speed;
	  path_info[j].jerk = 0.0;
	  path_info[j].acceleration = 0.0;
	  path_info[j].time = constant_time_step;
	}
      }
      else {
	double dist = local_minima_pts[i].dist - local_minima_pts[i-1].dist;
	double speed_check = getNewNewSpeed(v0, v1, dist);
	if(speed_check > max_speed_) speed_check = max_speed_;
	//check if this higher speed check not violating the max speed
	int high_speed_idx;
	cout<<"Speed check = "<<speed_check<<" v0 "<<v0<<endl;
	if(speed_check > v0){
	  cout<<" min dist = "<<getMinDistFullProfile(v0, speed_check)<<endl;
	  double high_speed_dist = getMinDistFullProfile(v0, speed_check);
	  high_speed_idx = high_speed_dist/dist_res_+local_minima_pts[i-1].idx;
	cout<<"high_speed_idx = "<<high_speed_idx<<" min_dst "<< high_speed_dist<<endl;
	}
	else
	  high_speed_idx = local_minima_pts[i-1].idx;
	cout<<i<<": Speed check v0 "<<v0<<" v1 "<<v1<<" dist "<<dist<<" suggested speed "<<speed_check<<endl;
	cout<<" matching max speed "<<path_info[high_speed_idx].max_speed<<endl;
	
	if(speed_check > v0 && speed_check > v1 && 
	  (path_info[high_speed_idx].max_speed > speed_check || fabs(path_info[high_speed_idx].max_speed-speed_check)<1e-5)){
	  int start_idx = local_minima_pts[i-1].idx;
	  cout<<" OK! and start at "<<start_idx<<endl;
	  int end_idx = local_minima_pts[i].idx;
	  double req_speed_inc = max_acc_*max_acc_/max_jerk_;
	  vector<PointInfo> acc_profile;
	  if(speed_check-v0 > req_speed_inc) {
	    acc_profile = completeAccelerationProfile(v0, speed_check);
	    cout<<"Size of acc_profile = "<<acc_profile.size()<<" "<<start_idx<<endl;
	  }
	  else {
	    //add acceleration profile for short speed diff
	    acc_profile = shortAccelerationProfile(v0, speed_check, max_jerk_, dist_res_);
	    cout<<"Need acceleration profile for short speed diff "<<acc_profile.size()<<endl;
	  }
	  for(size_t j=0; j<acc_profile.size(); j++, start_idx++){
	    if(start_idx<0) continue;
	    path_info[start_idx].copy(acc_profile[j]);
	  }
	  vector<PointInfo> dec_profile;
	  if(speed_check-v1 > req_speed_inc){
	    dec_profile = completeDeccelerationProfile(speed_check, v1);
	    cout<<"Size of dec_profile = "<<dec_profile.size()<<" "<<end_idx<<endl;
	  }
	  else {
	    dec_profile = shortAccelerationProfile(speed_check, v1, -max_jerk_, dist_res_);
	    cout<<"Need decceleration profile for short speed diff "<<dec_profile.size()<<endl;
	  }
	  for(int j=(int)dec_profile.size()-1; j>=0; j--, end_idx--){
	    path_info[end_idx].copy(dec_profile[j]);
	  }
	  PointInfo constant_speed;
	  constant_speed.speed_profile = speed_check;
	  constant_speed.time = dist_res_/speed_check;
	  for(int j=start_idx; j<=end_idx; j++){
	    if(start_idx<0) continue;
	    path_info[j].copy(constant_speed);
	  }
	}
	else {
	  int start_idx = local_minima_pts[i-1].idx;
	  int end_idx = local_minima_pts[i].idx;
	  double req_speed_inc = max_acc_*max_acc_/max_jerk_;
	  double given_dist = local_minima_pts[i].dist - local_minima_pts[i-1].dist;
	  if(v0<v1){
	    vector<PointInfo> acc_single_profile;
	    double profile_dist;
	    if(v1-v0 >= req_speed_inc){
	      profile_dist = getMinDistFullProfile(v0, v1);
	      if(given_dist >= profile_dist)
		acc_single_profile = completeAccelerationProfile(v0, v1);
	      else{
		local_minima_pts[i].max_speed = getNewSpeedFullProfile(v0, given_dist);
		cout<<" New full speed acc: "<<local_minima_pts[i].max_speed<<endl;
		i--;
	      }
	    }
	    else {
	      profile_dist = getMinDist(v0, v1);
	      cout<<endl<<" profile dist "<<profile_dist<<" given dist "<<given_dist<<endl;
	      if(given_dist >= profile_dist || fabs(profile_dist-given_dist)<1e-5)
		acc_single_profile = shortAccelerationProfile(v0, v1, max_jerk_, dist_res_);
	      else{
		local_minima_pts[i].max_speed = getNewSpeedShortProfile(v0, given_dist);
		cout<<" New short speed acc: "<<local_minima_pts[i].max_speed<<endl;
		i--;
	      }
	    }
	    cout <<" Ok prepare for single acc profile "<<given_dist<<" "<<profile_dist<<" "<<acc_single_profile.size()*dist_res_<<endl;  
	    for(size_t j=0; j<acc_single_profile.size(); j++, start_idx++){
	      if(start_idx<0) continue;
	      path_info[start_idx].copy(acc_single_profile[j]);
	    }
	    PointInfo constant_speed;
	    constant_speed.speed_profile = v1;
	    constant_speed.time = dist_res_/v1;
	    for(int j=start_idx; j<=end_idx; j++)
	      path_info[j].copy(constant_speed);
	  }
	  else{
	    vector<PointInfo> dec_single_profile;
	    double profile_dist;
	    if(v1-v0 >= req_speed_inc){
	      profile_dist = getMinDistFullProfile(v1, v0);
	      cout<<endl<<"v1-v0>=req_speed_inc profile dist "<<profile_dist<<" given dist "<<given_dist<<endl;
	      if(given_dist >= profile_dist)
		dec_single_profile = completeDeccelerationProfile(v0, v1);
	      else {
		  local_minima_pts[i-1].max_speed = getNewSpeedFullProfile(v1, given_dist);
		  cout<<" New full speed dec: "<<local_minima_pts[i-1].max_speed<<endl;
		  if(i>1) i-=2;
		  else i--;
	      }
	    }
	    else {
	      profile_dist = getMinDist(v1, v0);
	      cout<<endl<<"v1-v0<req_speed_inc profile dist "<<profile_dist<<" given dist "<<given_dist<<endl;
	      if(given_dist >= profile_dist || fabs(profile_dist-given_dist)<1e-5)
		dec_single_profile = shortAccelerationProfile(v0, v1, -max_jerk_, dist_res_);
	      else {
		local_minima_pts[i-1].max_speed = getNewSpeedShortProfile(v1, given_dist);
		cout<<" New short speed dec: "<<local_minima_pts[i-1].max_speed<<endl;
		if(i>1) i-=2;
		else i--;
	      }
	    }
	    cout <<" Ok prepare for single dec profile "<<given_dist<<" "<<profile_dist<<" "<<dec_single_profile.size()*dist_res_<<endl;
	    for(int j=(int)dec_single_profile.size()-1; j>=0; j--, end_idx--){
	      path_info[end_idx].copy(dec_single_profile[j]);
	    }
	    PointInfo constant_speed;
	    constant_speed.speed_profile = v0;
	    constant_speed.time = dist_res_/v0;
	    for(int j=start_idx; j<=end_idx; j++){
	      if(start_idx<0) continue;
		path_info[j].copy(constant_speed);
	    }
	  }
	}
      }
    }  
  }
  
  double getNewSpeedFullProfile(double v0, double dist){
    //solveCubic(0.0, 1.0/(2*max_acc_), -max_acc_/max_jerk_-v0/(2*max_acc_), v0*max_acc_/max_jerk_-dist);
    return solveCubic(0.0, 1.0/(2*max_acc_), max_acc_/max_jerk_, 
		      v0*max_acc_/(2*max_jerk_) - v0*v0/(2*max_acc_) - dist);
  }
  
  double getNewSpeedShortProfile(double low_speed, double dist){
    return solveCubic(1.0, low_speed, -(low_speed*low_speed), -pow(low_speed,3)-dist*dist*max_jerk_, true);
  }
  
  double getMinDist(double v0, double v1){
    //this function will not respect maximum acceleration
    return (v1+v0)*sqrt((v1-v0)/max_jerk_);
  }
  double getMinDistFullProfile(double v0, double v1){
    double min_dist;
    //min_dist = v1*v1/(2*max_acc_) + v1*(-max_acc_/max_jerk_-v0/(2*max_acc_)) + v0*max_acc_/max_jerk_;
    min_dist = (v0+v1)*max_acc_/(2*max_jerk_)+(v1*v1-v0*v0)/(2*max_acc_);
    return min_dist;
  }
  vector<PointInfo> completeDeccelerationProfile(double v_start, double v_end){
    double speed_diff = max_acc_*max_acc_/(2*max_jerk_);
    double v0 = v_start;
    double v1 = v_start - speed_diff;
    double v2 = v_end + speed_diff;
    double v3 = v_end;
    vector<PointInfo> dec_p1 = dynamicAccelerationProfile(v0, v1, 0.0, -max_acc_, -max_jerk_, dist_res_);
    vector<PointInfo> dec_p2 = constAccelerationProfile(v1, v2, -max_acc_, dist_res_);
    vector<PointInfo> dec_p3 = dynamicAccelerationProfile(v2, v3, -max_acc_, 0.0, max_jerk_, dist_res_);
    vector<PointInfo> completeProfile = dec_p1;
    completeProfile.insert(completeProfile.end(), dec_p2.begin(), dec_p2.end());
    completeProfile.insert(completeProfile.end(), dec_p3.begin(), dec_p3.end());
    return completeProfile;
  }
  vector<PointInfo> completeAccelerationProfile(double v_start, double v_end){
    double speed_diff = max_acc_*max_acc_/(2*max_jerk_);
    double v0 = v_start;
    double v1 = v_start + speed_diff;
    double v2 = v_end - speed_diff;
    double v3 = v_end;
    vector<PointInfo> acc_p1 = dynamicAccelerationProfile(v0, v1, 0.0, max_acc_, max_jerk_, dist_res_);
    vector<PointInfo> acc_p2 = constAccelerationProfile(v1, v2, max_acc_, dist_res_);
    vector<PointInfo> acc_p3 = dynamicAccelerationProfile(v2, v3, max_acc_, 0.0, -max_jerk_, dist_res_);
    vector<PointInfo> completeProfile = acc_p1;
    completeProfile.insert(completeProfile.end(), acc_p2.begin(), acc_p2.end());
    completeProfile.insert(completeProfile.end(), acc_p3.begin(), acc_p3.end());
    return completeProfile;
  }
  vector<PointInfo> addVirtualPoint(double v0, double a0, double j0){
    vector<PointInfo> speed_profile;
    PointInfo pt;
    if(fabs(j0)<1e-9){
      if(a0 > 0) j0 = max_jerk_;
      else j0 = -max_jerk_;
    }
    double max_jerk = j0;
    double acc_now = a0;
    double speed_now = v0;
    cout<<"Adding virtual point"<<endl;
    bool continue_neg_jerk = false;
    while(true){
      double time_step = getReversedTimeStep(max_jerk, acc_now, speed_now, dist_res_);
      speed_now += -acc_now*time_step + max_jerk*time_step*time_step*0.5;
      acc_now -= max_jerk*time_step;
      pt.max_speed = speed_now;
      pt.time = time_step;
      pt.acceleration = acc_now;
      pt.jerk = max_jerk;
      pt.idx = -speed_profile.size();
      pt.dist = pt.idx*dist_res_;
      //cout<<"p1 "<<pt.time<<" "<<pt.speed_profile<<" "<<pt.acceleration<<" "<<pt.jerk<<endl;
      if(a0 > 0){
	if(acc_now < 0 || acc_now > max_acc_){
	  if(acc_now > max_acc_) continue_neg_jerk = true;
	  break;
	}
      } else {
	if(acc_now > 0 || acc_now < -max_acc_){
	  if(acc_now < -max_acc_) continue_neg_jerk = true;
	  break;
	}
      }
      speed_profile.push_back(pt);
    }
    
    if(continue_neg_jerk){
      max_jerk = -j0;
      if(speed_profile.size() > 0){
	acc_now = speed_profile[speed_profile.size()-1].acceleration;
	speed_now = speed_profile[speed_profile.size()-1].speed_profile;
      }
      else {
	cout<<"Please check the initial condition. ";
	cout<<"This may occur when the combination of jerk and acceleration is not proper. Will continue for now"<<endl;
      }
      while(true){
	double time_step = getReversedTimeStep(max_jerk, acc_now, speed_now, dist_res_);
	speed_now += -acc_now*time_step + max_jerk*time_step*time_step*0.5;
	acc_now -= max_jerk*time_step;
	pt.max_speed = speed_now;
	pt.time = time_step;
	pt.acceleration = acc_now;
	pt.jerk = max_jerk;
	pt.idx = -speed_profile.size();
	pt.dist = pt.idx*dist_res_;
	//cout<<"p2 "<<pt.time<<" "<<pt.speed_profile<<" "<<pt.acceleration<<" "<<pt.jerk<<endl;
	if(a0 > 0){
	  if(acc_now < 0)
	    break;
	} else {
	  if(acc_now > 0)
	    break;
	}
	speed_profile.push_back(pt);
      }
    }
   
    if(speed_profile.size()>0){
      reverse(speed_profile.begin(), speed_profile.end());
    }
    else {
      PointInfo virtual_pt;
      virtual_pt.max_speed = v0;
      virtual_pt.idx = 0.0;
      virtual_pt.dist = 0.0;
      speed_profile.push_back(virtual_pt);
    }
    return speed_profile;
  }
  vector<PointInfo> shortAccelerationProfile(double v0, double v1, double max_jerk, double dist_res, bool debug=false){
    vector<PointInfo> speed_profile;
    double v_switch = (v0 + v1)/2.0;
    PointInfo pt;
    double acc_now = 0.0;
    double speed_now = v0;
    bool acc_acc = v0 < v1;
    while(true){
      double time_step = getTimeStep(max_jerk, acc_now, speed_now, dist_res);
      speed_now += acc_now*time_step + max_jerk*time_step*time_step*0.5;
      acc_now += max_jerk*time_step;
      pt.speed_profile = speed_now;
      pt.time = time_step;
      pt.acceleration = acc_now;
      pt.jerk = max_jerk;
      if(acc_acc){
	if(speed_now > v_switch)
	  break;
      }
      else {
	if(speed_now < v_switch)
	  break;
      }
      speed_profile.push_back(pt);
    }
    while(true){
      double time_step = getTimeStep(-max_jerk, acc_now, speed_now, dist_res);
      //due to quantization of distance, it could be a case where the time step is too huge overwhelming
      //speed now and turning into the other side of the profile, which we will have to avoid at all cost
      speed_now += acc_now*time_step - max_jerk*time_step*time_step*0.5;
      acc_now -= max_jerk*time_step;
      pt.speed_profile = speed_now;
      pt.time = time_step;
      pt.acceleration = acc_now;
      pt.jerk = -max_jerk;
      if(acc_acc){
	if(speed_now > v1)
	  break;
	if(speed_now < speed_profile[speed_profile.size()-1].speed_profile)
	  break;
      }
      else {
	if(speed_now < v1)
	  break;
	if(speed_now > speed_profile[speed_profile.size()-1].speed_profile)
	  break;
      }
      speed_profile.push_back(pt);
    }
    return speed_profile;
  }
  vector<PointInfo> dynamicAccelerationProfile(double v0, double v1, double a0, double a1, double max_jerk, double dist_res){
    vector<PointInfo> speed_profile;
    double speed_now=v0;
    PointInfo pt;
    double acc_now = a0;
    bool acc_acc = a0 < a1;
    while(true){
      double time_step = getTimeStep(max_jerk, acc_now, speed_now, dist_res);
      speed_now += acc_now*time_step + max_jerk*time_step*time_step*0.5;
      acc_now += max_jerk*time_step;
      pt.speed_profile = speed_now;
      pt.time = time_step;
      pt.acceleration = acc_now;
      pt.jerk = max_jerk;
      
      if(acc_acc) {
	if(acc_now > a1) 
	  break;
      }
      else {
	if(acc_now < a1) 
	  break;
      }
      speed_profile.push_back(pt);
    }
    return speed_profile;
  }
  vector<PointInfo> constAccelerationProfile(double v0, double v1, double max_acc, double dist_res){
    vector<PointInfo> speed_profile;
    double speed_now=v0;
    PointInfo pt;
    pt.jerk = 0.0;
    pt.acceleration = max_acc;
    bool acc_acc = v0 < v1;
    while(true){
      double time_step = getTimeStep(0.0, max_acc, speed_now, dist_res);
      speed_now += max_acc*time_step;
      pt.speed_profile = speed_now;
      pt.time = time_step;
      if(acc_acc){
	if(speed_now > v1)
	  break;
      }
      else {
	if(speed_now < v1)
	  break;
      }
      speed_profile.push_back(pt);
    }
    return speed_profile;
  }
};
tf::TransformListener* tf_;
double delay_;
string global_frame_, robot_frame_;
ros::Subscriber *cmd_steer_sub_;
ros::Publisher *cmd_vel_pub_;
geometry_msgs::Twist cmd_steer_;

bool getRobotPose(tf::Stamped<tf::Pose> &robot_pose) {
    robot_pose.setIdentity();
    tf::Stamped<tf::Pose> i_pose;
    i_pose.setIdentity();
    i_pose.frame_id_ = robot_frame_;
    i_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later
    try {
        tf_->transformPose(global_frame_, i_pose, robot_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s", ex.what());
        return false;
    }
    // check robot_pose timeout
    if (current_time.toSec() - robot_pose.stamp_.toSec() > delay_) {
        ROS_WARN("PurePursuit transform timeout. Current time: %.4f, pose(%s) stamp: %.4f, tolerance: %.4f",
                 current_time.toSec(), global_frame_.c_str(), robot_pose.stamp_.toSec(), delay_);
        return false;
    }
    return true;
 }
 
int main(int argc, char** argv){
  ros::init(argc, argv, "trajectory_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  double dist_res, max_lat_acc, max_speed, max_acc, max_jerk;
  double speed_ini, running_freq;
  tf_ = new tf::TransformListener();
  priv_nh.param("dist_res", dist_res, 0.05);
  priv_nh.param("max_lat_acc", max_lat_acc, 1.0);
  priv_nh.param("max_speed", max_speed, 5.0);
  priv_nh.param("max_acc", max_acc, 0.5);
  priv_nh.param("max_jerk", max_jerk, 0.5);
  priv_nh.param("speed_ini", speed_ini, 0.0);
  priv_nh.param("running_freq", running_freq, 25.0);
  priv_nh.param("global_frame", global_frame_, string("robot_0/map"));
  priv_nh.param("robot_frame", robot_frame_, string("robot_0/base_link"));
  priv_nh.param("max_pose_delay", delay_, 0.2);
  geometry_msgs::Pose robot_pose;
  TrajectorySimulator ts(max_lat_acc, max_speed, max_acc, max_jerk, dist_res,nh);
  
  vector<PointInfo> info = ts.getSpeedProfile();
  
  ros::Rate r(running_freq);
  double sleep_time = 1.0/running_freq;
  double speed_now= speed_ini;
  double acc_ini = 0.0, acc_now=0.0;
  ros::spin();
//   while(ros::ok()){
//     tf::Stamped<tf::Pose> robot_pose_tf;
//     if(getRobotPose(robot_pose_tf)){
//       geometry_msgs::PoseStamped robot_pose_msg;
//       tf::poseStampedTFToMsg(robot_pose_tf, robot_pose_msg);
//       robot_pose = robot_pose_msg.pose;
//     }
//     int nearest_idx = 0;
//     double nearest_dist = 1e9;
//     for(size_t i=0; i<info.size(); i++){
//       double x = info[i].position.x - robot_pose.position.x;
//       double y = info[i].position.y - robot_pose.position.y;
//       double dist = sqrt(x*x + y*y);
//       if(dist < nearest_dist){
// 	nearest_idx = i;
// 	nearest_dist = dist;
//       }
//     }
//     //cout<<"Advised speed = "<<info[nearest_idx].speed_profile<<" at "<<nearest_idx<<" found at dist "<<nearest_dist<<endl;
//     double advised_speed = info[nearest_idx].speed_profile;
//     /*double jerk = info[nearest_idx].jerk;
//     acc_now = acc_ini + jerk * sleep_time;
//     speed_now = speed_ini + acc_ini * sleep_time + 0.5 * jerk * sleep_time * sleep_time;
//     
//     cout<<"SAJ: "<<speed_now<<" "<<acc_now<<" "<<jerk<<endl;*/
//     cout<<advised_speed<<endl;
//     acc_ini = acc_now;
//     speed_ini = speed_now;
//     ros::spinOnce();
//     r.sleep();
//   }
  
  return 0;
}