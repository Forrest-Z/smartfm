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
  double dist_res_;
  double max_acc_, max_jerk_, max_speed_;
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
  
  
  void publishPathInfo(vector<PointInfo> &path_info, vector<PointInfo> &local_minima_pts){
    sensor_msgs::PointCloud pc;
    ros::Publisher pub, curvature_pub, dist_pub;
    ros::Publisher max_speed_pub, speed_pub, speed_xy_pub, acc_pub, jerk_pub;
    ros::NodeHandle nh;
    
    curvature_pub = nh.advertise<sensor_msgs::PointCloud>("curvature_pt", 1, true);
    max_speed_pub = nh.advertise<sensor_msgs::PointCloud>("max_speed_pt", 1, true);
    speed_pub = nh.advertise<sensor_msgs::PointCloud>("speed_pt", 1, true);
    speed_xy_pub = nh.advertise<sensor_msgs::PointCloud>("speed_xy_pt", 1, true);
    acc_pub = nh.advertise<sensor_msgs::PointCloud>("acc_pt", 1, true);
    jerk_pub = nh.advertise<sensor_msgs::PointCloud>("jerk_pt", 1, true);
    dist_pub = nh.advertise<sensor_msgs::PointCloud>("dist_pt", 1, true);
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
    ros::Publisher local_min_pub = nh.advertise<sensor_msgs::PointCloud>("local_min_puts", 1, true);
    for(size_t i=0; i<local_minima_pts.size(); i++){
      geometry_msgs::Point32 p;
      p.x = speed_pc.points[local_minima_pts[i].idx].x;
      p.y = local_minima_pts[i].max_speed+=100;
      p.z = local_minima_pts[i].max_speed;
      local_pc.points.push_back(p);
    }
    local_min_pub.publish(local_pc);
    ros::spin();
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
  
  
  TrajectorySimulator(int argc, char** argv){
    ros::init(argc, argv, "trajectory_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    double min_look_ahead_dist = 4.0;
    double forward_achor_pt_dist = 1.0;
    double car_length = 2.55;
    double dist_res, max_lat_acc, max_speed, max_acc, max_jerk;
    double acc_ini, speed_ini, max_sim_length;
    priv_nh.param("dist_res", dist_res, 0.05);
    priv_nh.param("max_lat_acc", max_lat_acc, 1.0);
    priv_nh.param("max_speed", max_speed, 5.0);
    priv_nh.param("max_acc", max_acc, 0.5);
    priv_nh.param("max_jerk", max_jerk, 0.5);
    priv_nh.param("acc_ini", acc_ini, 0.0);
    priv_nh.param("speed_ini", speed_ini, 0.0);
    priv_nh.param("max_sim_length", max_sim_length, 50.0);
    max_acc_ = max_acc;
    max_jerk_ = max_jerk;
    max_speed_ = max_speed;
    dist_res_ = dist_res;
    global_frame_ = "/robot_0/map";
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    ros::Publisher global_path_pub;
    global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 1, true);
    
    setGlobalPath(global_path_pub);
    
    vector<PointInfo> path_info;
    
    geometry_msgs::Point first_pt = pp_->path_.poses[0].pose.position;
    geometry_msgs::Point sec_pt = pp_->path_.poses[1].pose.position;
    double car_init_orientation = atan2(sec_pt.y-first_pt.y, sec_pt.x-first_pt.x);
    CarModel model(car_length, dist_res, poseVec(pp_-> path_.poses[0].pose.position.x, 
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
    PointInfo initial_point;
    initial_point.position = initial_pose.position;
    initial_point.dist = 0.0;
    initial_point.max_speed = max_speed;
    initial_point.curve_max_speed = max_speed;
    initial_point.idx = path_no++;
    path_info.push_back(initial_point);
    fmutil::Stopwatch steering_sw("steering", false);
    size_t total_max_path = max_sim_length/dist_res;
    while(path_exist && ros::ok()){
      double steer_angle, dist_to_goal;
      steering_sw.start();
      path_exist = pp_->steering_control(&steer_angle, &dist_to_goal);
      steering_sw.end(false);
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
      point_info.max_speed = sqrt(max_lat_acc*turning_rad);
      point_info.idx = path_no++;
      if(point_info.max_speed > max_speed) point_info.max_speed = max_speed;
      point_info.curve_max_speed = point_info.max_speed;
      point_info.time = dist_res/max_speed;
      path_info.push_back(point_info);
      if(path_info.size() > total_max_path) break;
    }
    cout<<steering_sw.total_/1000<<" ms"<<endl;
    cout<<path_info.size()<<" point size"<<endl;
    vector<PointInfo> local_minima_pts = localMinimaSearch(path_info);
    path_info[0].max_speed = speed_ini;
    path_info[path_info.size()-1].max_speed = 0.0;
    local_minima_pts.push_back(path_info[0]);
    local_minima_pts.push_back(path_info[path_info.size()-1]);
    sort(local_minima_pts.begin(), local_minima_pts.end());
    printLocalMinimaStatus("after sort 2", local_minima_pts);
    fmutil::Stopwatch sw2("connecting points");
    
    for(size_t i=1; i<local_minima_pts.size(); i++){
      double v0 = local_minima_pts[i-1].max_speed;
      double v1 = local_minima_pts[i].max_speed;
      if(fabs(v1-v0)<1e-5){
	//trivial case just ok  
	cout<<i<<": Same speed OK!"<<endl;
	double constant_time_step = dist_res/local_minima_pts[i].max_speed;
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
	if(speed_check > max_speed) speed_check = max_speed;
	cout<<i<<": Speed check v0 "<<v0<<" v1 "<<v1<<" dist "<<dist<<" suggested speed "<<speed_check;
	double full_jerk_dist = getMinDistFullProfile(v0, speed_check);
	int full_jerk_idx = full_jerk_dist/dist_res+local_minima_pts[i-1].idx;
	double full_jerk_max_speed = path_info[full_jerk_idx].max_speed;
	cout<<" full jerk speed "<<full_jerk_max_speed<<" @ full_jerk_idx"<<full_jerk_idx<<" ";
	if(full_jerk_max_speed>=speed_check && speed_check > v0 && speed_check > v1){
	  cout<<" OK! "<<endl;
	  int start_idx = local_minima_pts[i-1].idx;
	  int end_idx = local_minima_pts[i].idx;
	  double req_speed_inc = max_acc*max_acc/max_jerk;
	  vector<PointInfo> acc_profile;
	  if(speed_check-v0 > req_speed_inc) {
	    acc_profile = completeAccelerationProfile(v0, speed_check);
	    cout<<"Size of acc_profile = "<<acc_profile.size()<<" "<<start_idx<<endl;
	  }
	  else {
	    //add acceleration profile for short speed diff
	    acc_profile = shortAccelerationProfile(v0, speed_check, max_jerk, dist_res);
	    cout<<"Need acceleration profile for short speed diff "<<acc_profile.size()<<endl;
	  }
	  for(size_t j=0; j<acc_profile.size(); j++, start_idx++){
	    path_info[start_idx].copy(acc_profile[j]);
	  }
	  vector<PointInfo> dec_profile;
	  if(speed_check-v1 > req_speed_inc){
	    dec_profile = completeDeccelerationProfile(speed_check, v1);
	    cout<<"Size of dec_profile = "<<dec_profile.size()<<" "<<end_idx<<endl;
	  }
	  else {
	    dec_profile = shortAccelerationProfile(speed_check, v1, -max_jerk, dist_res);
	    cout<<"Need decceleration profile for short speed diff "<<dec_profile.size()<<endl;
	  }
	  for(int j=(int)dec_profile.size()-1; j>=0; j--, end_idx--){
	    path_info[end_idx].copy(dec_profile[j]);
	  }
	  PointInfo constant_speed;
	  constant_speed.speed_profile = speed_check;
	  constant_speed.time = dist_res_/speed_check;
	  for(int j=start_idx; j<=end_idx; j++)
	    path_info[j].copy(constant_speed);
	}
	else {
	  int start_idx = local_minima_pts[i-1].idx;
	  int end_idx = local_minima_pts[i].idx;
	  double req_speed_inc = max_acc*max_acc/max_jerk;
	  if(v0<v1){
	    double single_profile_dist = getMinDistFullProfile(v0, v1);
	    double given_dist = local_minima_pts[i].dist - local_minima_pts[i-1].dist;
	    vector<PointInfo> acc_single_profile;
	    cout<<" given_dist vs single_profile_dist "<<given_dist<<" "<<single_profile_dist<<" "<<given_dist-single_profile_dist;
	    if(given_dist >= single_profile_dist || fabs(given_dist-single_profile_dist)<1e-9){
	      cout<<" Acc single"<<endl;
	      if(v1 - v0 > req_speed_inc){
		acc_single_profile = completeAccelerationProfile(v0, v1);
	      }
	      else {
		acc_single_profile = shortAccelerationProfile(v0, v1, max_jerk, dist_res);
	      }
	      
	    }
	    else {
	      /*single_profile_dist = getMinDist(v0, v1);
	      cout<<" single_profile_dist "<<single_profile_dist<<" "<<given_dist<<" ";
	      if(fabs(single_profile_dist - given_dist)<1e-5 || single_profile_dist<given_dist){
		acc_single_profile = shortAccelerationProfile(v0, v1, max_jerk, dist_res);
		cout <<" Ok prepare for single short acc profile "<<given_dist<<" "<<single_profile_dist<<" "<<acc_single_profile.size()<<endl;
	      }
	      else {*/
		double new_speed = getNewSpeedShortProfile(v0,v1, given_dist);
		//if(new_speed < local_minima_pts[i].max_speed)
		cout<<"Reversing at acc at new speed "<<new_speed<<endl;
		local_minima_pts[i].max_speed = new_speed;
		i-=2;
	      //}
	    }
	    cout <<" Ok prepare for single acc profile "<<given_dist<<" "<<single_profile_dist<<" "<<acc_single_profile.size()*dist_res<<endl;  
	    for(size_t j=0; j<acc_single_profile.size(); j++, start_idx++){
	      path_info[start_idx].copy(acc_single_profile[j]);
	    }
	    PointInfo constant_speed;
	    constant_speed.speed_profile = v1;
	    constant_speed.time = dist_res_/v1;
	    for(int j=start_idx; j<=end_idx; j++)
	      path_info[j].copy(constant_speed);
	  }
	  else{
	    double single_profile_dist = getMinDistFullProfile(v1, v0);
	    double given_dist = local_minima_pts[i].dist - local_minima_pts[i-1].dist;
	    vector<PointInfo> dec_single_profile;
	    cout<<" given_dist vs single_profile_dist "<<given_dist<<" "<<single_profile_dist<<" "<<given_dist-single_profile_dist;
	    if(given_dist >= single_profile_dist || fabs(given_dist-single_profile_dist)<1e-9){
	      cout<<" Dec single"<<endl;
	      if(v0 - v1 > req_speed_inc){
		dec_single_profile = completeDeccelerationProfile(v0, v1);
	      }
	      else {
		dec_single_profile = shortAccelerationProfile(v0, v1, -max_jerk, dist_res);
	      }
	    }
	    else {
	      /*single_profile_dist = getMinDist(v1, v0);
	      cout<<" single_profile_dist "<<single_profile_dist<<" "<<given_dist<<" ";
	      if(fabs(single_profile_dist - given_dist)<1e-5 || single_profile_dist<given_dist) {
		cout <<" Ok prepare for single short dec profile "<<given_dist<<" "<<single_profile_dist<<" v0 "<<v0<<" v1 "<<v1<<endl;
		dec_single_profile = shortAccelerationProfile(v0, v1, -max_jerk, dist_res, true);
	      }
	      else {*/
		double new_speed = getNewSpeedShortProfile(v1,v0, given_dist);
		//if(new_speed < local_minima_pts[i-1].max_speed)
		cout<<"Reversing at dec new max speed "<<new_speed<<endl;
		  local_minima_pts[i-1].max_speed = new_speed;
		i-=2;
	      //}
	    }
	    cout <<" Ok prepare for single dec profile "<<given_dist<<" "<<single_profile_dist<<" "<<dec_single_profile.size()*dist_res<<endl;
	    for(int j=(int)dec_single_profile.size()-1; j>=0; j--, end_idx--){
	      path_info[end_idx].copy(dec_single_profile[j]);
	    }
	    PointInfo constant_speed;
	    constant_speed.speed_profile = v0;
	    constant_speed.time = dist_res_/v0;
	    for(int j=start_idx; j<=end_idx; j++)
	      path_info[j].copy(constant_speed);
	  }
	}
      }
    }  
    sw2.end();
    sw.end();
    publishPathInfo(path_info, local_minima_pts);
  }
  double getNewSpeedShortProfile(double low_speed, double high_speed, double dist){
    
    double suggest_speed_1 = solveCubic(0.0, 0.5/max_acc_, 0.5*max_acc_/max_jerk_, 
					0.5*low_speed/max_jerk_ - 0.5*low_speed*low_speed/max_acc_-dist);
    //just don't deal with speed that is smaller
    if(suggest_speed_1 - low_speed < max_acc_*max_acc_/max_jerk_)
      suggest_speed_1 = low_speed;
    //cout<<"Not implemented yet!! suggested "<<suggest_speed_1<<" "<<min_dist<<" given "<<high_speed<<" "<<dist<<endl;
    return suggest_speed_1;
    //this also not respect maximum acceleration
    //return solveCubic(1.0, v0, -v0*v0, -pow(v0, 3)-max_jerk_*dist*dist);
    
  }
  double getMinDist(double v0, double v1){
    //this function will not respect maximum acceleration
    return (v1+v0)*sqrt((v1-v0)/max_jerk_);
  }
  double getMinDistFullProfile(double v0, double v1){
    double min_dist;
      //wrong equation given by the paper
      //min_dist = v1/2.0*((v1-v0)/max_acc_+max_acc_/max_jerk_);
      min_dist = (v1+v0)*max_acc_/max_jerk_+v1*(v1/(2*max_acc_)-max_acc_/(2*max_jerk_))
      -v0*(v0/(2*max_acc_)+max_acc_/(2*max_jerk_));
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


int main(int argc, char** argv){
  TrajectorySimulator ts(argc, argv);
  return 0;
}