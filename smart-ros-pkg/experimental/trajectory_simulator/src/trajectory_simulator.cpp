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
  PointInfo(): verified_ok(false){};
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
			   double &acc_pt_pre, double &speed_pt_pre, int &last_acceleration_idx,
			   bool speed_limit = false, double v_switch = 0.0
 				){
    cout<<"Start idx "<<last_acceleration_idx<<" end idx "<<end<<" acc "<<acc_pt_pre<<" speed "<<speed_pt_pre<<endl;
    for(int k=last_acceleration_idx; k<=end; k++){
      double time_step = getTimeStep(max_jerk, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre + max_jerk*time_step;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step + max_jerk*pow(time_step,2)*0.5;
      if(max_jerk>0){
      if(acc_now >= max_acc_ || (speed_limit && speed_now > v_switch)) {
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
  
  void dynamicDecclerationProfile(int end, double max_jerk, vector<PointInfo> &path_info,
				  double &acc_pt_pre, double &speed_pt_pre, int &last_decceleration_idx,
				  bool speed_limit = false, double v_switch = 0.0
 				){
    cout<<"Start idx "<<last_decceleration_idx<<" end idx "<<end<<" acc "<<acc_pt_pre<<" speed "<<speed_pt_pre<<endl;
    for(int k=last_decceleration_idx; k<=end; k++){
      double time_step = getTimeStep(max_jerk, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre + max_jerk*time_step;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step + max_jerk*pow(time_step,2)*0.5;
      if(max_jerk<0){
	if(acc_now <=-max_acc_ || (speed_limit && speed_now < v_switch)){
	  last_decceleration_idx = k;
	  break;
	}
      }
      else {
	if(acc_now >=0){
	  last_decceleration_idx = k;
	  break;
	}
      }
      path_info[k].speed_profile = speed_now;
      path_info[k].time = time_step;
      path_info[k].acceleration = acc_now;
      path_info[k].jerk = max_jerk;
      acc_pt_pre = acc_now;
      speed_pt_pre = speed_now;
      //cout<<speed_now<<" "<<acc_now<<" "<<max_jerk<<endl;
    }
  }
  
  void constAccelerationProfile(int end, double v2, vector<PointInfo> &path_info, 
			   double &acc_pt_pre, double &speed_pt_pre, int &last_acceleration_idx){
    cout<<"Start idx "<<last_acceleration_idx<<" end idx "<<end<<" acc "<<acc_pt_pre<<" speed "<<speed_pt_pre<<endl;
    bool brake_normal = false;
    for(int k=last_acceleration_idx; k<=end; k++){
      double time_step = getTimeStep(0.0, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step;
      //cout<<"2: "<<time_step<<" "<<speed_now<<" "<<acc_pt_pre<<" "<<v2<<endl;
      if(speed_now >=v2){
	last_acceleration_idx = k;
	cout<<"Brake away because "<<speed_now<<" >= "<<v2<<endl;
	brake_normal = true;
	break;
      }
      path_info[k].speed_profile = speed_now;
      path_info[k].time = time_step;
      path_info[k].acceleration = acc_now;
      path_info[k].jerk = 0.0;
      acc_pt_pre = acc_now;
      speed_pt_pre = speed_now;
      
    }
    if(!brake_normal)
      cout<<"Const acceleration profile ended prematurely. Please check!!"<<endl;
  }
  
    void constDeccelerationProfile(int end, double v2, vector<PointInfo> &path_info, 
			   double &acc_pt_pre, double &speed_pt_pre, int &last_acceleration_idx){
    bool brake_normal = false;
    cout<<"Start idx "<<last_acceleration_idx<<" end idx "<<end<<" acc "<<acc_pt_pre<<" speed "<<speed_pt_pre<<endl;
    for(int k=last_acceleration_idx; k<=end; k++){
      double time_step = getTimeStep(0.0, acc_pt_pre, speed_pt_pre, 0.05);
      double acc_now = acc_pt_pre;
      double speed_now = speed_pt_pre + acc_pt_pre*time_step;
      //cout<<speed_now<<" "<<v2<<endl;
      if(speed_now <=v2){
	last_acceleration_idx = k;
	brake_normal = true;
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
    if(!brake_normal)
      cout<<"Const decceleration profile ended prematurely. Please check!!"<<endl;
  }
  
  double getMinDistFullProfile(double v0, double v1){
    double min_dist;
      //wrong equation given by the paper
      //min_dist = v1/2.0*((v1-v0)/max_acc_+max_acc_/max_jerk_);
      min_dist = (v1+v0)*max_acc_/max_jerk_+v1*(v1/(2*max_acc_)-max_acc_/(2*max_jerk_))
      -v0*(v0/(2*max_acc_)+max_acc_/(2*max_jerk_));
    return min_dist;
  }
  
  double getMinDist(double v0, double v1){
    return (v1+v0)*sqrt((v1-v0)/max_jerk_);
  }
  
  double getNewMaxSpeed(double distance, double low_speed){
    double new_speed_1 = solveCubic(1.0, 2*low_speed, -(low_speed*low_speed)-low_speed, 
				    -pow(low_speed, 3) - max_jerk_*distance*distance);
    double new_speed_2 = solveCubic(0.0, 1/(2*max_acc_), -low_speed/(2*max_acc_)+max_acc_/(2*max_jerk_), -distance);
    cout<<"New speed "<<new_speed_1<<" "<<new_speed_2<<" given "<<distance<<" "<<low_speed;
    /*if(new_speed_1 - low_speed < max_acc_*max_acc_/max_jerk_) {
      cout<<" speed1 selected"<<endl;
      return new_speed_1;
    }
    cout<<" speed2 selected"<<endl;
    cout<<" recheck: "<<new_speed_2/2*((new_speed_2-low_speed)/max_acc_ + max_acc_/max_jerk_)<<endl;
    */
    
    return new_speed_2>new_speed_1? new_speed_1 : new_speed_2;
  }
  
  inline bool determineAcceleration(double v0_in, double v1_in){
    double v0, v1;
    return determineAcceleration(v0_in, v1_in, v0, v1);
  }
  inline bool determineAcceleration(double v0_in, double v1_in, double &v0_out, double &v1_out){
    bool acceleration = v1_in > v0_in;
    if(acceleration){
      v0_out = v0_in;
      v1_out = v1_in;
    }
    else {
      v0_out = v1_in;
      v1_out = v0_in;
    }
    return acceleration;
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
  
  int verifyAndModifyLocalMinima(vector<PointInfo> &local_minima_pts){
    
    //final check and combine indices that are closely pack together
    //important functions
	//double getMinDist(v0, v1)
	//double getNewMaxSpeed(dist, initial_speed)
	//bool determineAcceleration(v0_in, v1_in, v0_out[opt], v1_out[opt])
    //By default, assume first pt is verified
    local_minima_pts[0].verified_ok = true;
    int modification_no = 0;
    for(size_t i=1; i<local_minima_pts.size(); i++){
      
      bool is_acc = determineAcceleration(local_minima_pts[i].max_speed, 
					  local_minima_pts[i-1].max_speed);
      int local_min_pts_idx = i;
      cout<<"Evaluating local minima pt "<<i<<endl;
      while(local_min_pts_idx > 0){
	if(!local_minima_pts[local_min_pts_idx-1].verified_ok) {
	  local_min_pts_idx--;
	  continue;
	}
	double v0, v1;
	bool is_acc_pre = determineAcceleration(local_minima_pts[local_min_pts_idx].max_speed, 
					  local_minima_pts[local_min_pts_idx-1].max_speed);
	if(is_acc_pre){
	  v1 = local_minima_pts[local_min_pts_idx-1].max_speed;
	  v0 = local_minima_pts[i].max_speed;
	}
	else {
	  v1 = local_minima_pts[i].max_speed;
	  v0 = local_minima_pts[local_min_pts_idx-1].max_speed;
	}
	cout<<"\t"<<local_min_pts_idx<<": max_speed "<<v0<<" "<<v1<<" "<<is_acc_pre<<" "<<is_acc<<endl;
	//only proceed if previous local min has the same acceleration state as the current one
	//if not have to reconsider the maximum speed of this previous local min
	if(is_acc_pre == is_acc){
	  double min_dist = getMinDist(v0, v1);
	  local_minima_pts[i].min_dist = min_dist;
	  if(min_dist <= local_minima_pts[i].dist - local_minima_pts[local_min_pts_idx-1].dist){
	    local_minima_pts[i].verified_ok = true;
	    break;
	  }
	  else {
	    local_minima_pts[local_min_pts_idx-1].verified_ok = false;
	    local_min_pts_idx--;
	    modification_no++;
	  }
	}
	else{
	  cout<<"Need a new speed on "<<local_minima_pts[i].max_speed<<" "<<local_minima_pts[local_min_pts_idx].max_speed<<endl;
	  double the_dist = local_minima_pts[i].dist - local_minima_pts[local_min_pts_idx].dist;
	  double set_speed, new_speed;
	  if(is_acc)
	    set_speed = local_minima_pts[i].max_speed;
	  else
	    set_speed = local_minima_pts[local_min_pts_idx].max_speed;
	  new_speed = getNewMaxSpeed(the_dist, set_speed);
	  if(new_speed > max_speed_) new_speed = max_speed_;
	  cout<<"New speed obtained: "<<new_speed<<endl;
	  if(is_acc)
	    local_minima_pts[i].max_speed = new_speed;
	  else
	    local_minima_pts[local_min_pts_idx].max_speed = new_speed;
	  local_minima_pts[local_min_pts_idx].verified_ok = true;
	  local_minima_pts[i].verified_ok = true;
	  modification_no++;
	  break;
	}
      }
    }
    return modification_no;
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
  
  void mainFunction(vector<PointInfo> &local_minima_pts, vector<PointInfo> &path_info, double &max_speed, double &max_acc, double &max_jerk){
    fmutil::Stopwatch sw2("connecting points");
    double req_speed_inc = max_acc*max_acc/max_jerk;
    for(size_t i=1; i<local_minima_pts.size(); i++){
      //if 2 local minima happens at very near each other, the one have the lower speed wins
      if(local_minima_pts[i].idx - local_minima_pts[i-1].idx <= 5){
	if(local_minima_pts[i].max_speed < local_minima_pts[i-1].max_speed)
	  local_minima_pts[i-1].max_speed = local_minima_pts[i].max_speed;
	else
	  local_minima_pts[i].max_speed = local_minima_pts[i-1].max_speed;
      }
      double v0 = local_minima_pts[i-1].max_speed;
      double v1 = local_minima_pts[i].max_speed;
      if(fabs(v1-v0)<1e-5){
	//last couple of points are zero, make sure this don't happen
	//so the vehicle will stop at the very end
	/*if(v1 < 1e-5 && i == local_minima_pts.size()-1){
	  local_minima_pts.erase(local_minima_pts.begin()+i-1);
	  i-=2;
	}*/
	//trivial case just ok  
	cout<<i<<": Same speed OK!"<<endl;
	double constant_time_step = 0.05/local_minima_pts[i].max_speed;
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
	if(speed_check > v0 && speed_check > v1){
	  cout<<" OK! ";
	  //double check
	  double min_dist_a = getMinDistFullProfile(v0, speed_check);
	  double min_dist_b = getMinDistFullProfile(v1, speed_check);
	  int path_info_idx = local_minima_pts[i].idx;
	  double v_start, v_end;
	  double end = local_minima_pts[i].idx;
	  double acc_pt_pre = 0.0;
	  int last_acceleration_idx;
	  last_acceleration_idx = local_minima_pts[i-1].idx;
	  //acceleration is guranteed then diccelerate
	  if(speed_check-v0 > req_speed_inc){
	    v_start = v0 + req_speed_inc/2.0;
	    v_end = speed_check - req_speed_inc/2.0;
	    dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
	    double local_max_acc = max_acc;
	    constAccelerationProfile(end, v_end, path_info, local_max_acc, v0, last_acceleration_idx);
	    local_max_acc = max_acc;
	    dynamicAccelerationProfile(end, -max_jerk, path_info, local_max_acc, v0, last_acceleration_idx);
	  }
	  else {
	    double delta_t = sqrt(fabs(speed_check-v0)/max_jerk);
	    double v_switch = v0 + 0.5*max_jerk*delta_t*delta_t;
	    bool speed_limit = true;
	    dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx, speed_limit, v_switch);
	    dynamicAccelerationProfile(end, -max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
	  }
	  acc_pt_pre = 0.0;
	  int dist_buffer = min_dist_b/0.05 + 10;
	  vector<PointInfo> path_info_temp;
	  path_info_temp.resize(dist_buffer);
	  int last_dec_idx = 0;
	  if(speed_check-v1 > req_speed_inc){
	    v_start = v1 + req_speed_inc/2.0;
	    double max_dec = -max_acc;
	    double start_dec_speed = speed_check;
	    dynamicDecclerationProfile(dist_buffer, -max_jerk, path_info_temp, acc_pt_pre, start_dec_speed, last_dec_idx);
	    max_dec = -max_acc;
	    constDeccelerationProfile(dist_buffer, v_start, path_info_temp, max_dec, start_dec_speed, last_dec_idx);
	    max_dec = -max_acc;
	    dynamicDecclerationProfile(dist_buffer, max_jerk, path_info_temp, max_dec, start_dec_speed, last_dec_idx);
	  }
	  else {
	    double delta_t = sqrt(fabs(speed_check-v1)/max_jerk);
	    double v_switch = v1 + 0.5*max_jerk*delta_t*delta_t;
	    bool speed_limit = true;
	    dynamicDecclerationProfile(end, -max_jerk, path_info_temp, acc_pt_pre, v0, last_dec_idx, speed_limit, v_switch);
	    dynamicDecclerationProfile(end, max_jerk, path_info_temp, acc_pt_pre, v0, last_dec_idx);
	  }
	  for(int j=last_dec_idx; j>=0; j--, path_info_idx--){
	    path_info[path_info_idx].speed_profile = path_info_temp[j].speed_profile;
	    path_info[path_info_idx].jerk = path_info_temp[j].jerk;
	    path_info[path_info_idx].acceleration = path_info_temp[j].acceleration;
	    path_info[path_info_idx].time = path_info_temp[j].time;
	  }
	  double constant_time_step = 0.05/speed_check;
	  for(int j=last_acceleration_idx; j<=path_info_idx; j++){
	    path_info[j].speed_profile = speed_check;
	    path_info[j].jerk = 0.0;
	    path_info[j].acceleration = 0.0;
	    path_info[j].time = constant_time_step;
	  }
	}
	else {
	  if(fabs(v1-v0) >= max_acc*max_acc/max_jerk){
	    cout<<endl<<"    Hmm, let's see other alternatives"<<endl;
	    cout<<"      How about direct acceleration ";
	    double min_dist_alt;
	    if(v1>v0)
	      min_dist_alt= getMinDistFullProfile(v0, v1);
	    else
	      min_dist_alt= getMinDistFullProfile(v1, v0);
	    cout<<min_dist_alt<<" < "<<dist<<"? ";
	    if(min_dist_alt < dist) {
		double end = local_minima_pts[i].idx;
		int last_acceleration_idx;
		last_acceleration_idx = local_minima_pts[i-1].idx;
	      if(v1>v0){
		double v_start, v_end;
		double acc_pt_pre = 0.0;
		v_start = v0 + req_speed_inc/2.0;
		v_end = v1 - req_speed_inc/2.0;
		dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
		double local_max_acc = max_acc;
		constAccelerationProfile(end, v_end, path_info, local_max_acc, v0, last_acceleration_idx);
		local_max_acc = max_acc;
		dynamicAccelerationProfile(end, -max_jerk, path_info, local_max_acc, v0, last_acceleration_idx);
	      }
	      else {
		double v_start = v1 + req_speed_inc/2.0;
		double acc_pt_pre = 0.0;
		double max_dec = -max_acc;
		double start_dec_speed = v0;
		dynamicDecclerationProfile(end, -max_jerk, path_info, acc_pt_pre, start_dec_speed, last_acceleration_idx);
		max_dec = -max_acc;
		constDeccelerationProfile(end, v_start, path_info, max_dec, start_dec_speed, last_acceleration_idx);
		max_dec = -max_acc;
		dynamicDecclerationProfile(end, max_jerk, path_info, max_dec, start_dec_speed, last_acceleration_idx);
	      }
	      double constant_time_step = 0.05/v1;
	      for(int j=last_acceleration_idx; j<=local_minima_pts[i].idx; j++){
		path_info[j].speed_profile = v1;
		path_info[j].jerk = 0.0;
		path_info[j].acceleration = 0.0;
		path_info[j].time = constant_time_step;
	      } 
	    }
	    else {
	      double min_dist_alt;
	      cout<<"Nah1!"<<endl<<"    Hmm, alright, how about this?"<<endl;
	      cout<<"      velocity diff to small criteria ";
	      if(v1>v0)
		min_dist_alt = getMinDist(v0, v1);
	      else
		min_dist_alt = getMinDist(v1, v0);
	      cout<<min_dist_alt<<" < "<<dist<<"? ";
	      if(dist-min_dist_alt >= -1e-6) {
		cout<<" OK!"<<endl;
		double delta_t = sqrt(fabs(v1-v0)/max_jerk);
		int end = local_minima_pts[i].idx;
		double acc_pt_pre = 0.0;
		int last_acceleration_idx = local_minima_pts[i-1].idx;
		bool speed_limit = true;
		if(v1>v0) {
		  double v_switch = v0 + 0.5*max_jerk*delta_t*delta_t;
		  dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx, speed_limit, v_switch);
		  dynamicAccelerationProfile(end, -max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
		}
		else {
		  double v_switch = v1 + 0.5*max_jerk*delta_t*delta_t;
		  dynamicDecclerationProfile(end, -max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx, speed_limit, v_switch);
		  dynamicDecclerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
		}
		double constant_time_step = 0.05/v1;
		for(int j=last_acceleration_idx; j<=local_minima_pts[i].idx; j++){
		  path_info[j].speed_profile = v1;
		  path_info[j].jerk = 0.0;
		  path_info[j].acceleration = 0.0;
		  path_info[j].time = constant_time_step;
		} 
	      }
	      else {
		cout<<" Nah2! Got to look backward"<<endl;
		//final resort to changing the higher speed to lower one
		/*int new_idx = min_dist_alt/0.05 - 1;
		local_minima_pts[i-1].idx -= new_idx;
		local_minima_pts[i-1].dist = local_minima_pts[i-1].idx * 0.05;
		int j=1;
		while(local_minima_pts[i-j-1].idx > local_minima_pts[i-j].idx){
		  cout<<local_minima_pts[i-j-1].idx<<" > "<<local_minima_pts[i-j].idx<<endl;
		  cout<<"Earlier idx is larger, need to change!!!"<<endl;
		  local_minima_pts[i-j-1].idx = local_minima_pts[i-j].idx-1;
		  local_minima_pts[i-j-1].dist = local_minima_pts[i-j-1].idx * 0.05;
		  j++;
		  i--;
		}*/
		if(v1>v0){
		  local_minima_pts[i].max_speed = getNewMaxSpeed(dist, v0);
		  cout<<"Possible new speed "<<getNewMaxSpeed(dist, v0)<<endl;
		}
		else {
		  local_minima_pts[i-1].max_speed = getNewMaxSpeed(dist, v1);
		  cout<<"Possible new speed "<<getNewMaxSpeed(dist, v1)<<endl;
		}
		i-=2;
	      }
	    }
	  }
	  else {
	    double min_dist_alt;
	    cout<<"Nah3!"<<endl<<"    Hmm, alright, how about this?"<<endl;
	    cout<<"      velocity diff to small criteria ";
	    if(v1>v0)
	      min_dist_alt = getMinDist(v0, v1);
	    else
	      min_dist_alt = getMinDist(v1, v0);
	    cout<<min_dist_alt<<" < "<<dist<<" dist-min_dist_alt "<<dist-min_dist_alt<<"? ";
	    if(dist - min_dist_alt >= -1e-6) {
	      cout<<" OK!"<<endl;
	      double delta_t = sqrt(fabs(v1-v0)/max_jerk);
	      int end = local_minima_pts[i].idx;
	      double acc_pt_pre = 0.0;
	      int last_acceleration_idx = local_minima_pts[i-1].idx;
	      bool speed_limit = true;
	      if(v1>v0) {
		double v_switch = v0 + 0.5*max_jerk*delta_t*delta_t;
		dynamicAccelerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx, speed_limit, v_switch);
		dynamicAccelerationProfile(end, -max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
	      }
	      else {
		double v_switch = v1 + 0.5*max_jerk*delta_t*delta_t;
		dynamicDecclerationProfile(end, -max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx, speed_limit, v_switch);
		dynamicDecclerationProfile(end, max_jerk, path_info, acc_pt_pre, v0, last_acceleration_idx);
	      }
	      double constant_time_step = 0.05/v1;
	      for(int j=last_acceleration_idx; j<=local_minima_pts[i].idx; j++){
		path_info[j].speed_profile = v1;
		path_info[j].jerk = 0.0;
		path_info[j].acceleration = 0.0;
		path_info[j].time = constant_time_step;
	      } 
	    }
	    else {
	      cout<<" Nah4! Got to look backward"<<endl;
	      //final resort to changing the higher speed to lower one
	      /*
	      int new_idx = min_dist_alt/0.05 - 1;
	      local_minima_pts[i-1].idx -= new_idx;
	      local_minima_pts[i-1].dist = local_minima_pts[i-1].idx  * 0.05;
	      int j=1;
	      cout<<local_minima_pts[i-j-1].idx<<" > "<<local_minima_pts[i-j].idx<<endl;
	      while(local_minima_pts[i-j-1].idx > local_minima_pts[i-j].idx){
		cout<<local_minima_pts[i-j-1].idx<<" > "<<local_minima_pts[i-j].idx<<endl;
		cout<<"Earlier idx is larger, need to change!!!"<<endl;
		local_minima_pts[i-j-1].idx = local_minima_pts[i-j].idx-1;
		local_minima_pts[i-j-1].dist = local_minima_pts[i-j-1].idx * 0.05;
		j++;
		i--;
	      }*/
		if(v1>v0){
		  local_minima_pts[i].max_speed = getNewMaxSpeed(dist, v0);
		  cout<<"Possible new speed "<<getNewMaxSpeed(dist, v0)<<endl;
		}
		else {
		  local_minima_pts[i-1].max_speed = getNewMaxSpeed(dist, v1);
		  cout<<"Possible new speed "<<getNewMaxSpeed(dist, v1)<<endl;
		}
	      i-=2;
	      
	    }
	  }
	}
      }
    }
    sw2.end();
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
    max_speed_ = max_speed;
    global_frame_ = "/robot_0/map";
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    ros::Publisher global_path_pub;
    global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 1, true);
    
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
      point_info.max_speed = sqrt(max_lat_acc*turning_rad);
      point_info.idx = path_no++;
      if(point_info.max_speed > max_speed) point_info.max_speed = max_speed;
      point_info.curve_max_speed = point_info.max_speed;
      point_info.time = 0.05/max_speed;
      path_info.push_back(point_info);
    }
    sw.end();    
    cout<<path_info.size()<<" point size"<<endl;
    vector<PointInfo> local_minima_pts = localMinimaSearch(path_info);
    //publishPathInfo(path_info, local_minima_pts);
    cout<<local_minima_pts.size()<<" local minima found"<<endl;
    path_info[0].max_speed = 0.0;
    path_info[path_info.size()-1].max_speed = 0.0;
    local_minima_pts.push_back(path_info[0]);
    local_minima_pts.push_back(path_info[path_info.size()-1]);
    sort(local_minima_pts.begin(), local_minima_pts.end(), compareByIdx);
    
    vector<PointInfo> local_minima_pts_copy = local_minima_pts;
    //sort(local_minima_pts.begin(), local_minima_pts.end(), compareByMaxSpeed);
    printLocalMinimaStatus("temp minima pts", local_minima_pts);
    mainFunction(local_minima_pts, path_info, max_speed, max_acc, max_jerk);
    publishPathInfo(path_info, local_minima_pts_copy);
    return;
    //check for error
    vector<PointInfo> error_pts;
    for(size_t i=0; i<path_info.size(); i++){
      double speed_err = path_info[i].max_speed - path_info[i].speed_profile;
      if(speed_err < 0){
	//cout<<"Error at "<<i<<" max_speed is "<<path_info[i].max_speed<<" but running at "<<path_info[i].speed_profile<<" error "<<speed_err<<endl;
	PointInfo p;
	//temperorary uses max_speed as the value holder for speed err
	p.max_speed = speed_err;
	p.idx = path_info[i].idx;
	p.speed_profile = path_info[i].max_speed;
	p.dist = path_info[i].dist;
	error_pts.push_back(p);
      }
    }
    vector<PointInfo> new_local_minima = localMinimaSearch(error_pts);
    printLocalMinimaStatus("new local minima", new_local_minima);
    for(size_t i=0; i<new_local_minima.size(); i++){
      //change back to the correct max speed
      new_local_minima[i].max_speed = new_local_minima[i].speed_profile;
      local_minima_pts.push_back(new_local_minima[i]);
    }
    sort(local_minima_pts.begin(), local_minima_pts.end(), compareByIdx);
    printLocalMinimaStatus("temp minima pts 2", local_minima_pts);
    mainFunction(local_minima_pts, path_info, max_speed, max_acc, max_jerk);
    publishPathInfo(path_info, local_minima_pts_copy);
    printLocalMinimaStatus("temp minima pts 3", local_minima_pts);
    ros::spin();
      
  }
  
  
};

int main(int argc, char** argv){
  TrajectorySimulator ts(argc, argv);
  return 0;
}