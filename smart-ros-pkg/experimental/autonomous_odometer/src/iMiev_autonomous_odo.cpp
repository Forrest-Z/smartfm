#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pnc_msgs/speed_contribute.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <long_control/PID_Msg.h>
#include <mysql++/mysql++.h>

using namespace std;

bool speed_status_positive = false;
double speedstatus_lastreceived_ = 0.0;
double joy_steer_=0.0, steer_ang_=0.0;
double joysteer_lastreceived_=0.0, steerang_lastreceived_=0.0;
double speed_err_ = 999.9;
geometry_msgs::Point last_point_;
class MySQLHelper{
public:
  string table_name_, record_id_;
  mysqlpp::Connection conn_;
  int table_size_;
  bool first_update_;
  MySQLHelper(string database, string name, string record_id): table_name_(name), record_id_(record_id), conn_(false), table_size_(-1){
     const char* db = database.c_str(), *server = "localhost", *user = "root", *pass = "nus";
    if (conn_.connect(db, server, user, pass)) {
      cout <<"DB connected"<<endl;
    }
    else {
      cerr << "DB connection failed: " << conn_.error() << endl;
      exit(1);
    }
    mysqlpp::Query query = conn_.query("select * from "+table_name_);
    if(mysqlpp::StoreQueryResult res = query.store()){
      cout << "Table "<<table_name_<<" have "<<res.num_fields()<<"x"<<res.num_rows()<<endl;
      if (res.num_fields() > 0)
	table_size_ = res.num_rows();
    }
    else {
      cout << "Table not found, creating one"<<endl;
      createTable();
    }
    first_update_ = true;
  }
  
  void updateData(double autonomous_dist, ros::Time starttime, ros::Time endtime, double total_time, double max_speed, double average_speed){
    mysqlpp::DateTime sql_s_time((time_t)starttime.toSec()), sql_e_time((time_t)endtime.toSec());
    string sql_s_time_str = sql_s_time.str();
    string sql_e_time_str = sql_e_time.str();
    if(first_update_){
      first_update_ = false;
      if(!createRecord(autonomous_dist, sql_s_time_str, sql_e_time_str, total_time, max_speed, average_speed)){
	cout<<"Data is available, not going to process the odometer"<<endl;
	exit(1);
      }
    }
    updateDetails(autonomous_dist, sql_e_time_str, total_time, max_speed, average_speed);
  }
private:
  void createTable()
  {
    //reminder: mysql has a hard limit on 4096 columns
    stringstream createtb_ss;
    createtb_ss << "create table "<<table_name_ <<" (id varchar(100) not null, starttime datetime not null, endtime datetime not null";
    createtb_ss << ", distance double not null, totaltime double not null, maxspeed double not null, averagespeed double not null"; 
    createtb_ss << ", PRIMARY Key(id))";
    mysqlpp::Query createtb_query = conn_.query(createtb_ss.str());
    mysqlpp::SimpleResult createtb_res = createtb_query.execute();
    cout<<createtb_ss.str()<<endl;
    if(createtb_res) cout<<"Table create successful"<<endl;
    else cout<<"Table create failed: "<<createtb_query.error()<<endl;
  }
  void updateDetails(double autonomous_dist, string endtime, double total_time, double max_speed, double average_speed){
    stringstream ss;
    ss<<"update "<<table_name_ << " set distance="<<autonomous_dist<<", totaltime="<<total_time;
    ss<<", endtime='"<<endtime<<"', maxspeed="<<max_speed<<", averagespeed="<<average_speed<<" where "<<table_name_<<".id='"<<record_id_<<"'";
    mysqlpp::Query updatedata_query = conn_.query(ss.str());
    mysqlpp::SimpleResult insert_res = updatedata_query.execute();
    if(!insert_res) 
      cout<<"Update failed: "<<updatedata_query.error()<<endl;
  }
  bool createRecord(double autonomous_dist, string starttime, string endtime, double total_time, double max_speed, double average_speed){
    stringstream insertdata_ss;
    insertdata_ss<<"insert into "<<table_name_<<" (id, starttime, endtime, distance, totaltime, maxspeed, averagespeed) values (";
    insertdata_ss<<"'"<<record_id_<<"', ";
    insertdata_ss<<"'"<<starttime<<"', ";
    insertdata_ss<<"'"<<endtime<<"', ";
    insertdata_ss<<autonomous_dist<<", ";
    insertdata_ss<<total_time<<", ";
    insertdata_ss<<max_speed<<", ";
    insertdata_ss<<average_speed<<")";
    mysqlpp::Query insertdata_query = conn_.query(insertdata_ss.str());
    mysqlpp::SimpleResult insert_res = insertdata_query.execute();
    if(!insert_res) cout<<"Failed "<<insertdata_query.error()<<endl;
    return insert_res;
  }
  
};
  
void speedStatusCallback(pnc_msgs::speed_contribute speed_status){
  speedstatus_lastreceived_=ros::Time::now().toSec();
  if(speed_status.element == 0 /*no response*/ || speed_status.element == 2 /*no path*/
    || speed_status.element == 7 /*emergency*/ || speed_status.element == 15 /*manual*/)
    speed_status_positive = false;
  else
    speed_status_positive = true;
}

void joysteerCallback(std_msgs::Float64 joy_steer){
  joysteer_lastreceived_=ros::Time::now().toSec();
  joy_steer_ = joy_steer.data;
}

void steerangCallback(std_msgs::Float64 steer_ang){
  steerang_lastreceived_=ros::Time::now().toSec();
  steer_ang_ = steer_ang.data;
}

double speed_now_;
void pidCallback(long_control::PID_Msg pid){
  speed_err_ = pid.vel_err;
  speed_now_ = pid.v_filter;
}

double autonomous_dist_ = 0.0;
double total_time_ = 0.0;
double max_speed_ = 0.0;
double last_steer_ang_ = 0.0;
bool first_odo_ = true;
ros::Time starttime_, endtime_;
MySQLHelper *sql_;
void amclposeCallback(geometry_msgs::PoseWithCovarianceStamped amcl_pose){
  geometry_msgs::Point cur_point = amcl_pose.pose.pose.position;
  double x = last_point_.x - cur_point.x;
  double y = last_point_.y - cur_point.y;
  double dist = sqrt(x*x + y*y);
  
  vector<bool> flags;
  flags.push_back(dist < 1.0);
  flags.push_back(fabs(joy_steer_ - steer_ang_) < 5.0 || fabs(last_steer_ang_ - steer_ang_)>1e-3);
  flags.push_back(fabs(speed_err_) < 1.5);
  double time_now = ros::Time::now().toSec();
  flags.push_back(time_now - joysteer_lastreceived_ < 1.0);
  flags.push_back(time_now - steerang_lastreceived_ < 1.0);
  bool add_odo = true;
  for(size_t i=0; i<flags.size(); i++){
    if(!flags[i]) {
      add_odo = false;
      break;
    }
  }
  if(add_odo){
    autonomous_dist_+=dist;
    if(max_speed_ < speed_now_) max_speed_ = speed_now_;
    
    if(first_odo_)
      starttime_ = amcl_pose.header.stamp;
    endtime_ = amcl_pose.header.stamp;
    if(fabs(speed_now_)>1e-3){
      total_time_+=dist/speed_now_;
      sql_->updateData(autonomous_dist_, starttime_, endtime_, total_time_, max_speed_, autonomous_dist_/total_time_);
    }
  }
//   cout<<"Distance_travel "<<dist<<" joysteer "<<joy_steer_<<" steerang "<<steer_ang_<<" speed_stat "<<speed_status_positive
//   <<" speed_err "<<speed_err<<" auto_dist "<<autonomous_dist_<<endl;
//   cout<<"Summary: Odometer:"<<autonomous_dist_<<" s_t:"<<starttime_.toSec()<<" e_t:"<<endtime_.toSec()<<" total time:"<<total_time_<<" max speed:"<<max_speed_<<" average speed:"<<autonomous_dist_/total_time_<<endl;
  last_steer_ang_=steer_ang_;
  last_point_ =cur_point ;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "iMiev_autonomous_odo");
  if(argc < 2) {
    cout<<"Please give a record id for the current run"<<endl;
    return 1;
  }
  ros::NodeHandle nh;
  ros::Subscriber joysteer_sub = nh.subscribe("iMiev/pronto/joy_steer", 10, joysteerCallback);
  ros::Subscriber  steerang_sub = nh.subscribe("iMiev/pronto/steerang", 10, steerangCallback);
  ros::Subscriber amclpose_sub = nh.subscribe("iMiev/amcl_pose", 10, amclposeCallback);
  ros::Subscriber speed_status = nh.subscribe("iMiev/speed_status", 10, speedStatusCallback);
  ros::Subscriber pid_sub = nh.subscribe("iMiev/pid_term", 10, pidCallback);
  MySQLHelper sql(string("autonomous_odometer"), string("SCOT"), string(argv[1]));
  sql_ = &sql;
  ros::spin();

  return 0;
}