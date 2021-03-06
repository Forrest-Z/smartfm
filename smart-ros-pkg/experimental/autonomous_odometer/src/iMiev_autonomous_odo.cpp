#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <pnc_msgs/speed_contribute.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <long_control/PID_Msg.h>
#include <long_control/PID_Msg_old.h>
#include <lowlevel_controllers/PID.h>
#include <mysql++/mysql++.h>
#include <yaml-cpp/yaml.h>
#include <phidget_encoders/Encoders.h>

using namespace std;

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
    if(table_name_.size() > 255) {
      table_name_.substr(table_name_.size()-255, 255);
      cout<<"table size too large, trucating to the maximum size "<<table_name_<<endl;
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
  
  void updateData(double autonomous_dist, ros::Time starttime, ros::Time endtime, double total_time, double max_speed, double average_speed, int status){
    mysqlpp::DateTime sql_s_time((time_t)starttime.toSec()), sql_e_time((time_t)endtime.toSec());
    string sql_s_time_str = sql_s_time.str();
    string sql_e_time_str = sql_e_time.str();
    updateDetails(autonomous_dist, sql_s_time_str, sql_e_time_str, total_time, max_speed, average_speed, status);
  }
  bool createRecord(double autonomous_dist, string starttime, string endtime, double total_time, double max_speed, double average_speed, int status){
    stringstream insertdata_ss;
    insertdata_ss<<"insert into "<<table_name_<<" (id, starttime, endtime, distance, totaltime, maxspeed, averagespeed, status) values (";
    insertdata_ss<<"'"<<record_id_<<"', ";
    insertdata_ss<<"'"<<starttime<<"', ";
    insertdata_ss<<"'"<<endtime<<"', ";
    insertdata_ss<<autonomous_dist<<", ";
    insertdata_ss<<total_time<<", ";
    insertdata_ss<<max_speed<<", ";
    insertdata_ss<<average_speed<<", ";
    insertdata_ss<<status<<")";
    mysqlpp::Query insertdata_query = conn_.query(insertdata_ss.str());
    mysqlpp::SimpleResult insert_res = insertdata_query.execute();
    if(!insert_res) cout<<"Failed "<<insertdata_query.error()<<endl;
    return insert_res;
  }
  int getData(string id){
    stringstream ss;
    ss<<"select * from "<<table_name_<<" where "<<table_name_<<".id='"<<id<<"'";
    mysqlpp::Query query = conn_.query(ss.str());
    if(mysqlpp::StoreQueryResult res = query.store()){
      if(res.num_rows() == 0) return -1;
      return res[0][7];
    }
    return -1;
  }
private:
  void createTable()
  {
    //reminder: mysql has a hard limit on 4096 columns
    stringstream createtb_ss;
    createtb_ss << "create table "<<table_name_ <<" (id varchar(255) not null, starttime datetime not null, endtime datetime not null";
    createtb_ss << ", distance double not null, totaltime double not null, maxspeed double not null, averagespeed double not null, status int not null"; 
    createtb_ss << ", PRIMARY Key(id))";
    mysqlpp::Query createtb_query = conn_.query(createtb_ss.str());
    mysqlpp::SimpleResult createtb_res = createtb_query.execute();
    cout<<createtb_ss.str()<<endl;
    if(createtb_res) cout<<"Table create successful"<<endl;
    else cout<<"Table create failed: "<<createtb_query.error()<<endl;
  }
  void updateDetails(double autonomous_dist, string starttime, string endtime, double total_time, double max_speed, double average_speed, int status){
    stringstream ss;
    ss<<"update "<<table_name_ << " set distance="<<autonomous_dist<<", totaltime="<<total_time<<", status="<<status<<", starttime='"<<starttime<<"'";
    ss<<", endtime='"<<endtime<<"', maxspeed="<<max_speed<<", averagespeed="<<average_speed<<" where "<<table_name_<<".id='"<<record_id_<<"'";
    mysqlpp::Query updatedata_query = conn_.query(ss.str());
    mysqlpp::SimpleResult insert_res = updatedata_query.execute();
    if(!insert_res) 
      cout<<"Update failed: "<<updatedata_query.error()<<endl;
  }
  
  
};


bool speed_status_positive = false;
double speedstatus_lastreceived_ = 0.0;
double joy_steer_=100.0, steer_ang_=0.0;
double joysteer_lastreceived_=0.0, steerang_lastreceived_=0.0;
double speed_err_ = 999.9;
geometry_msgs::Point last_point_;
double autonomous_dist_ = 0.0;
double total_time_ = 0.0;
double max_speed_ = 0.0;
double last_steer_ang_ = 0.0;
bool first_odo_ = true;
ros::Time starttime_, endtime_;
MySQLHelper *sql_;

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

template <typename T>
void pidCallback(T pid){
  speed_err_ = pid.v_filter-pid.desired_vel;
  speed_now_ = pid.v_filter;
}

int pid_type_ = 0;
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
    
    if(first_odo_){
      starttime_ = amcl_pose.header.stamp;
      first_odo_ = false;
    }
    endtime_ = amcl_pose.header.stamp;
    if(fabs(speed_now_)>1e-3){
      total_time_+=dist/speed_now_;
      sql_->updateData(autonomous_dist_, starttime_, endtime_, total_time_, max_speed_, autonomous_dist_/total_time_, pid_type_);
    }
  }
//   cout<<"Distance_travel "<<dist<<" joysteer "<<joy_steer_<<" steerang "<<steer_ang_<<" speed_stat "<<speed_status_positive
//   <<" speed_err "<<speed_err<<" auto_dist "<<autonomous_dist_<<endl;
   cout<<"Summary: Odometer:"<<autonomous_dist_<<" s_t:"<<starttime_.toSec()<<" e_t:"<<endtime_.toSec()<<" total time:"<<total_time_<<" max speed:"<<max_speed_<<" average speed:"<<autonomous_dist_/total_time_<<"\xd"<<flush;
  last_steer_ang_=steer_ang_;
  last_point_ =cur_point ;
}

struct TopicType{
  string type;
  string md5;
};

struct TopicTypes{
  vector<TopicType> topic_types;
};

struct TopicName{
  string topic;
  string type;
  int messages;
};

struct TopicNames{
  vector<TopicName> topic_names;
};

void operator >> (const YAML::Node& node, TopicTypes& t){
  for(size_t i=0; i<node.size(); i++){
    TopicType tt;
    node[i]["type"] >> tt.type;
    node[i]["md5"] >> tt.md5;
    t.topic_types.push_back(tt);
  }
}

void operator >> (const YAML::Node& node, TopicNames& t){
  for(size_t i=0; i<node.size(); i++){
    TopicName tt;
    node[i]["topic"] >> tt.topic;
    node[i]["type"] >> tt.type;
    node[i]["messages"] >> tt.messages;
    t.topic_names.push_back(tt);
  }
}

std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
    }
    pclose(pipe);
    return result;
}

string findTopicAndHash(string match_topic, string match_hash, TopicNames &topic_names, TopicTypes &topic_types){
  string topic="";
  string hash="";
  string type="";
  bool topic_match = false;
  for(size_t i=0; i<topic_names.topic_names.size(); i++){
//     cout<<"searching for match "<<topic_names.topic_names[i].topic<<" == "<<match_topic<<" ?"<<endl;
    if(topic_names.topic_names[i].topic.find(match_topic) != string::npos){
      topic = topic_names.topic_names[i].topic;
      type = topic_names.topic_names[i].type;
      topic_match = true;
    }
  }
  string final_topic="";
  if(topic_match){
//     cout<<"found match: "<<topic<<endl;
    for(size_t i=0; i<topic_types.topic_types.size(); i++){
      if(topic_types.topic_types[i].md5.compare(match_hash) == 0){
// 	cout<<"found md5"<<endl;
	final_topic=topic;
      }
    }
  }
  return final_topic;
}

string checkout_topics(string topic, vector<bool> &topic_checkout){
  if(topic.size()==0) topic_checkout.push_back(false);
  return topic;
}

string rosbag_file_;
void *startROSbagPlay(void *threadid){
  stringstream bag_play_cmd; 
  bag_play_cmd<<"rosbag play -r3 --clock "<<rosbag_file_;
  cout<<bag_play_cmd.str()<<endl;
  string output = exec(bag_play_cmd.str().c_str());
  cout<<endl;
  ros::shutdown();
}

#include <fstream>
int main(int argc, char** argv){
  ros::init(argc, argv, "iMiev_autonomous_odo");
  if(argc < 2) {
    cout<<"Please give a bag file for the current run"<<endl;
    return 1;
  }
  rosbag_file_ = argv[1];
  MySQLHelper sql(string("autonomous_odometer"), string("SCOT"), string(argv[1]));
  sql_ = &sql;
  int database_status = sql.getData(string(argv[1]));
  // -1: not found
  // 0: no matching topics
  // 1: pid long_control_new
  // 2: pid long_control_old
  // 3: pid lowlevel_controllers
  if(database_status != -1) {
    cout<<"Record found, not going through again. "<<argv[1]<<endl;
    return 1;
  }
  stringstream bag_open_cmd;
  bag_open_cmd<<"rosbag info --yaml "<<argv[1];
  string output = exec(bag_open_cmd.str().c_str());
  istringstream output_istream(output);
  YAML::Parser parser(output_istream);
  cout<<"rosbag "<<rosbag_file_<< " parsed"<<endl;
//   std::ifstream fin(argv[1]);
//   YAML::Parser parser(fin);

  YAML::Node doc;
  parser.GetNextDocument(doc);
  const YAML::Node& topic_type_node = doc["types"];
  TopicTypes topic_types;
  topic_type_node >> topic_types;
  
  const YAML::Node& topic_name_node = doc["topics"];
  TopicNames topic_names;
  topic_name_node >> topic_names;

  string speed_contribute_md5 = "2fdc008d2a7c0e8f6d1a11c04f3f9536";
  string float64_md5 = "fdb28210bfa9d7c91146260178d9a584";
  string PoseWithCovarianceStamped_md5 = "953b798c0f514ff060a53a3498ce6246";
  string long_control_new = "7da7da52c4166a35ed80e28993284bbc";
  string long_control_old = "053e89c94725c4fc2fe528580fd49042";
  string lowlevel_control = "f6afedc367dc1ac3ed37c0fd2e923fd9";
  
  vector<bool> topics_checkout;
  string speed_status_topic = checkout_topics(findTopicAndHash(string("speed_status"), speed_contribute_md5, topic_names, topic_types), topics_checkout);
  string joy_steer_topic = checkout_topics(findTopicAndHash(string("joy_steer"), float64_md5, topic_names, topic_types), topics_checkout);
  string steerang_topic = checkout_topics(findTopicAndHash(string("steerang"), float64_md5, topic_names, topic_types), topics_checkout);
  string amcl_pose_topic = checkout_topics(findTopicAndHash(string("amcl_pose"), PoseWithCovarianceStamped_md5, topic_names, topic_types), topics_checkout);
  string pid_term_new = findTopicAndHash(string("pid"), long_control_new, topic_names, topic_types);
  string pid_term_old  = findTopicAndHash(string("pid"), long_control_old, topic_names, topic_types);
  string lowlevel_pid = findTopicAndHash(string("pid"), lowlevel_control, topic_names, topic_types);
  ros::Subscriber pid_sub;
  ros::NodeHandle nh;
  if(pid_term_new.size() > 0){
    pid_sub = nh.subscribe(pid_term_new, 10, pidCallback<long_control::PID_Msg>);
    pid_type_ = 1;
  }
  else if(pid_term_old.size() > 0){
    pid_sub = nh.subscribe(pid_term_old, 10, pidCallback<long_control::PID_Msg_old>);
    pid_type_ = 2;
  }
  else if(lowlevel_pid.size() > 0) {
    pid_sub = nh.subscribe(lowlevel_pid, 10, pidCallback<lowlevel_controllers::PID>);
    pid_type_ = 3;
  }
  else {
    topics_checkout.push_back(false);
  }
  mysqlpp::DateTime sql_time((time_t)0);
  if(topics_checkout.size()>0){
    sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, 0);
    return 1;
  }
  else
    sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, pid_type_);
  ros::Subscriber joysteer_sub = nh.subscribe(joy_steer_topic, 10, joysteerCallback);
  ros::Subscriber  steerang_sub = nh.subscribe(steerang_topic, 10, steerangCallback);
  ros::Subscriber amclpose_sub = nh.subscribe(amcl_pose_topic, 10, amclposeCallback);
  ros::Subscriber speed_status = nh.subscribe(speed_status_topic, 10, speedStatusCallback);
  
  pthread_t threads[1];
  int thread = 0;
  int rc = pthread_create(&threads[0], NULL, startROSbagPlay, (void *)thread);
  if(rc) cout<<"Unable to create thread"<<endl;
  ros::spin();

  return 0;
}
