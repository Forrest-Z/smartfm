#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <pnc_msgs/speed_contribute.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <long_control/PID_Msg.h>
#include <long_control/PID_Msg_old.h>
#include <lowlevel_controllers/PID.h>
#include <mysql++/mysql++.h>
#include <yaml-cpp/yaml.h>
#include <phidget_encoders/Encoders.h>
#include <old_msgs/angle.h>
#include <old_msgs/halsampler.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

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
      cout << "Table "<<table_name_<<" has "<<res.num_fields()<<"x"<<res.num_rows()<<endl;
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


double autonomous_dist_ = 0.0;
double total_time_ = 0.0;
double max_speed_ = 0.0;
double last_steer_ang_ = 0.0;
bool first_odo_ = true;
ros::Time starttime_, endtime_;
double speed_now_ = 0.0;
double last_cmd_speed_ = 0.0;
MySQLHelper *sql_;
geometry_msgs::Point last_point_;
ros::Time last_amcl_stamp_;
ros::Time last_steering_callback_;
int automode_ = 0;
bool auto_state_ = false;
void automodeCallback(std_msgs::Bool automode){
  auto_state_ = automode.data;
}

void samplerCallback(old_msgs::halsampler sampler){
  speed_now_ = sampler.vel;
}
void odomCallback(nav_msgs::Odometry odom){
  speed_now_ = odom.twist.twist.linear.x;
}
void steeringCallback(old_msgs::angle angle){
  last_steering_callback_ = ros::Time::now();
}
void cmdvelCallback(geometry_msgs::Twist tw){
  last_cmd_speed_ = tw.linear.x;
}

void amclposeCallback(geometry_msgs::PoseWithCovarianceStamped amcl_pose){
  geometry_msgs::Point cur_point = amcl_pose.pose.pose.position;
  double x = last_point_.x - cur_point.x;
  double y = last_point_.y - cur_point.y;
  double dist = sqrt(x*x + y*y);
  
  bool add_odo = false;
  if(automode_ == 1)
    add_odo = auto_state_;
  else if(automode_ == 2){
    double time_diff = fabs((ros::Time::now().toSec() - last_steering_callback_.toSec()));
    double speed_diff = fabs(last_cmd_speed_ - speed_now_);
    add_odo = time_diff < 0.5 && speed_diff < 1.5;
  }
  if(add_odo){
    if(first_odo_){
      starttime_ = amcl_pose.header.stamp;
      last_amcl_stamp_ = starttime_;
      first_odo_ = false;
      dist = 0.0;
    }
    if(dist > 1.0) return;
    autonomous_dist_+=dist;
    double duration = (amcl_pose.header.stamp-last_amcl_stamp_).toSec();
    endtime_ = amcl_pose.header.stamp;
    total_time_+= duration;
    if(max_speed_<speed_now_) max_speed_ = speed_now_;
    sql_->updateData(autonomous_dist_, starttime_, endtime_, total_time_, max_speed_, autonomous_dist_/total_time_, automode_);
  }
//   cout<<"Distance_travel "<<dist<<" joysteer "<<joy_steer_<<" steerang "<<steer_ang_<<" speed_stat "<<speed_status_positive
//   <<" speed_err "<<speed_err<<" auto_dist "<<autonomous_dist_<<endl;
   cout<<"Summary: Odometer:"<<autonomous_dist_<<" s_t:"<<starttime_.toSec()<<" e_t:"<<endtime_.toSec()<<" total time:"<<total_time_<<" max speed:"<<max_speed_<<" average speed:"<<autonomous_dist_/total_time_<<"\xd"<<flush;

  last_point_ =cur_point ;
  last_amcl_stamp_ = amcl_pose.header.stamp;
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
      //quick hack
      if(topic_names.topic_names[i].topic.find("gps") != string::npos) continue;
      topic = topic_names.topic_names[i].topic;
      type = topic_names.topic_names[i].type;
      topic_match = true;
      break;
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
  ros::init(argc, argv, "Rudolph_autonomous_odo");
  if(argc < 2) {
    cout<<"Please give a bag file for the current run"<<endl;
    return 1;
  }
  rosbag_file_ = argv[1];
  MySQLHelper sql(string("autonomous_odometer"), string("Rudolph"), string(argv[1]));
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

  string angle_md5 = "2d11dcdbe5a6f73dd324353dc52315ab";
  string throttle_md5 = "0d974440801c3c58757fe6ef22b11eb3";
  string float64_md5 = "fdb28210bfa9d7c91146260178d9a584";
  string geometry_twist_md5 = "9f195f881246fdfa2798d1d3eebca84a";
  string PoseWithCovarianceStamped_md5 = "953b798c0f514ff060a53a3498ce6246";
  string nav_msgs_path_md5 = "6227e2b7e9cce15051f669a5e197bbf7";
  string nav_msgs_odom_md5 = "cd5e73d190d741a2f92e81eda573aca7";
  string std_msgs_bool = "8b94c1b53db61fb6aed406028ad6332a";
  string sampler_md5 = "e68537df914d3d6ed7f2787b0001f4ed";
  vector<bool> topics_checkout;
  
  string autonomode_topic = checkout_topics(findTopicAndHash(string("button_state_automode"), std_msgs_bool, topic_names, topic_types), topics_checkout);
  string amcl_pose_topic = checkout_topics(findTopicAndHash(string("amcl_pose"), PoseWithCovarianceStamped_md5, topic_names, topic_types), topics_checkout);
  string odom_topic = checkout_topics(findTopicAndHash(string("odom"), nav_msgs_odom_md5, topic_names, topic_types), topics_checkout);
  mysqlpp::DateTime sql_time((time_t)0);
  if(amcl_pose_topic.size() == 0){
    sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, 0);
    return 1;
  }
  ros::NodeHandle nh;
  if(topics_checkout.size() == 0){
    //easy, automode topic is available
    automode_ = 1;
    sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, automode_);
    ros::Subscriber amclpose_sub = nh.subscribe(amcl_pose_topic, 10, amclposeCallback);
    ros::Subscriber automode_sub = nh.subscribe(autonomode_topic, 10, automodeCallback);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odomCallback);
    pthread_t threads[1];
    int thread = 0;
    int rc = pthread_create(&threads[0], NULL, startROSbagPlay, (void *)thread);
    if(rc) cout<<"Unable to create thread"<<endl;
    ros::spin();
  } else {
    //alright for the very old bags....
    topics_checkout.clear();
    string amcl_pose_topic = checkout_topics(findTopicAndHash(string("amcl_pose"), PoseWithCovarianceStamped_md5, topic_names, topic_types), topics_checkout);
    string golfcar_brake = checkout_topics(findTopicAndHash(string("golfcar_brake"), angle_md5, topic_names, topic_types), topics_checkout);
    string golfcar_steering = checkout_topics(findTopicAndHash(string("golfcar_steering"), angle_md5, topic_names, topic_types), topics_checkout);
    string golfcar_speed = checkout_topics(findTopicAndHash(string("golfcar_speed"), throttle_md5, topic_names, topic_types), topics_checkout);
    string cmd_vel = checkout_topics(findTopicAndHash(string("cmd_vel"), geometry_twist_md5, topic_names, topic_types), topics_checkout);
    string odom_topic = checkout_topics(findTopicAndHash(string("odom"), nav_msgs_odom_md5, topic_names, topic_types), topics_checkout);
    string global_plan = checkout_topics(findTopicAndHash(string("global_plan"), nav_msgs_path_md5, topic_names, topic_types), topics_checkout);
    if(topics_checkout.size()>0){
      sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, 0);
      return 1;
    }
    //checked for odom but priority for getting the speed is on golfcar_sampler
    string sampler_topic = checkout_topics(findTopicAndHash(string("sampler"), sampler_md5, topic_names, topic_types), topics_checkout);
    
    sql.createRecord(0.0, sql_time.str(), sql_time.str(), 0.0, 0.0, 0.0, automode_);
    ros::Subscriber amclpose_sub = nh.subscribe(amcl_pose_topic, 10, amclposeCallback);
    ros::Subscriber steering_sub = nh.subscribe(golfcar_steering, 10, steeringCallback);
    ros::Subscriber speed_sub;
    if(sampler_topic.size()>0){
      speed_sub = nh.subscribe(sampler_topic, 10, samplerCallback);
      automode_ = 3;
    }
    else{
      speed_sub = nh.subscribe(odom_topic, 10, odomCallback);
      automode_ = 2;
    }
    ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_vel, 10, cmdvelCallback);
    pthread_t threads[1];
    int thread = 0;
    int rc = pthread_create(&threads[0], NULL, startROSbagPlay, (void *)thread);
    if(rc) cout<<"Unable to create thread"<<endl;
    ros::spin();
  }
  
 

  return 0;
}