#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/tokenizer.hpp>
#include <pronto_adapter/pronto.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#define MAX_LEN 1000

//Adapter that synchronize with pronto's inbound and outbound list
//and turn the list into equivalent ROS's subscribers and publishers

using boost::asio::ip::udp;
using namespace std;

struct PacketTypeAndData{
  //following pronto's convention
  //0: int, 2: double, 4: string
  int type;
  
  //only one of the following fields will be 
  //populated according to the type
  string last_data_str;
  int last_data_int;
  double last_data_double;
  
  PacketTypeAndData() : last_data_str(""),
  last_data_int(0), last_data_double(0.) {}
};
  
struct ROSPubAndPTD{
  ros::Publisher publisher;
  PacketTypeAndData packet;
};

//http://stackoverflow.com/questions/289347/using-strtok-with-a-stdstring
void split(const string& str, const string& delim, vector<string>& parts) {
  size_t start, end = 0;
  while (end < str.size()) {
    start = end;
    while (start < str.size() && (delim.find(str[start]) != string::npos)) {
      start++;  // skip initial whitespace
    }
    end = start;
    while (end < str.size() && (delim.find(str[end]) == string::npos)) {
      end++; // skip to end of word
    }
    if (end-start != 0) {  // just ignore zero-length strings.
      parts.push_back(string(str, start, end-start));
    }
  }
}


std::vector<double> messageDecodingToDouble (std::string msg_in)
{
  std::vector<std::string> vect_out;
  std::vector<double> vect_double;
  std::string msg_out;
  double msg_double;
  std::string separator = "|";
  bool end_of_msg = false;
  unsigned end_pos = 0;

    //erase the last "]"
  msg_in = msg_in.erase(msg_in.size()-1,1);
  //erase the first "["
  msg_in = msg_in.erase(0,1);
  while(!end_of_msg)
    {
      end_pos = msg_in.find(separator);

      msg_out = msg_in.substr(0,end_pos);
      vect_out.push_back(msg_out);
      //erase the message out fron in put message
      msg_in = msg_in.erase(0,msg_out.size());
      //erase the separator "|"
      msg_in = msg_in.erase(0,1);
      if(msg_in.size() == 0) end_of_msg = true;
    
    }

    for(size_t i = 0; i < vect_out.size(); i++)
    {
      if(!(vect_out[i].at(0) == 'D'))
    {
      //if begin with ':','N' or 'C' then we don't want to convert it
      msg_double = NAN;
      vect_double.push_back(msg_double);
    }else{
        vect_out[i] = vect_out[i].erase(0,1); //erase the 'D'
        std::stringstream convert(vect_out[i]);
        if(!(convert >> msg_double)) msg_double = NAN;  //is not convertable then declare as "Not A Number"
        vect_double.push_back(msg_double);
      }
    }
    return vect_double;
}

string add_check_sum(string pMsg)
{
        int len = pMsg.size();
        if(len >= MAX_LEN || len < 5)
        {
                return NULL;
        }
        unsigned char sum = 0;

        if(pMsg[len-2] == '|' && pMsg[len-1] == 'C')
        {
                for(int i = 1; i < len ; i++)
                {
                        sum += pMsg[i];
                }
        }
        sum = sum % 256;
        char sum_hex_str[5] = "";

        sprintf(sum_hex_str,"%02x",sum); // set the width to 2 and filled with zero.


        pMsg += toupper(sum_hex_str[0]);
        pMsg += toupper(sum_hex_str[1]);
        pMsg += ']';

        return pMsg;
}

class ProntoAdapter
{
  boost::asio::io_service& io_service_;
  udp::socket socket_;
  udp::endpoint sender_endpoint_, remote_endpoint_;
  const string PING;
  const string PONG;
  const string REQ_OUT, REQ_OUT_ANS;
  const string REQ_IN, REQ_IN_ANS;
  
  string buffer_;
  enum { max_length = 1024 };
  char data_[max_length];
  ros::Subscriber brake_angle_sub_;
  ros::NodeHandle nh_;
  map<string, PacketTypeAndData> topic_ptd_map_;
  vector<string> subscribed_topics_;
  void messageToROSSubscriber(string msg, ros::NodeHandle &nh, vector<ros::Subscriber> &subs, map<string, PacketTypeAndData> &topic_ptd_map){
    vector<string> parts;
    split(msg, "|", parts);
    
    for(size_t i=2; i<parts.size(); i++){
      //default to float datatype
        int data_type = 2;
        //check if data type available
        string data_type_str = parts[i];
        //a checksum packet received, skip this packet
        if(data_type_str[0] == 'C') continue;
       stringstream ss_type(data_type_str.erase(0,1));
        if(ss_type >> data_type) i++;
       if(i >= parts.size()) continue;
       string topic(parts[i]);
       topic = topic.erase(0,1);
       stringstream ss;
       ss << "/pronto/"<<topic;
       PacketTypeAndData ptd;
       ptd.type = data_type;
       switch(data_type) {
         case 0: 
         subs.push_back(nh.subscribe<std_msgs::Int32>(ss.str(), 1, boost::bind(&ProntoAdapter::intCallback, this, _1, ss.str())));
         cout<<"Setting up int type subscriber under topic "<<ss.str()<<endl;
         break;
         case 2:
         subs.push_back(nh.subscribe<std_msgs::Float64>(ss.str(), 1, boost::bind(&ProntoAdapter::floatCallback, this, _1, ss.str())));
         cout<<"Setting up double type subscriber under topic "<<ss.str()<<endl;
         break;
         case 4:
         subs.push_back(nh.subscribe<std_msgs::String>(ss.str(), 1, boost::bind(&ProntoAdapter::strCallback, this, _1, ss.str())));
         cout<<"Setting up string type subscriber under topic "<<ss.str()<<endl;
         break;
       }
       //initialize the storage for storing incoming data packet
       topic_ptd_map[ss.str()] = ptd; 
       subscribed_topics_.push_back(ss.str());
     }
     
}
vector<ROSPubAndPTD> messageToROSPublisher(string msg, ros::NodeHandle &nh) {

  cout<<"Raw outbound list received: "<<msg<<endl;
  vector<string> parts;
  split(msg, "|", parts);  
  
  vector<ROSPubAndPTD> topics;
  
 
   for(size_t i=2; i<parts.size(); i++){
       //default to float datatype
       int data_type = 2;
       //check if data type available
       string data_type_str = parts[i];
       //a checksum packet received, skip this packet
       if(data_type_str[0] == 'C') continue;
       stringstream ss_type(data_type_str.erase(0,1));
       if(ss_type >> data_type) i++;
       if(i >= parts.size()) continue;
       string topic(parts[i]);
       topic = topic.erase(0,1);
       stringstream ss;
       ss << "pronto/"<<topic;
       ROSPubAndPTD pub;
       switch(data_type) {
         case 0: 
         pub.publisher = nh.advertise<std_msgs::Int32>(ss.str(), 1);
         cout<<"Setting up int type publish under topic "<<ss.str()<<endl;
         break;
         case 2:
         pub.publisher = nh.advertise<std_msgs::Float64>(ss.str(), 1);
         cout<<"Setting up double type publish under topic "<<ss.str()<<endl;
         break;
         case 4:
         pub.publisher = nh.advertise<std_msgs::String>(ss.str(), 1);
         cout<<"Setting up string type publish under topic "<<ss.str()<<endl;
         break;
       }
       pub.packet.type = data_type;
       topics.push_back(pub);
     }
     
  return topics;
}
  string getResponse(string cmd, string expected_match){
    cmd = add_check_sum(cmd);
    string response;
    do{
    socket_.send_to(boost::asio::buffer(cmd), remote_endpoint_);
    boost::array<char, max_length> recv_buf;
    size_t len = socket_.receive_from(
    boost::asio::buffer(recv_buf), sender_endpoint_);
    response = string(recv_buf.data());
    response = response.substr(0, len);
    }while(response.find(expected_match) == string::npos);
    return response;
  }
  vector< ROSPubAndPTD > publishers_;
  vector<ros::Subscriber> subscribers_;
  void publishData(string data, vector< ROSPubAndPTD > &publishers){
    vector<string> data_vec;
    split(data, "|", data_vec);
    cout<<data_vec.size()<<endl;
    cout<<publishers.size()<<endl;
    if(data_vec.size() == publishers.size() + 3){
      for(size_t i=2; i<data_vec.size()-1; i++){
        string data = data_vec[i];
        data = data.substr(1, data.size());
        stringstream ss(data);
        std_msgs::Int32 msg_int;
        std_msgs::Float64 msg_double;
        ROSPubAndPTD *cur_pub = &publishers[i-2];
        switch (cur_pub->packet.type){
          case 0:
            if(ss>>msg_int.data) cur_pub->packet.last_data_int = msg_int.data;
            else msg_int.data = cur_pub->packet.last_data_int;
            cur_pub->publisher.publish(msg_int);
            break;
          case 2:
            if(ss>>msg_double.data) cur_pub->packet.last_data_double = msg_double.data;
            else msg_double.data =  (cur_pub->packet.last_data_double);
            cur_pub->publisher.publish(msg_double);
            break;
          case 4:
            std_msgs::String msg_str;
            if(ss.str().size()>0) {
              msg_str.data = ss.str();
              cur_pub->packet.last_data_str = ss.str();
            }
            else msg_str.data = cur_pub->packet.last_data_str;
            cur_pub->publisher.publish(msg_str);
            break;
        }
      }
    }
      
  }
public:
  ProntoAdapter(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      socket_(io_service, udp::endpoint(udp::v4(), port)),
      PING("[:AB|P0|C78]"),
      PONG("[:BA|G0|C"),
      REQ_OUT("[:BA|N?|C"),
      REQ_OUT_ANS("[:AB|L0|T"),
      REQ_IN("[:BA|L?|C"),
      REQ_IN_ANS("[:AB|N0|T")
  {
    remote_endpoint_ = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string("192.168.200.110"),  4000);
   
    //setup ros message publish list
    publishers_ = messageToROSPublisher(getResponse(REQ_IN, REQ_IN_ANS), nh_);
    
    //setup ros message subscription
    messageToROSSubscriber(getResponse(REQ_OUT, REQ_OUT_ANS), nh_, subscribers_, topic_ptd_map_);
    
    
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&ProntoAdapter::handleReceiveFrom, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    
    io_service.run();
  }
  
  void floatCallback(const ros::MessageEvent<std_msgs::Float64 const>& event, const string topic)
  {
    const std::string& publisher_name = event.getPublisherName();
    ros::M_string header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
  
    const std_msgs::Float64ConstPtr& msg = event.getMessage();
    cout<<publisher_name<<" "<<header["topic"]<<" "<<topic<<" "<<receipt_time<<" "<<msg->data<<endl;
    PacketTypeAndData ptd;
    ptd.last_data_double = msg->data;
    ptd.type = 2;
    topic_ptd_map_[topic] = ptd; 
  }
  
  void strCallback(const ros::MessageEvent<std_msgs::String const>& event, const string topic)
  {
    const std::string& publisher_name = event.getPublisherName();
    ros::M_string header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
  
    const std_msgs::StringConstPtr& msg = event.getMessage();
    cout<<publisher_name<<" "<<header["topic"]<<" "<<topic<<" "<<receipt_time<<" "<<msg->data<<endl;
    PacketTypeAndData ptd;
    ptd.last_data_str = msg->data;
    ptd.type = 4;
    topic_ptd_map_[topic] = ptd;
  }
  
  void intCallback(const ros::MessageEvent<std_msgs::Int32 const>& event, const string topic)
  {
    const std::string& publisher_name = event.getPublisherName();
    ros::M_string header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
  
    const std_msgs::Int32ConstPtr& msg = event.getMessage();
    cout<<publisher_name<<" "<<header["topic"]<<" "<<topic<<" "<<receipt_time<<" "<<msg->data<<endl;
    PacketTypeAndData ptd;
    ptd.last_data_int = msg->data;
    ptd.type = 0;
    topic_ptd_map_[topic] = ptd;
  }
  
  void handleReceiveFrom(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    if(ros::ok())
    {
      if (!error && bytes_recvd > 0)//data.size() > 0)
      {
        string data;
        for(size_t i=0; i<bytes_recvd; i++){
          buffer_+=data_[i];
          int escape_pos = buffer_.find("\r\n");
            
          if(escape_pos>0){
            data = buffer_.substr(0,escape_pos);
            cout<<"data received: "<<data<<endl;
            buffer_.erase(0, escape_pos+1);
          }
        }
        vector<double> decoded_data = messageDecodingToDouble(data);
      
        if(data.size()>0)
        {
          string answer_str;
          if(data == PING){
            answer_str = add_check_sum(PONG);
            socket_.async_send_to(
            boost::asio::buffer(answer_str), remote_endpoint_,
            boost::bind(&ProntoAdapter::handleSendTo, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
          }
          else {
            publishData(data, publishers_);
            stringstream ss;
            ss<<"[:BA|N0";
            for(size_t i=0; i<subscribed_topics_.size(); i++){
              ss<<"|D";
              PacketTypeAndData ptd = topic_ptd_map_[subscribed_topics_[i]];
              switch (ptd.type){
                case 0: ss<<ptd.last_data_int; break;
                case 2: ss<<ptd.last_data_double; break;
                case 4: ss<<ptd.last_data_str; break;
              }
            }
            ss<<"|C";
            //answer_str = boost::str(boost::format("[:BA|N0|D%d|D%d|D%d|D%d|D%d|D%d|D%d|D%d|D%d|D%d|C") %video_mux1 %veh_start %veh_enable %veh_motion 
            //%joy_steer %veh_steer %veh_brake %veh_throttle %veh_shift %veh_clutch);
            answer_str = add_check_sum(ss.str());
            cout<<"Sending "<<answer_str<<endl;
            for(map<string, PacketTypeAndData>::iterator it = topic_ptd_map_.begin(); it != topic_ptd_map_.end(); it++){
              switch (it->second.type){
                case 0: cout<<it->first<<" "<<it->second.type<<" "<<it->second.last_data_int<<endl; break;
                case 2: cout<<it->first<<" "<<it->second.type<<" "<<it->second.last_data_double<<endl; break;
                case 4: cout<<it->first<<" "<<it->second.type<<" "<<it->second.last_data_str<<endl; break;
              }
            }
          }
          socket_.async_send_to(
          boost::asio::buffer(answer_str), remote_endpoint_,
          boost::bind(&ProntoAdapter::handleSendTo, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
        }
        else {
          socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          boost::bind(&ProntoAdapter::handleReceiveFrom, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
        }
      }
      ros::spinOnce();
    }
    else {
      io_service_.stop();
    }
  }

  void handleSendTo(const boost::system::error_code& /*error*/,
      size_t /*bytes_sent*/)
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&ProntoAdapter::handleReceiveFrom, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pronto_adapter");
  ros::NodeHandle n("~");
  int port_number;
  n.param("port", port_number, atoi(argv[1])); 
  boost::asio::io_service io_service;
  ProntoAdapter s(io_service, port_number);
  
  
  
 
  return 0;
}