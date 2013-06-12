#include <ctime>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
//message type
#include <pronto_adapter/Pronto_msg.h>

#define MAX_LEN 1000

using boost::asio::ip::udp;
using namespace std;

std::vector<double> messageDecodingToDouble (std::string msg_in)
{
  std::vector<std::string> vect_out;
  std::vector<double> vect_double;
  std::string msg_out;
  double msg_double;
  std::string separator = "|";
  bool end_of_msg = false;
  unsigned end_pos = 0;

  std::cout << "message in: " << msg_in << std::endl << std::endl;
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

    for(int i = 0; i < vect_out.size(); i++)
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

	sprintf(sum_hex_str,"%x",sum);


	pMsg += toupper(sum_hex_str[0]);
	pMsg += toupper(sum_hex_str[1]);
	pMsg += ']';

	return pMsg;
}

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class udp_server
{
  const string PING;
  const string PONG;
public:
  udp_server(boost::asio::io_service& io_service)
  : socket_(io_service, udp::endpoint(udp::v4(), 4100)), remote_endpoint_(udp::v4(), 4000),
  PING("[:AB|P0|C78]"),
  PONG("[:BA|G0|C")
  {
    start_receive();
  }
  
private:
  string buffer_; 
  void start_receive()
  {
    socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
			       boost::bind(&udp_server::handle_receive, this,
					   boost::asio::placeholders::error,
		      boost::asio::placeholders::bytes_transferred));
  }
  
  void handle_receive(const boost::system::error_code& error,
		      std::size_t bytes_transferred)
  {
    string data = "";
    for(size_t i=0; i<bytes_transferred; i++){
      buffer_+=recv_buffer_[i];
      int escape_pos = buffer_.find("\r\n");
	
	if(escape_pos>0){
	  data = buffer_.substr(0,escape_pos);
	  cout<<data<<endl;
	  buffer_.erase(0, escape_pos+2);
	}
    }
    
    
    if (!error || error == boost::asio::error::message_size)
    {
      if(data == PING) {
	cout<<"PING received"<<endl;
	string answer_str = add_check_sum(PONG);
	boost::shared_ptr<std::string> message(
	new std::string(answer_str));
	cout<<"PONG sent "<<answer_str<<endl;
	socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
			  boost::bind(&udp_server::handle_send, this, message,
				      boost::asio::placeholders::error,
		  boost::asio::placeholders::bytes_transferred));
      }
      start_receive();
    }
  }
  
  void handle_send(boost::shared_ptr<std::string> /*message*/,
		   const boost::system::error_code& /*error*/,
		   std::size_t /*bytes_transferred*/)
  {
  }
  
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 512> recv_buffer_;
};

int main()
{
  try
  {
    boost::asio::io_service io_service;
    udp_server server(io_service);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  
  return 0;
}