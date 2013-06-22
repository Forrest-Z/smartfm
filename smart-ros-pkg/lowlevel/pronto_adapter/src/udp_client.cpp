#include <ctime>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
//message type
//#include <pronto_adapter/Pronto_msg.h>

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

class udp_server
{
  const string PING;
  const string PONG;
public:
  udp_server(boost::asio::io_service& io_service)
  : socket_(io_service),
  PING("[:AB|P0|C78]"),
  PONG("[:BA|G0|C")
  {
    
    local_endpoint_ = udp::endpoint(
      boost::asio::ip::address::from_string("192.168.200.101"), 4100);
    
    boost::system::error_code myError;
    //socket_.open( boost::asio::ip::udp::v4(), myError );
    //cout<<"Open socket status: "<<myError.message()<<endl;
    //boost::asio::socket_base::broadcast option(true);
    //socket_.set_option(option);
    remote_endpoint_ = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string("192.168.200.110"),  4000);
    cout<<"Listening to " <<   local_endpoint_ << endl;
    std::cout << "Send to " << remote_endpoint_ << std::endl;
    
    /*socket_.connect(remote_endpoint_, myError);
    cout<<"Connect status: "<<myError.message()<<endl;
    string data = "Hello world!";
    //socket_.send_to(boost::asio::buffer(data), remote_endpoint_);
    */
    start_receive();
  }
  
private:
  string buffer_; 
  void start_receive()
  {
    cout<<"Everything done"<<endl;
    string answer_str = add_check_sum(PONG);
    //socket_.send_to(boost::asio::buffer(data), remote_endpoint_);
    socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
			       boost::bind(&udp_server::handle_receive, this,
					   boost::asio::placeholders::error,
		      boost::asio::placeholders::bytes_transferred));
    
    //socket_.send_to(boost::asio::buffer(answer_str), remote_endpoint_);
  
    
  }
  
  void handle_receive(const boost::system::error_code& error,
		      std::size_t bytes_transferred)
  {
    cout<<"handle_receive status code: "<<error.message()<<endl;
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
	cout<<"PONG sent "<<answer_str<<endl;
        
         boost::shared_ptr<std::string> message(new std::string(answer_str));
        socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
          boost::bind(&udp_server::handle_send, this, message,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
        cout<<remote_endpoint_.address().to_string()<<":"<<remote_endpoint_.port()<<endl;
      }
      start_receive();
    }
  }
  
  void handle_send(boost::shared_ptr<std::string> message,
      const boost::system::error_code& error,
      std::size_t bytes_transferred)
  {
    cout<<"send handled"<<endl;
    cout<<*message<<" "<<error<<" "<<bytes_transferred<<endl;
  }
  
  udp::socket socket_;
  udp::endpoint remote_endpoint_, local_endpoint_;
  boost::array<char, 512> recv_buffer_;
};

int main()
{
  boost::system::error_code myError;

   /* boost::asio::ip::udp::endpoint myEndpoint;                  // Create endpoint on specified IP.
    myEndpoint.address(boost::asio::ip::address::from_string("192.168.200.101"));
    myEndpoint.port(4100);
    boost::asio::ip::udp::endpoint remoteEndpoint;                  // Create endpoint on specified IP.
    remoteEndpoint.address(boost::asio::ip::address::from_string("192.168.200.110"));
    remoteEndpoint.port(4000);
    std::cout << "Endpoint IP:   " << myEndpoint.address().to_string() << std::endl;
    std::cout << "Endpoint Port: " << myEndpoint.port() << std::endl;

    boost::asio::io_service io_service;                         // Create socket and IO service, bind socket to endpoint.
    udp::socket socket(io_service);
    socket.open( myEndpoint.protocol(), myError );
    std::cout << "Open - " << myError.message() << std::endl;
   
    char myMessage[] = "UDP Hello World!";                      // Send basig string, enable socket level debugging.
    socket.send_to(boost::asio::buffer(myMessage, sizeof(myMessage)), remoteEndpoint);
    
    std::cout << "Send - " << myError.message() << std::endl;
    return 0;*/
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
