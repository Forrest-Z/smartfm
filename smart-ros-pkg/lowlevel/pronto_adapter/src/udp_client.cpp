#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define MAX_LEN 1000

using boost::asio::ip::udp;
using namespace std;

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