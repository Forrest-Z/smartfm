#include "socket_handler/ServerSocket.hh"
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
int main ( int argc, int argv[] )
{
  std::cout << "running....\n";
  // Create the listening socket
  ServerSocket server ( 8888 );
  int customerID = 1;

  while ( true )
  {    //create the conversational socket
    ServerSocket new_sock1; //, new_sock2;
    // wait for a client connection
    server.accept ( new_sock1 );
    //server.accept ( new_sock2 );
    std::string data1; //, data2;
    new_sock1 >> data1;
    std::cout << "Received: " << data1 << std::endl;

    int customerID, pickup, dropoff;
    
    while ( true )
    {  // read the string and write it back
      printf ("  Enter customer ID: "); 
      scanf ("%d",&customerID); 
      printf ("  Enter the pick-up location: "); 
      scanf ("%d",&pickup); 
      printf ("  Enter the drop-off location: "); 
      scanf ("%d",&dropoff);
      std::ostringstream ss;
	  //ss << ";" << customerID << ":1:" << pickup << ":" << dropoff << "\n";
	  ss << ";" << customerID << ":" << pickup << ":" << dropoff << "\n";
      new_sock1 << ss.str();

      new_sock1 >> data1;
      std::cout << "Received: " << data1 << std::endl;
      /*
      new_sock1 << "I got your message: " << data1;
      //new_sock2 >> data2;
      //std::cout << "Received: " << data2 << std::endl;
      //new_sock2 << "I got your message: " << data2;
      */
      customerID++;
    }
  }

  return 0;
}
