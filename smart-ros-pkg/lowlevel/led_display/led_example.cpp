#include <vector>
#include <iostream>
#include <iomanip>

#include <SerialStream.h>

typedef std::vector<char> Message;

char msg1_header[] = {0x00, 0x02, 0x31, 0x06, 0x00, 0x35, 0x31, 0x42};
char msg2_header[] = {0x02, 0x31, 0x06, 0x40}; // + 64 times 00
char msg3_header[] = {0x77, 0x02, 0x31, 0x06, 0x80}; // + 64 times 00
char msg4_header[] = {0xB7, 0x02, 0x31, 0x06, 0xC0}; // + 64 times 00
char msg5_header[] = {0xF7, 0x02, 0x33, 0x01};

#define LEN(arr) (sizeof(arr)/sizeof(char))


char checksum(const Message & m, int bgn, int end)
{
    int c=0;
    for(int i = bgn; i<end; ++i)
        c += m[i];
    return c%256;
}

/// Creates a message, given the string to display
///TODO: add some parameters to include the speed and mode, etc.
Message make_message(std::string m)
{
    Message bytes;
    unsigned i=0;

    // start with the first header
    for( i=0; i<LEN(msg1_header); i++ )
        bytes.push_back(msg1_header[i]);
    // add the number of characters in the message
    bytes.push_back(m.size());
    // add the actual message
    for( i=0; i<m.size(); i++ )
        bytes.push_back(m[i]);
    // pad with zeros to make 60 characters
    for( ; i<60; i++ )
        bytes.push_back(0x00);
    // add the checksum
    bytes.push_back( checksum(bytes, 2, bytes.size()) );

    // add the second message
    for( i=0; i<LEN(msg2_header); i++ )
        bytes.push_back(msg2_header[i]);
    for( i=0; i<64; i++ )
        bytes.push_back(0x00);

    // add the third message
    for( i=0; i<LEN(msg3_header); i++ )
        bytes.push_back(msg3_header[i]);
    for( i=0; i<64; i++ )
        bytes.push_back(0x00);

    // add the fourth message
    for( i=0; i<LEN(msg4_header); i++ )
        bytes.push_back(msg4_header[i]);
    for( i=0; i<64; i++ )
        bytes.push_back(0x00);

    // add the last message
    for( i=0; i<LEN(msg5_header); i++ )
        bytes.push_back(msg5_header[i]);

    return bytes;
}

int main()
{
    // Create and open the serial port for communication.
    LibSerial::SerialStream serial;
    serial.Open( "/dev/ttyUSB0" );
    if( ! serial.IsOpen() )
    {
        std::cerr <<"Error: could not open serial line" <<std::endl;
        //return 1;
    }
    serial.SetBaudRate( LibSerial::SerialStreamBuf::BAUD_38400 );

    Message bytes = make_message("Hello World");
    for(unsigned i=0; i<bytes.size(); i++) {
        serial <<bytes[i];
        std::cout <<std::setw(2) <<std::setfill('0') <<std::hex <<std::uppercase <<unsigned(bytes[i]) <<' ';
    }
    std::cout <<std::endl;

    serial.Close();
    return 0;
}
