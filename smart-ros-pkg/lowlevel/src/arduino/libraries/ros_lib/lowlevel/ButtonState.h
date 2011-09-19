#ifndef ros_ButtonState_h
#define ros_ButtonState_h

#include "WProgram.h"
#include "ros.h"

namespace lowlevel
{

  class ButtonState : public ros::Msg
  {
    public:
      bool emergency;
      bool automode;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_emergency;
      u_emergency.real = this->emergency;
      *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
      union {
        bool real;
        unsigned char base;
      } u_automode;
      u_automode.real = this->automode;
      *(outbuffer + offset + 0) = (u_automode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->automode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_emergency;
      u_emergency.base = 0;
      u_emergency.base |= ((typeof(u_emergency.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency = u_emergency.real;
      offset += sizeof(this->emergency);
      union {
        bool real;
        unsigned char base;
      } u_automode;
      u_automode.base = 0;
      u_automode.base |= ((typeof(u_automode.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->automode = u_automode.real;
      offset += sizeof(this->automode);
     return offset;
    }

    const char * getType(){ return "lowlevel/ButtonState"; };

  };

}
#endif