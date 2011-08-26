#ifndef ros_Arduino_h
#define ros_Arduino_h

#include "WProgram.h"
#include "ros.h"

namespace lowlevel
{

  class Arduino : public ros::Msg
  {
    public:
      float throttle_volt;
      float brake_angle;
      float steer_angle;
      bool left_blinker;
      bool right_blinker;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * val_throttle_volt = (long *) &(this->throttle_volt);
      long exp_throttle_volt = (((*val_throttle_volt)>>23)&255);
      if(exp_throttle_volt != 0)
        exp_throttle_volt += 1023-127;
      long sig_throttle_volt = *val_throttle_volt;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_throttle_volt<<5) & 0xff;
      *(outbuffer + offset++) = (sig_throttle_volt>>3) & 0xff;
      *(outbuffer + offset++) = (sig_throttle_volt>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_throttle_volt<<4) & 0xF0) | ((sig_throttle_volt>>19)&0x0F);
      *(outbuffer + offset++) = (exp_throttle_volt>>4) & 0x7F;
      if(this->throttle_volt < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_brake_angle = (long *) &(this->brake_angle);
      long exp_brake_angle = (((*val_brake_angle)>>23)&255);
      if(exp_brake_angle != 0)
        exp_brake_angle += 1023-127;
      long sig_brake_angle = *val_brake_angle;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_brake_angle<<5) & 0xff;
      *(outbuffer + offset++) = (sig_brake_angle>>3) & 0xff;
      *(outbuffer + offset++) = (sig_brake_angle>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_brake_angle<<4) & 0xF0) | ((sig_brake_angle>>19)&0x0F);
      *(outbuffer + offset++) = (exp_brake_angle>>4) & 0x7F;
      if(this->brake_angle < 0) *(outbuffer + offset -1) |= 0x80;
      long * val_steer_angle = (long *) &(this->steer_angle);
      long exp_steer_angle = (((*val_steer_angle)>>23)&255);
      if(exp_steer_angle != 0)
        exp_steer_angle += 1023-127;
      long sig_steer_angle = *val_steer_angle;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_steer_angle<<5) & 0xff;
      *(outbuffer + offset++) = (sig_steer_angle>>3) & 0xff;
      *(outbuffer + offset++) = (sig_steer_angle>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_steer_angle<<4) & 0xF0) | ((sig_steer_angle>>19)&0x0F);
      *(outbuffer + offset++) = (exp_steer_angle>>4) & 0x7F;
      if(this->steer_angle < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        bool real;
        unsigned char base;
      } u_left_blinker;
      u_left_blinker.real = this->left_blinker;
      *(outbuffer + offset + 0) = (u_left_blinker.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_blinker);
      union {
        bool real;
        unsigned char base;
      } u_right_blinker;
      u_right_blinker.real = this->right_blinker;
      *(outbuffer + offset + 0) = (u_right_blinker.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_blinker);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned long * val_throttle_volt = (unsigned long*) &(this->throttle_volt);
      offset += 3;
      *val_throttle_volt = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_throttle_volt |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_throttle_volt |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_throttle_volt |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_throttle_volt = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_throttle_volt |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_throttle_volt !=0)
        *val_throttle_volt |= ((exp_throttle_volt)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->throttle_volt = -this->throttle_volt;
      unsigned long * val_brake_angle = (unsigned long*) &(this->brake_angle);
      offset += 3;
      *val_brake_angle = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_brake_angle |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_brake_angle |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_brake_angle |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_brake_angle = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_brake_angle |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_brake_angle !=0)
        *val_brake_angle |= ((exp_brake_angle)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->brake_angle = -this->brake_angle;
      unsigned long * val_steer_angle = (unsigned long*) &(this->steer_angle);
      offset += 3;
      *val_steer_angle = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);
      *val_steer_angle |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_steer_angle |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_steer_angle |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;
      unsigned long exp_steer_angle = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;
      exp_steer_angle |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_steer_angle !=0)
        *val_steer_angle |= ((exp_steer_angle)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->steer_angle = -this->steer_angle;
      union {
        bool real;
        unsigned char base;
      } u_left_blinker;
      u_left_blinker.base = 0;
      u_left_blinker.base |= ((typeof(u_left_blinker.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_blinker = u_left_blinker.real;
      offset += sizeof(this->left_blinker);
      union {
        bool real;
        unsigned char base;
      } u_right_blinker;
      u_right_blinker.base = 0;
      u_right_blinker.base |= ((typeof(u_right_blinker.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_blinker = u_right_blinker.real;
      offset += sizeof(this->right_blinker);
     return offset;
    }

    const char * getType(){ return "lowlevel/Arduino"; };

  };

}
#endif