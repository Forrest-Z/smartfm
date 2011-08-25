//step_pwm_lights.pde
// -*- mode: C++ -*-
//
//       1         2         3         4         5         6         7         8
//345678901234567890123456789012345678901234567890123456789012345678901234567890
//
// Performs stepper motors control (brake and steering), PWM (throttle) and
// lights control (blinkers and stop light).
//
// - Stepper motor control is done using the AccelStepper library:
//     http://www.open.com.au/mikem/arduino/AccelStepper
//     Number of pulses per seconds (velocity) is limited to approx 1,000
// - PWM is a direct function of Arduino
// - Lights control is with IOs
// - Communication with ROS is handled by rosserial:
//     http://www.ros.org/wiki/rosserial
//
// Author: Brice Rebsamen (August 2011)

#include <AccelStepper.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#define BOUND(x,m,M) ( (x)>(M) ? (M) : (x)<(m) ? (m) : (x) )


//------------------------------------------------------------------------------
// Some parameters

// pin connection
uint8_t left_blinker_pin = 2;
uint8_t right_blinker_pin = 3;
uint8_t stop_light_pin = 4;
uint8_t throttle_pin = 6;
uint8_t emergency_pin = 7;
uint8_t steer_pulse_pin = 8;
uint8_t steer_dir_pin = 9;
//uint8_t steer_enbl_pin = 10;
uint8_t brake_pulse_pin = 11;
uint8_t brake_dir_pin = 12;
//uint8_t brake_enbl_pin = 13;


// Angle to number of steps conversion factor
// 1440 pulses / rev
float step_scale = 160.0/360;

//max speed (in steps per second). Becomes unreliable above 1000
//in steps per second per second
float brake_max_speed = 1000;
float brake_acc = 100;
float steer_max_speed = 1000;
float steer_acc = 100;

float blinkers_default_half_period = 0.5;


//------------------------------------------------------------------------------
// Blinker class

class Blinker {

  public:
    Blinker(uint8_t pin) : _pin(pin), _blink(0), _state(0) {
      pinMode(pin, OUTPUT);
      half_period(blinkers_default_half_period);
    }
    
    void blink() {
      digitalWrite(_pin, HIGH);
      _lastTime = millis();
      _blink = 1;
      _state = 1;
    }
    
    void off() {
      digitalWrite(_pin, LOW);
      _blink = 0;
      _state = 0;
    }

    void half_period(float sec) {
      _half_period = (unsigned long) (sec*1000);
    }
    
    void run() {
      if( !_blink )
        return;
      unsigned long t = millis();
      if( t - _lastTime > _half_period ) {
        _lastTime = t;
        _state = !_state;
        if( _state )
          digitalWrite(_pin, HIGH);
        else
          digitalWrite(_pin, LOW);
      }
    }
    
  private:
    uint8_t _pin;
    int _blink, _state;
    unsigned long _lastTime;
    unsigned long _half_period; //in micro seconds
};


//------------------------------------------------------------------------------
// Some global variables

// Define the steppers and the pins they will use
// first argument: 1 means interface with a pulse-and-direction stepper
// the 2 following arguments indicate which pins to use for pulse and direction
// respectively. For direction, high means forward.
AccelStepper stepper_steer(1, steer_pulse_pin, steer_dir_pin);
AccelStepper stepper_brake(1, brake_pulse_pin, brake_dir_pin);

Blinker left_blinker(left_blinker_pin), right_blinker(right_blinker_pin);



//------------------------------------------------------------------------------
// Some callback functions for our subscribers.
//
// A callback function is defined using the ROS_CALLBACK macro which takes
// 3 parameters: the name of the callback function to create, the type of the
// message, and the name you wish to refer to the message by within the callback

ROS_CALLBACK(throttleCb, std_msgs::Float64, throttle_msg)
  // The golfcar is expecting a max of 3.3V (to be verified). The Arduino's
  // normal output is 5V. 2 solutions: use a voltage divider (2 resistors), or
  // limit the PWM duty cycle.
  // - voltage divider, might do a RC circuit which might slow down the pulse...
  // - scale: 255 ~ 5V ==>  168 ~ 3.3V
  int v = (int) (BOUND(throttle_msg.data, 0, 3.3) / 3.3 * 168);
  analogWrite(throttle_pin, v );
}

ROS_CALLBACK(brakeCb, std_msgs::Float64, brake_msg)
  stepper_brake.moveTo((long)(brake_msg.data*step_scale));
  if( brake_msg.data >= 5 )
    digitalWrite(stop_light_pin, HIGH);
  else
    digitalWrite(stop_light_pin, LOW);
}

ROS_CALLBACK(steerCb, std_msgs::Float64, steer_msg)
  stepper_steer.moveTo((long)(steer_msg.data*step_scale));
}

ROS_CALLBACK(leftBlinkerCb, std_msgs::Bool, leftBlinker_msg)
  if( leftBlinker_msg.data )
    left_blinker.blink();
  else
    left_blinker.off();
}

ROS_CALLBACK(rightBlinkerCb, std_msgs::Bool, rightBlinker_msg)
  if( rightBlinker_msg.data )
    right_blinker.blink();
  else
    right_blinker.off();
}


//------------------------------------------------------------------------------
// ROS stuffs

ros::NodeHandle nh;

ros::Subscriber throttle_sub("throttle_volt", &throttle_msg, throttleCb );
ros::Subscriber brake_sub("brake_angle", &brake_msg, brakeCb );
ros::Subscriber steer_sub("steer_angle", &steer_msg, steerCb );
ros::Subscriber lblink_sub("left_blinker", &leftBlinker_msg, leftBlinkerCb );
ros::Subscriber rblink_sub("right_blinker", &rightBlinker_msg, rightBlinkerCb );

std_msgs::Bool emergency_msg;
ros::Publisher emergency_pub("emergency", &emergency_msg);


//------------------------------------------------------------------------------
// Arduino functions

void setup()
{
  stepper_brake.setMaxSpeed(brake_max_speed);
  stepper_brake.setAcceleration(brake_acc);
  stepper_steer.setMaxSpeed(steer_max_speed);
  stepper_steer.setAcceleration(steer_acc);
  
  pinMode(throttle_pin, OUTPUT);
  analogWrite(throttle_pin, 0);
  
  pinMode(stop_light_pin, OUTPUT);
  digitalWrite(stop_light_pin, LOW);
  
  pinMode(emergency_pin, INPUT);
  digitalWrite(emergency_pin, HIGH); // turn on pullup resistors --> input defaults to HIGH
  
  nh.initNode();
  nh.subscribe(throttle_sub);
  nh.subscribe(brake_sub);
  nh.subscribe(steer_sub);
  nh.subscribe(lblink_sub);
  nh.subscribe(rblink_sub);
  nh.advertise(emergency_pub);
}


unsigned long old_time = 0;

void loop()
{
  stepper_brake.run();
  stepper_steer.run();
  
  left_blinker.run();
  right_blinker.run();

  unsigned long time = millis();
  if( time - old_time > 50 ) {
    emergency_msg.data = digitalRead(emergency_pin)==LOW;
    emergency_pub.publish( &emergency_msg );
    nh.spinOnce();
    old_time = time;
  }
}
