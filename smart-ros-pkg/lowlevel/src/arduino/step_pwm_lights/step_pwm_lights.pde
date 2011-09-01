/** step_pwm_lights.pde
-*- mode: C++ -*-

         1         2         3         4         5         6         7         8
12345678901234567890123456789012345678901234567890123456789012345678901234567890

Performs stepper motors control (brake and steering), PWM (throttle) and
lights control (blinkers and stop light). Also monitors the state of the
emergency button.

Subscribers:
- arduino_cmd (type: lowlevel::Arduino)

Publishers:
- emergency (type: std_msgs::Bool)



- Stepper motor control is done using the AccelStepper library:
   http://www.open.com.au/mikem/arduino/AccelStepper
   Number of pulses per seconds (velocity) is limited to approx 1,000
- PWM is a direct function of Arduino
- Lights control is with IOs
- Communication with ROS is handled by rosserial:
    http://www.ros.org/wiki/rosserial

Author: Brice Rebsamen (August 2011)
*/


#include <AccelStepper.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <lowlevel/Arduino.h>


//------------------------------------------------------------------------------
// Some parameters

// If defined, ROS is not used and a state machine is controlling the hardware.
//#define testing_mode

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
float n_steps_per_rev = 800;
float step_scale = n_steps_per_rev / 360.0;

//max speed (in steps per second). Becomes unreliable above 1000
//in steps per second per second
float brake_max_speed = n_steps_per_rev;
float brake_acc = 6000;
float steer_max_speed = n_steps_per_rev;
float steer_acc = 6000;

float full_brake = 220;

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
      if( _blink ) return; //already blinking
      digitalWrite(_pin, HIGH);
      _lastTime = millis();
      _blink = 1;
      _state = 1;
    }

    void off() {
      if( !_blink ) return; //already off
      digitalWrite(_pin, LOW);
      _blink = 0;
      _state = 0;
    }

    void set(int b) {
      b ? blink() : off();
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
    int _blink; // are we in blinking mode
    int _state; // is the light on or off
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

bool emergency = false;


//------------------------------------------------------------------------------
// Some helper functions

void set_throttle(float v) {
  // The golfcar is expecting a max of 3.3V (to be verified). The Arduino's
  // normal output is 5V. 2 solutions: use a voltage divider (2 resistors), or
  // limit the PWM duty cycle.
  // - voltage divider, might do a RC circuit which might slow down the pulse...
  // - scale: 255 ~ 5V ==>  168 ~ 3.3V
  v = v>3.3 ? 3.3 : v<0 ? 0 : v;
  int cycle = (int) ( v / 3.3 * 168 );
  if( emergency ) cycle = 0;
  analogWrite(throttle_pin, cycle);
}

void set_brake(float a) {
  if( emergency ) a = full_brake;
  stepper_brake.moveTo( (long) ( a * step_scale ) );
  if( a >= 5 )
    digitalWrite(stop_light_pin, HIGH);
  else
    digitalWrite(stop_light_pin, LOW);
}

void set_steer(float a) {
  stepper_steer.moveTo( (long) ( a * step_scale ) );
}


#ifndef testing_mode

//------------------------------------------------------------------------------
// Some callback functions for our subscribers.
//
// A callback function is defined using the ROS_CALLBACK macro which takes
// 3 parameters: the name of the callback function to create, the type of the
// message, and the name you wish to refer to the message by within the callback

ROS_CALLBACK(cmd_CB, lowlevel::Arduino, cmd_msg)
  set_throttle(cmd_msg.throttle_volt);
  set_steer(cmd_msg.steer_angle);
  set_brake(cmd_msg.brake_angle);
  left_blinker.set(cmd_msg.left_blinker);
  right_blinker.set(cmd_msg.right_blinker);
}


//------------------------------------------------------------------------------
// ROS stuffs

ros::NodeHandle nh;
ros::Subscriber arduino_cmd_sub("arduino_cmd", &cmd_msg, cmd_CB );

std_msgs::Bool emergency_msg;
ros::Publisher emergency_pub("emergency", &emergency_msg);


#endif


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
  emergency = digitalRead(emergency_pin)==LOW;
  
#ifndef testing_mode
  nh.initNode();
  nh.advertise(emergency_pub);
  nh.subscribe(arduino_cmd_sub);
#endif
}


unsigned long old_time = 0;
int state = 0;

void loop()
{
  stepper_brake.run();
  stepper_steer.run();
  
  left_blinker.run();
  right_blinker.run();

#if defined testing_mode
  unsigned long time = millis();
  if( time - old_time > 2000 ) {
    switch(state++) {
      case 0: set_brake(180); break;
      case 1: set_brake(0); break;
      case 2: set_steer(180); break;
      case 3: set_steer(0); break;
      case 4: set_brake(180); set_steer(-180); break;
      case 5: set_brake(0); set_steer(0); state=0; break;
    }
    old_time = time;
  }
#else
  if( !emergency && digitalRead(emergency_pin)==LOW ) {
    set_throttle(0);
    set_brake(full_brake);
    emergency = true;
  }
  else if( digitalRead(emergency_pin)==HIGH ) {
    emergency = false;
  }
  
  unsigned long time = millis();
  if( time - old_time > 250 ) {
    emergency_msg.data = emergency;
    emergency_pub.publish( &emergency_msg );
  }

  nh.spinOnce();

#endif
  
}