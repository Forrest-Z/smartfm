/** step_pwm_lights.pde
-*- mode: C++ -*-

         1         2         3         4         5         6         7         8
12345678901234567890123456789012345678901234567890123456789012345678901234567890

Performs stepper motors control (brake and steering), PWM (throttle) and
lights control (blinkers and stop light). Also monitors the state of the
emergency and auto mode button.

Subscribers:
- arduino_cmd (type: lowlevel::Arduino)

Publishers:
- button_state (type: lowlevel::ButtonState)



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
#include <Blinkers.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <lowlevel/Arduino.h>
#include <lowlevel/ButtonState.h>


//------------------------------------------------------------------------------
// Some parameters

// If defined, ROS is not used and a state machine is controlling the hardware.

// pin connection
enum { emergency_pin = 2, throttle_pin,
  l293_3_pin, l293_2_pin, l293_4_pin, l293_1_pin,
  auto_mode_pin, steer_pulse_pin, steer_dir_pin, brake_pulse_pin, brake_dir_pin,
  motors_enable_pin };

uint8_t right_blinker_pin = l293_1_pin;
uint8_t left_blinker_pin = l293_2_pin;
uint8_t stop_light_pin = l293_3_pin;



// Angle to number of steps conversion factor
// alpha: 32767, beta: 5
float n_steps_per_rev = 800;
float step_scale = n_steps_per_rev / 360.0;

//max speed (in steps per second). Becomes unreliable above 1000
//in steps per second per second
float brake_max_speed = n_steps_per_rev;
float brake_acc = 6000;
float steer_max_speed = n_steps_per_rev;
float steer_acc = 6000;


float full_brake = 180;



//------------------------------------------------------------------------------
// Some global variables

// Define the steppers and the pins they will use
// first argument: 1 means interface with a pulse-and-direction stepper
// the 2 following arguments indicate which pins to use for pulse and direction
// respectively. For direction, high means forward.
AccelStepper stepper_steer(1, steer_pulse_pin, steer_dir_pin);
AccelStepper stepper_brake(1, brake_pulse_pin, brake_dir_pin);

Blinker left_blinker(left_blinker_pin), right_blinker(right_blinker_pin);

bool emergency = false, motorsEnabled = false;
unsigned long time = 0, old_time = 0, last_msg_time = 0;


//------------------------------------------------------------------------------
// Some helper functions

void set_throttle(float v) {
  // v is supposed to be between 0 and 1: 1 corresponds to max speed.

  if( ! emergency ) {
    int cycle = (int) ( (v>1.0 ? 1.0 : v<0.0 ? 0.0 : v) * 255.0 );
    analogWrite(throttle_pin, cycle);
  }
  else {
    analogWrite(throttle_pin, 0);
  }
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

void setMotorsEnabled(bool b) {
  digitalWrite(motors_enable_pin, b?HIGH:LOW);
  motorsEnabled = b;
}


//------------------------------------------------------------------------------
// Some callback functions for our subscribers.
//
// A callback function is defined using the ROS_CALLBACK macro which takes
// 3 parameters: the name of the callback function to create, the type of the
// message, and the name you wish to refer to the message by within the callback

ROS_CALLBACK(cmd_CB, lowlevel::Arduino, cmd_msg)
  last_msg_time = time;

  if( !motorsEnabled )
    setMotorsEnabled(true);

  set_throttle(cmd_msg.throttle);
  set_steer(cmd_msg.steer_angle);
  set_brake(cmd_msg.brake_angle);

  if( emergency ) {
    left_blinker.off();
    left_blinker.blink();
    right_blinker.off();
    right_blinker.blink();
  } else {
    left_blinker.set(cmd_msg.left_blinker);
    right_blinker.set(cmd_msg.right_blinker);
  }
}


//------------------------------------------------------------------------------
// ROS stuffs

ros::NodeHandle nh;
ros::Subscriber arduino_cmd_sub("arduino_cmd", &cmd_msg, cmd_CB );

lowlevel::ButtonState button_state_msg;
ros::Publisher button_state_pub("button_state", &button_state_msg);



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

  pinMode(motors_enable_pin, OUTPUT);
  digitalWrite(motors_enable_pin, LOW);

  pinMode(emergency_pin, INPUT);
  pinMode(auto_mode_pin, INPUT);
  emergency = digitalRead(emergency_pin)==HIGH;
  button_state_msg.automode = digitalRead(auto_mode_pin)==HIGH;

  nh.initNode();
  nh.advertise(button_state_pub);
  nh.subscribe(arduino_cmd_sub);
}

void button_state()
{
  button_state_msg.automode = digitalRead(auto_mode_pin)==HIGH;

  if( !emergency && digitalRead(emergency_pin)==HIGH ) {
    set_throttle(0);
    set_brake(full_brake);
    emergency = true;
    button_state_msg.emergency = emergency;
    button_state_pub.publish( &button_state_msg );
    old_time = time;
  }
  else if( emergency && digitalRead(emergency_pin)==LOW ) {
    emergency = false;
    set_brake(0);
    button_state_msg.emergency = emergency;
    button_state_pub.publish( &button_state_msg );
    old_time = time;
  }
  else {
    if( time - old_time > 250 ) {
      button_state_msg.emergency = emergency;
      button_state_pub.publish( &button_state_msg );
      old_time = time;
    }
  }
}


void loop()
{
  time = millis();
  button_state();
  nh.spinOnce();

  if( motorsEnabled && (time > last_msg_time + 5000) )
    setMotorsEnabled(false);


  if( motorsEnabled ) {
    stepper_brake.run();
    stepper_steer.run();
  }

  left_blinker.run();
  right_blinker.run();
}
