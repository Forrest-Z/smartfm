/*
 * ControlApplication.cpp
 *
 * Modifications by Tawit, Singapore MIT Alliance in Research and Technology, Singapore
 * Alomost entirely based of Software by Derek Molloy, School of Electronic Engineering, DCU
 * www.derekmolloy.ie
 *
 * YouTube Channel: http://www.youtube.com/derekmolloydcu/
 * 
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "GPIOconfigure.h"
#include "PWMconfigure.h"
#include "ControlApplication.h"

unsigned int THROTTLE_value;
unsigned int BUZZER_value;
unsigned int Button_1_Input_value;
unsigned int Button_2_Input_value;
unsigned int Button_3_Input_value;
unsigned int curr_Mode;
unsigned int prev_Mode;

void forwardReverseCallback(std_msgs::Bool forwardReverse){
  if(forwardReverse.data){//Forward
 		std::cout << "Forward" << std::endl;
		gpio_set_value(Reverse_signal, HIGH);
		gpio_set_value(Forward_signal, LOW);
  }
  else {//Reverse
		std::cout << "Reverse" << std::endl;
		gpio_set_value(Reverse_signal, LOW);
		gpio_set_value(Forward_signal, HIGH);
  }
}

void setThrottle(std_msgs::Int32 throttle){ 
	std::cout<<"Set throttle "<<throttle.data<<std::endl;
	pwm_set_duty(THROTTLE,throttle.data);
}

void setBuzzer(std_msgs::Int32 buzzer){ 
	std::cout<<"Set buzzer "<<buzzer.data<<std::endl;
	pwm_set_duty(BUZZER,buzzer.data);
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "ControlApplication");
	std::cout << "Control application to test GPIO and PWM pins" << std::endl;
	gpio_init();
	
	ros::NodeHandle nh;
	ros::Subscriber forwardReverseSub = nh.subscribe("forward_reverse", 1, &forwardReverseCallback);
	ros::Subscriber throttleSub = nh.subscribe("throttle_duty", 1, &setThrottle);
	ros::Subscriber buzzerSub = nh.subscribe("buzzer_duty", 1, &setBuzzer);
	ros::Publisher throttlePub = nh.advertise<std_msgs::Int32>("throttle_feedback", 1);
	ros::Publisher buzzerPub = nh.advertise<std_msgs::Int32>("buzzer_feedback", 1);
	
	//Set LED status
	gpio_set_value(Button_1_LED,HIGH);
	gpio_set_value(Button_2_LED,HIGH);
	gpio_set_value(Button_3_LED,HIGH);
	
	//default to manual mode
	curr_Mode = 6;
	prev_Mode = 6;
	
	pwm_set_duty(THROTTLE,500000);
	pwm_set_duty(BUZZER,500000);
	
	ros::Rate loop(100);
	while(ros::ok()){
		gpio_get_value(Button_1_Input,&Button_1_Input_value);
		gpio_get_value(Button_2_Input,&Button_2_Input_value);
		gpio_get_value(Button_3_Input,&Button_3_Input_value);
		
// 		std::cout << "Button_1_Input: " << Button_1_Input_value 
// 		<< "  ,Button_2_Input: " << Button_2_Input_value
// 		<< "  ,Button_3_Input: " << Button_3_Input_value << std::endl;
		
		mode_select(Button_1_Input_value,Button_2_Input_value,Button_3_Input_value);
		
		unsigned int THROTTLE_value, BUZZER_value;/*
		pwm_get_duty(THROTTLE, &THROTTLE_value);
		pwm_get_duty(BUZZER, &BUZZER_value);*/
		
		std_msgs::Int32 throttle_msg, buzzer_msg;
		throttle_msg.data = THROTTLE_value;
		buzzer_msg.data = BUZZER_value;
		throttlePub.publish(throttle_msg);
		buzzerPub.publish(buzzer_msg);
		
		//std::cout << "Throttle value: " << THROTTLE_value << std::endl;
		ros::spinOnce();
		loop.sleep();
	}
	
	std::cout << "Good bye Dave, have a good day ..." << std::endl;
	return 0;
}

int mode_select(unsigned int button_1_value,unsigned int button_2_value,unsigned int button_3_value){
	//button_x_value = 0 when button is pressed and 1 when released
	unsigned int mode_mux = (button_1_value * 1) + (button_2_value * 2) + (button_3_value * 4);
	
	if(mode_mux == 7){
		curr_Mode = prev_Mode;
	}else{
		curr_Mode = mode_mux;
	}
	
	switch (curr_Mode){
		case (0):
			//do something
			break;
		
		case (1):
		//do something
			break;
		
		case (2):
		//do something
			break;
		
		case (3):
		//Semi-Autonomous mode
			gpio_set_value(Buzzer_select, LOW);// LOW -- not select
			gpio_set_value(Reverse_select, LOW);
			gpio_set_value(Forward_select, LOW);
			gpio_set_value(Accel_select, HIGH);
			gpio_set_value(Throttle_select, HIGH);
			pwm_set_duty(BUZZER, 0);
			
			//Set LED status
			gpio_set_value(Button_1_LED,HIGH);
			gpio_set_value(Button_2_LED,HIGH);
			gpio_set_value(Button_3_LED,LOW);
			break;
		
		case (4):
		//do something
			break;
		
		case (5):
			//Autonomous mode
			gpio_set_value(Buzzer_select, LOW);
			gpio_set_value(Reverse_select, HIGH);
			gpio_set_value(Forward_select, HIGH);
			gpio_set_value(Accel_select, HIGH);
			gpio_set_value(Throttle_select, HIGH);
			
			gpio_set_value(Accel_signal, LOW);//enable solenoid
			
			//Set LED status
			gpio_set_value(Button_1_LED,HIGH);
			gpio_set_value(Button_2_LED,LOW);
			gpio_set_value(Button_3_LED,HIGH);
			break;
		
		case (6):
			//Manual mode
			gpio_set_value(Buzzer_select, LOW);
			gpio_set_value(Reverse_select, LOW);
			gpio_set_value(Forward_select, LOW);
			gpio_set_value(Accel_select, LOW);
			gpio_set_value(Throttle_select, LOW);
			
			gpio_set_value(Accel_signal, HIGH);//disable solenoid
			//Set LED status
			gpio_set_value(Button_1_LED,LOW);
			gpio_set_value(Button_2_LED,HIGH);
			gpio_set_value(Button_3_LED,HIGH);
			break;
		
		case (7):
		//do nothing, no button is pressed
			std::cout << "This should not happen!" << std::endl;
			break;
		
		default:
			//Manual mode
			gpio_set_value(Buzzer_select, LOW);
			gpio_set_value(Reverse_select, LOW);
			gpio_set_value(Forward_select, LOW);
			gpio_set_value(Accel_select, LOW);
			gpio_set_value(Throttle_select, LOW);
			
			//Set LED status
			gpio_set_value(Button_1_LED,LOW);
			gpio_set_value(Button_2_LED,HIGH);
			gpio_set_value(Button_3_LED,HIGH);
			break;

	}
	
	//update mode status
	prev_Mode = curr_Mode;
	return 0;	
}