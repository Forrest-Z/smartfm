/*
 * PWMconfigure.h
 */

#ifndef PWMCONFIGURE_H_
#define PWMCONFIGURE_H_

#include <string.h>

 /****************************************************************
 * Constants
 ****************************************************************/
#define SYSFS_THROTTLE_DUTY "/sys/devices/ocp.3/P8_19_PWM.12/duty"
#define SYSFS_BUZZER_DUTY "/sys/devices/ocp.3/P8_13_PWM.11/duty"
#define MAX_BUF 64

enum PWM_DEVICE{
	THROTTLE=0,
	BUZZER=1
};

int pwm_set_duty(PWM_DEVICE device_flag, unsigned int value);
int pwm_get_duty(PWM_DEVICE device_flag, unsigned int *value);

#endif /* PWMCONFIGURE_H_ */