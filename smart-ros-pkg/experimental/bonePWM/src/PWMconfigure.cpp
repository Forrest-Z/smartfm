/*
 * PWMconfigure.cpp
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "PWMconfigure.h"

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int pwm_set_duty(PWM_DEVICE device_flag, unsigned int value)
{
		std::cout << "Set PWM duty" << std::endl;
		
    int fd;
    char buf[MAX_BUF];

		if (device_flag == THROTTLE){
			snprintf(buf, sizeof(buf), SYSFS_THROTTLE_DUTY);
		}else if (device_flag == BUZZER){
			snprintf(buf, sizeof(buf), SYSFS_BUZZER_DUTY);
		}else{
			fprintf(stderr, "pwm_set_duty: device not found \n");
			return -1;
		}

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("pwm/set-duty");
        return fd;
    }
    
    char sValue[32];
		snprintf(sValue, sizeof(sValue), "%d", value);
		int count = strlen(sValue);
		int result = write(fd, sValue, count);

    if (result < 0){
			fprintf(stderr, "pwm_set_duty: open for file '%s' failed: %s \n", buf, strerror(errno));
			return -1;
		}else if (result == count){
			result = 0;
		}else{
			fprintf(stderr, "pwm_set_duty: short write on file '%s' \n", buf);
			result = -1;
		}
		
    close(fd);
    return result;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int pwm_get_duty(PWM_DEVICE device_flag, unsigned int *value)
{
    int fd;
    char buf[MAX_BUF];

		if (device_flag == THROTTLE){
			snprintf(buf, sizeof(buf), SYSFS_THROTTLE_DUTY);
		}else if (device_flag == BUZZER){
			snprintf(buf, sizeof(buf), SYSFS_BUZZER_DUTY);
		}else{
			fprintf(stderr, "pwm_set_duty: device not found \n");
			return -1;
		}

    fd = open(buf, O_RDONLY);
    if (fd < 0) {
			perror("pwm_get-duty: open failed");
			fprintf(stderr, "pwm_get-duty: open failed for file '%s' \n", buf);
			return -1;
    }
		lseek( fd, 0, SEEK_SET);
		char sResult[32];
		int result = read(fd, sResult, sizeof(sResult)-1);
		
    if (result < 0) {
			fprintf(stderr, "pwm_get_duty: read from '%s' failed: %s (%d) \n", buf, strerror(errno),errno);
    } else if (result > 0){
			*value = atoi(sResult);
			result = 0;
    } else {
			fprintf(stderr,"pwm_get_duty: short read on file '%s' \n",buf);
			result = -1;
		}

    close(fd);
    return result;
}
