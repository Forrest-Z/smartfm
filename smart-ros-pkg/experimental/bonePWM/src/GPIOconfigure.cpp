/*
 * GPIOconfigure.cpp
 *
 * Modifications by Tawit, Singapore MIT Alliance in Research and Technology, Singapore
 * Almost entirely based of Software by Derek Molloy, School of Electronic Engineering, DCU
 * www.derekmolloy.ie
 * 
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "GPIOconfigure.h"

void gpio_init(void){
	std::cout << "Initialize GPIO ..." << std::endl;
	// --- GPIOs --- //
	//Export pins and set as Output
	std::cout << "Export pins and set as Output ..." << std::endl;
	gpio_export(Button_1_LED);
	gpio_export(Button_2_LED);
	gpio_export(Button_3_LED);
	gpio_export(EStop_1_LED);
	gpio_export(EStop_2_LED);
	gpio_export(ERemote_LED);
	gpio_export(Data_Output_Enable);
	gpio_export(Data_Input_Enable);
	gpio_export(Buzzer_select);
	gpio_export(Reverse_select);
	gpio_export(Forward_select);
	gpio_export(Accel_select);
	gpio_export(Throttle_select);
	gpio_export(Reverse_signal);
	gpio_export(Forward_signal);
	gpio_export(Accel_signal);
	
	gpio_set_dir(Button_1_LED, OUTPUT_PIN);
	gpio_set_dir(Button_2_LED, OUTPUT_PIN);
	gpio_set_dir(Button_3_LED, OUTPUT_PIN);
	gpio_set_dir(EStop_1_LED, OUTPUT_PIN);
	gpio_set_dir(EStop_2_LED, OUTPUT_PIN);
	gpio_set_dir(ERemote_LED, OUTPUT_PIN);
	gpio_set_dir(Data_Output_Enable, OUTPUT_PIN);
	gpio_set_dir(Data_Input_Enable, OUTPUT_PIN);
	gpio_set_dir(Buzzer_select, OUTPUT_PIN);
	gpio_set_dir(Reverse_select, OUTPUT_PIN);
	gpio_set_dir(Forward_select, OUTPUT_PIN);
	gpio_set_dir(Accel_select, OUTPUT_PIN);
	gpio_set_dir(Throttle_select, OUTPUT_PIN);
	gpio_set_dir(Reverse_signal, OUTPUT_PIN);
	gpio_set_dir(Forward_signal, OUTPUT_PIN);
	gpio_set_dir(Accel_signal, OUTPUT_PIN);
	
	//Export pins and set as Input
	std::cout << "Export pins and set as Input ..." << std::endl;
	gpio_export(Button_1_Input);
	gpio_export(Button_2_Input);
	gpio_export(Button_3_Input);
	gpio_export(EStop_1_Input);
	gpio_export(EStop_2_Input);
	gpio_export(ERemote_Input);
	
	gpio_set_dir(Button_1_Input, INPUT_PIN);
	gpio_set_dir(Button_2_Input, INPUT_PIN);
	gpio_set_dir(Button_3_Input, INPUT_PIN);
	gpio_set_dir(EStop_1_Input, INPUT_PIN);
	gpio_set_dir(EStop_2_Input, INPUT_PIN);
	gpio_set_dir(ERemote_Input, INPUT_PIN);
	
	//Set value of each output pins
	std::cout << "Set value of each output pins ..." << std::endl;
	gpio_set_value(Data_Output_Enable, HIGH);		//Disable data output
	gpio_set_value(Data_Input_Enable, HIGH);			//Disable data input
	
	gpio_set_value(Button_1_LED, HIGH);
	gpio_set_value(Button_2_LED,HIGH);
	gpio_set_value(Button_3_LED,HIGH);
	gpio_set_value(EStop_1_LED,HIGH);
	gpio_set_value(EStop_2_LED,HIGH);
	gpio_set_value(ERemote_LED,HIGH);
	
	gpio_set_value(Buzzer_select, LOW);
	gpio_set_value(Reverse_select, LOW);
	gpio_set_value(Forward_select, LOW);
	gpio_set_value(Accel_select, LOW);
	gpio_set_value(Throttle_select, LOW);
	
	gpio_set_value(Reverse_signal, HIGH);
	gpio_set_value(Forward_signal, LOW);
	gpio_set_value(Accel_signal, LOW);
	
	std::cout << "### Finish initialize GPIO ###" << std::endl;
	//After finish initialize, enable data input and output
	gpio_set_value(Data_Output_Enable, LOW);
	gpio_set_value(Data_Input_Enable, LOW);
}

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
    int fd, len;
    char buf[MAX_BUF];

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        perror("gpio/export");
        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);

    return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
    int fd, len;
    char buf[MAX_BUF];

    fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fd < 0) {
        perror("gpio/export");
        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);
    return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("gpio/direction");
        return fd;
    }

    if (out_flag == OUTPUT_PIN)
        write(fd, "out", 4);
    else
        write(fd, "in", 3);

    close(fd);
    return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, PIN_VALUE value)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("gpio/set-value");
        return fd;
    }

    if (value==LOW)
        write(fd, "0", 2);
    else
        write(fd, "1", 2);

    close(fd);
    return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
    int fd;
    char buf[MAX_BUF];
    char ch;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY);
    if (fd < 0) {
        perror("gpio/get-value");
        return fd;
    }

    read(fd, &ch, 1);

    if (ch != '0') {
        *value = 1;
    } else {
        *value = 0;
    }

    close(fd);
    return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int gpio_set_edge(unsigned int gpio, char *edge)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("gpio/set-edge");
        return fd;
    }

    write(fd, edge, strlen(edge) + 1);
    close(fd);
    return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY | O_NONBLOCK );
    if (fd < 0) {
        perror("gpio/fd_open");
    }
    return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
    return close(fd);
}
