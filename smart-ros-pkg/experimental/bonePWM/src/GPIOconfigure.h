/*
 * GPIOconfigure.h
 *
 * Modifications by Tawit, Singapore MIT Alliance in Research and Technology, Singapore
 * Almost entirely based of Software by Derek Molloy, School of Electronic Engineering, DCU
 * www.derekmolloy.ie
 *
*/

#ifndef GPIOCONFIGURE_H_
#define GPIOCONFIGURE_H_

 /****************************************************************
 * Constants
 ****************************************************************/

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

/* --- List of GPIO pins to be use --- */
// Output pins
const unsigned int Button_1_LED = 68;
const unsigned int Button_2_LED = 45;
const unsigned int Button_3_LED = 44;
const unsigned int EStop_1_LED = 27;
const unsigned int EStop_2_LED = 86;
const unsigned int ERemote_LED = 88;

// Input pins
const unsigned int Button_1_Input = 63;
const unsigned int Button_2_Input = 62;
const unsigned int Button_3_Input = 37;
const unsigned int EStop_1_Input = 36;
const unsigned int EStop_2_Input = 33;
const unsigned int ERemote_Input = 32;

// Data input, output enable pins
const unsigned int Data_Output_Enable = 73;
const unsigned int Data_Input_Enable = 70;

//Signal selection pins
const unsigned int Buzzer_select = 38;
const unsigned int Reverse_select = 39;
const unsigned int Forward_select = 34;
const unsigned int Accel_select = 66;
const unsigned int Throttle_select = 67;

//Signal pins
const unsigned int Reverse_signal = 26;
const unsigned int Forward_signal = 47;
const unsigned int Accel_signal = 46;

enum PIN_DIRECTION{
    INPUT_PIN=0,
    OUTPUT_PIN=1
};

enum PIN_VALUE{
    LOW=0,
    HIGH=1
};

/****************************************************************
 * gpio_initialize
 ****************************************************************/
void gpio_init(void);

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag);
int gpio_set_value(unsigned int gpio, PIN_VALUE value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);

#endif /* GPIOCONFIGURE_H_ */

