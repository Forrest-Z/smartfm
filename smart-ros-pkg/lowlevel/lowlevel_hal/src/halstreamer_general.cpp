/** Interface between ROS (user space) and HAL (kernel space). Streamer part:
 * writes the values to the DAQ card.
 *
 * Subscribes to "throttle", "brake_angle" and "steer_angle" and writes the
 * requested values on the DAQ card.
 *
 * Based on streamer_usr.c by John Kasunich
 */

/********************************************************************
* Description:  streamer_usr.c
*               User space part of "streamer", a HAL component that
*               can be used to stream data from a file onto HAL pins
*               at a specific realtime sample rate. This code is modified
*               from the original streamer_usr.c to allow command to be sent
*               using ROS messaging facility
*
* Author: John Kasunich <jmkasunich at sourceforge dot net>
* License: GPL Version 2
*
* Copyright (c) 2010 All rights reserved.
*
********************************************************************/


/* Note: to compile, use:
 *      sudo comp --compile --userspace streamer_gc.c
 *      which is a script that produce this compilation:
 *      gcc -Os -g -I. -I/usr/realtime-2.6.32-122-rtai/include -I.
 *         -I/usr/realtime-2.6.32-122-rtai/include -D_FORTIFY_SOURCE=0
 *         -ffast-math -mhard-float -DRTAI=3  -DRTAPI -D_GNU_SOURCE
 *         -Drealtime -D_FORTIFY_SOURCE=0 -D__MODULE__
 *         -I/usr/include/emc2 -URTAPI -U__MODULE__ -DULAPI -Os  -o
 *         streamer_gc /tmp/tmpt81zUr/streamer_gc.c -Wl,-rpath,/lib
 *         -L/lib -lemchal
 *      Yep, that long!
 *   Analysing the meaning:
 *     -Os: Optimize for size
 *     -g: Produce debugging information in the operating system's native format
 *     -Idir: Add the directory dir to the head of the list of directories to
 *        be searched for header files
 *     -D_FORTIFY_SOURCE: Provides compile-time best-practices errors for
 *        certain libc functions, and provides run-time checks of buffer lengths
 *        and memory regions
 *     -ffast-math: control optimisation
 *     -mhard-float: Generate output containing floating point instructions
 *     -DRTAI: no info
 *     and bla, bla, bla....
 * After trials, turns out only require this:
 *   gcc -I/usr/include/emc2 -DULAPI -o streamer_gc3 streamer_gc.c -lemchal
 *
 */

/* This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to www.linuxcnc.org.
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <rtapi.h> /* RTAPI realtime OS API */
#include <hal.h>   /* HAL public API decls */
#include <streamer.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <fmutil/fm_stopwatch.h>

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

int comp_id = -1;	/* -1 means hal_init() not called yet */
int shmem_id = -1;
int exitval = 1;	/* program return code - 1 means error */
int ignore_sig = 0;	/* used to flag critical regions */
int line=0;	/* used to print linenumber on errors */
int channel = 0; //default fifo channel

double speed_=0;
double steering_=0;
double brake_=0;

fifo_t *fifo;
std::vector<ros::Subscriber*> subs;
/***********************************************************************
*                           CALL FUNCTIONS                             *
************************************************************************/
#define BUF_SIZE 4000
/* signal handler */
static void quit(int sig)
{
    if ( ignore_sig ) {
        return;
    }
    if ( shmem_id >= 0 ) {
        rtapi_shmem_delete(shmem_id, comp_id);
    }
    if ( comp_id >= 0 ) {
        hal_exit(comp_id);
    }
    exit(exitval);
}

static void out()
{
    ignore_sig = 1;
    if ( shmem_id >= 0 ) {
        rtapi_shmem_delete(shmem_id, comp_id);
    }
    if ( comp_id >= 0 ) {
        hal_exit(comp_id);
    }
    exit(exitval);
}

void initialize_rtfifo()
{
    void *shmem_ptr; //pointer to the fifo address
    char comp_name[HAL_NAME_LEN];	/* name for this instance of streamer */
    int retval;
    exitval = 1; //set return code to "fail", clear it later if all goes well


    /* register signal handlers - if the process is killed
       we need to call hal_exit() to free the shared memory */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGPIPE, SIG_IGN);

    /* connect to HAL */
    /* create a unique module name, to allow for multiple streamers */
    snprintf(comp_name, HAL_NAME_LEN-1, "halstreamer%d", getpid());

    ignore_sig = 1;
    comp_id = hal_init(comp_name);
    ignore_sig = 0;

    /* check result */
    if (comp_id < 0) {
        fprintf(stderr, "ERROR: hal_init() failed: %d\n", comp_id );
        out();
    }
    hal_ready(comp_id);

    shmem_id = rtapi_shmem_new(STREAMER_SHMEM_KEY+channel, comp_id, sizeof(fifo_t));
    if ( shmem_id < 0 ) {
        fprintf(stderr, "ERROR: couldn't allocate user/RT shared memory\n");
        out();
    }
    retval = rtapi_shmem_getptr(shmem_id, &shmem_ptr);
    if ( retval < 0 ) {
        fprintf(stderr, "ERROR: couldn't map user/RT shared memory\n");
        out();
    }
    fifo =(fifo_t*)(shmem_ptr);
    if ( fifo->magic != FIFO_MAGIC_NUM ) {
        fprintf(stderr, "ERROR: channel %d realtime part is not loaded\n", channel );
        out();
    }
    /* now use data in fifo structure to calculate proper shmem size */
    int size = sizeof(fifo_t) + fifo->num_pins * fifo->depth * sizeof(shmem_data_t);
    /* close shmem, re-open with proper size */
    rtapi_shmem_delete(shmem_id, comp_id);
    shmem_id = rtapi_shmem_new(STREAMER_SHMEM_KEY+channel, comp_id, size);
    if ( shmem_id < 0 ) {
        fprintf(stderr, "ERROR: couldn't re-allocate user/RT shared memory\n");
        out();
    }
    retval = rtapi_shmem_getptr(shmem_id, &shmem_ptr);
    if ( retval < 0 ) {
        fprintf(stderr, "ERROR: couldn't re-map user/RT shared memory\n");
        out();
    }
    line = 1;
    fifo = (fifo_t*)(shmem_ptr);
}
using namespace std;

map<string, int> map_int;
map<string, float> map_float;
map<string, uint> map_uint;
map<string, int> map_bool;

template <class T>
string getTopicName(ros::MessageEvent<T> msg)
{
  ros::M_string header = msg.getConnectionHeader();
  string topic_name = header["topic"];
  return topic_name;
}

template <class T>
bool getPinNo(ros::MessageEvent<T> msg, string& pin_no)
{
  //it doesn't work!!!
  // http://answers.ros.org/question/68434/get-topic-name-in-callback/
  string topic = getTopicName(msg);
  cout<<"Topic: "<<topic<<endl;
  if(topic.size() == 0) return false;
  pin_no = topic.substr(topic.size()-1);
  return true;
}

bool getPinNo(string topic, string& pin_no)
{
  if(topic.size() == 0) return false;
  pin_no = topic.substr(topic.size()-1);
  return true;
}

void fifo_out(char *fifodata)
{
    shmem_data_t *data, *dptr;
    int tmpin, newin, n;
    struct timespec delay;
    const char *errmsg;
    char *cp, *cp2;

    data = fifo->data;
    tmpin = fifo->in;
    newin = tmpin + 1;
    if( newin >= fifo -> depth) {
        newin = 0;
    }

    /* wait until there is space in the buffer */
    while ( newin == fifo ->out) {
        /* fifo full, sleep for 10mS*/
        delay.tv_sec=0;
        delay.tv_nsec=10000000;
        nanosleep(&delay, NULL);
    }

    /* make pointer fifo entry */
    dptr = &data[tmpin*fifo->num_pins];

    errmsg = NULL;

    /* parsing data to fifo */
    cp = fifodata;
    //printf("%s",cp);
    for(n=0; n<fifo->num_pins; n++)
    {
        /* strip leading whitespace */
        while( isspace(*fifodata) ) fifodata++;

        switch (fifo->type[n]) {
        case HAL_FLOAT:
            dptr->f = strtod(cp, &cp2);
            break;
        case HAL_BIT:
            if ( *cp == '0' ) {
                dptr->b = 0;
                cp2 = cp + 1;
            } else if ( *cp == '1' ) {
                dptr->b = 1;
                cp2 = cp + 1;
            } else {
                errmsg = "bit value not 0 or 1";
                cp2 = cp;
            }
            break;
        case HAL_U32:
            dptr->u = strtoul(cp, &cp2, 10);
            break;
        case HAL_S32:
            dptr->s = strtol(cp, &cp2, 10);
            break;
        default:
            /* better not happen */
            out();
        }

        if ( errmsg == NULL ) {
            /* no error yet, check for other possibilties.
               whitespace separates fields, and there is a newline
               at the end... so if there is not space or newline at
               the end of a field, something is wrong. */
            if ( *cp2 == '\0' ) {
                errmsg = "premature end of line";
            } else if ( ! isspace(*cp2) ) {
                errmsg = "bad character";
            }
        }

        /* test for any error and abort loop on error */
        if ( errmsg != NULL ) break;

        /* advance pointers for next field */
        dptr++;
        cp = cp2+1;
    }

    if (errmsg != NULL) {
        fprintf(stderr,  "line %d, field %d: %s, skipping the line\n", line, n, errmsg );
        /** TODO - decide whether to skip this line and continue, or
        abort the program.  Right now it skips the line. */
    } else {
        /* good data, keep it */
        fifo->in = newin;
    }
    line++;
}

void writeFifo()
{
  stringstream data_ss;
  for (int n=0; n<fifo->num_pins; n++){
    stringstream num_pin_ss;
    num_pin_ss << n;
    string num_pin_str = num_pin_ss.str();
    switch(fifo->type[n]){
      case HAL_FLOAT:
	data_ss<<map_float[num_pin_str];
	break;
      case HAL_BIT:
	data_ss<<map_bool[num_pin_str];
	break;
      case HAL_U32:
	data_ss<<map_uint[num_pin_str];
	break;
      case HAL_S32:
	data_ss<<map_int[num_pin_str];
	break;
      default:
	std::cout<<"Unexpected pin type received, exiting..."<<std::endl;
	return;
    }
    if(n<fifo->num_pins-1) data_ss<<" ";
  }
  data_ss<<"\n";
  //cout<<data_ss.str()<<endl;
  char fifo_data[data_ss.str().size()];
  strcpy(fifo_data, data_ss.str().c_str());
  fifo_out(fifo_data);
}
//callback is slow by itself. It may due to near simultaneous publishing
//of topics from same node. This is very apparent in a slow processor.
void floatCallback(const std_msgs::Float32ConstPtr f, const std::string &topic)
{//cout<<topic<<endl;return;
  fmutil::Stopwatch sw(topic);
  string pin_no;
  if(getPinNo(topic,pin_no)){
    map_float[pin_no] = f->data;
    writeFifo();
  }
  sw.end(false);
  cout<<topic<<" "<<sw.total_<<" "<<f->data<<endl;
}

void bitCallback(const std_msgs::BoolConstPtr b, const std::string &topic)
{//cout<<topic<<endl;return;
  fmutil::Stopwatch sw(topic);
  string pin_no;
  if(getPinNo(topic,pin_no)){
    if(b->data)
      map_bool[pin_no] = 1;
    else
      map_bool[pin_no] = 0;
    writeFifo();
  }
  sw.end(false);
  cout<<topic<<" "<<sw.total_<<" "<<f->data<<endl;
}

void uintCallback(const std_msgs::UInt32ConstPtr u, const std::string &topic)
{
  string pin_no;
  if(getPinNo(topic,pin_no)){
    map_uint[pin_no] = u->data;
    writeFifo();
  }
}

void intCallback(const std_msgs::Int32ConstPtr i, const std::string &topic)
{
  string pin_no;
  if(getPinNo(topic,pin_no)){
    map_int[pin_no] = i->data;
    writeFifo();
  }
}

bool initializeSubs(ros::NodeHandle &nh)
{
  subs.resize(fifo->num_pins);
  for (int n=0; n<fifo->num_pins; n++){
    std::stringstream topic, num_pin_ss;
    num_pin_ss <<n;
    string num_pin_str = num_pin_ss.str();
    topic << "hal_streamer_";
    switch(fifo->type[n]){
      case HAL_FLOAT:
	topic << "float_"<<n;
	subs[n] = new ros::Subscriber(
	  nh.subscribe<std_msgs::Float32>(topic.str(), 1, boost::bind(floatCallback, _1, topic.str())));
	map_float[num_pin_str] = 0;
	break;
      case HAL_BIT:
	topic << "bit_"<<n;
	subs[n] = new ros::Subscriber(
	  nh.subscribe<std_msgs::Bool>(topic.str(), 1, boost::bind(bitCallback, _1, topic.str())));
	map_bool[num_pin_str] = false;
	break;
      case HAL_U32:
	topic << "uint_"<<n;
	subs[n] = new ros::Subscriber(
	  nh.subscribe<std_msgs::UInt32>(topic.str(), 1, boost::bind(uintCallback, _1, topic.str())));
	map_uint[num_pin_str] = 0;
	break;
      case HAL_S32:
	topic << "int_"<<n;
	subs[n] = new ros::Subscriber(
	  nh.subscribe<std_msgs::Int32>(topic.str(), 1, boost::bind(intCallback, _1, topic.str())));
	map_int[num_pin_str] = 0;
	break;
      default:
	std::cout<<"Unexpected pin type received, exiting..."<<std::endl;
	return false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
    ROS_INFO("Initializing fifo\n");
    initialize_rtfifo();

    ros::init(argc, argv, "halstreamer");
    ros::NodeHandle n;
    initializeSubs(n);
    ROS_INFO("Start spining");
    ros::spin();

    return 0;
}
