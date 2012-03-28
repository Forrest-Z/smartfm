/** Interface between ROS (user space) and HAL (kernel space). Sampling part:
 * reads the values from the DAQ card.
 *
 * Currently, only reads the state of the emergency state button. Publishes it
 * as a bool on topic "button_state_emergency".
 *
 * Based on sampler_usr.c from John Kasunich
 */

/********************************************************************
* Description:  sampler_usr.c
*               User space part of "sampler", a HAL component that
*               can be used to sample data from HAL pins and write
*               it to a file at a specific realtime sample rate.
*
* Author: John Kasunich <jmkasunich at sourceforge dot net>
* License: GPL Version 2
*
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/

/*  This program is free software; you can redistribute it and/or
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
#include <lowlevel_hal/odo.h>




/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

int comp_id = -1;	/* -1 means hal_init() not called yet */
int shmem_id = -1;
int exitval = 1;	/* program return code - 1 means error */
int ignore_sig = 0;	/* used to flag critical regions */
char comp_name[HAL_NAME_LEN];	/* name for this instance of sampler */

/***********************************************************************/

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

#define BUF_SIZE 4000

int channel, retval, size, tag;
long int samples;
unsigned long this_sample;
char *cp, *cp2;
void *shmem_ptr;
fifo_t *fifo;
shmem_data_t *data, *dptr, buf[MAX_PINS];
int tmpout, newout;
struct timespec delay;
ros::Publisher odo_pub, emergency_pub;
std_msgs::Bool emergency_msg;


void getOptions(int argc, char **argv)
{
    int n;

    /* set return code to "fail", clear it later if all goes well */
    exitval = 1;
    channel = 0;
    tag = 0;
    samples = -1;  /* -1 means run forever */

    /* FIXME - if I wasn't so lazy I'd learn how to use getopt() here */
    for ( n = 1 ; n < argc ; n++ ) {
        cp = argv[n];
        if ( *cp != '-' ) {
            break;
        }
        switch ( *(++cp) ) {
        case 'c':
            if (( *(++cp) == '\0' ) && ( ++n < argc )) {
                cp = argv[n];
            }
            channel = strtol(cp, &cp2, 10);
            if (( *cp2 ) || ( channel < 0 ) || ( channel >= MAX_SAMPLERS )) {
                fprintf(stderr,"ERROR: invalid channel number '%s'\n", cp );
                exit(1);
            }
            break;
        case 'n':
            if (( *(++cp) == '\0' ) && ( ++n < argc )) {
                cp = argv[n];
            }
            samples = strtol(cp, &cp2, 10);
            if (( *cp2 ) || ( samples < 0 )) {
                fprintf(stderr, "ERROR: invalid sample count '%s'\n", cp );
                exit(1);
            }
            break;
        case 't':
            tag = 1;
            break;
        default:
            fprintf(stderr,"ERROR: unknown option '%s'\n", cp );
            exit(1);
            break;
        }
    }

    if(n < argc) {
        int fd;
        if(argc > n+1) {
            fprintf(stderr, "ERROR: At most one filename may be specified\n");
            exit(1);
        }
        // make stdout be the named file
        fd = open(argv[n], O_WRONLY | O_CREAT, 0666);
        close(1);
        dup2(fd, 1);
    }
}

bool connectToHal()
{
    /* create a unique module name, to allow for multiple samplers */
    snprintf(comp_name, HAL_NAME_LEN-1, "halsampler%d", getpid());
    /* connect to the HAL */
    ignore_sig = 1;
    comp_id = hal_init(comp_name);
    ignore_sig = 0;
    /* check result */
    if (comp_id < 0) {
        fprintf(stderr, "ERROR: hal_init() failed: %d\n", comp_id );
        return false;
    }
    hal_ready(comp_id);

    /* open shmem for user/RT comms (fifo) */
    /* initial size is unknown, assume only the fifo structure */
    shmem_id = rtapi_shmem_new(SAMPLER_SHMEM_KEY+channel, comp_id, sizeof(fifo_t));
    if ( shmem_id < 0 ) {
        fprintf(stderr, "ERROR: couldn't allocate user/RT shared memory\n");
        return false;
    }
    retval = rtapi_shmem_getptr(shmem_id, &shmem_ptr);
    if ( retval < 0 ) {
        fprintf(stderr, "ERROR: couldn't map user/RT shared memory\n");
        return false;
    }
    fifo = (fifo_t*)shmem_ptr;
    if ( fifo->magic != FIFO_MAGIC_NUM ) {
        fprintf(stderr, "ERROR: channel %d realtime part is not loaded\n", channel );
        return false;
    }
    /* now use data in fifo structure to calculate proper shmem size */
    size = sizeof(fifo_t) + (1+fifo->num_pins) * fifo->depth * sizeof(shmem_data_t);
    /* close shmem, re-open with proper size */
    rtapi_shmem_delete(shmem_id, comp_id);
    shmem_id = rtapi_shmem_new(SAMPLER_SHMEM_KEY+channel, comp_id, size);
    if ( shmem_id < 0 ) {
        fprintf(stderr, "ERROR: couldn't re-allocate user/RT shared memory\n");
        return false;
    }
    retval = rtapi_shmem_getptr(shmem_id, &shmem_ptr);
    if ( retval < 0 ) {
        fprintf(stderr, "ERROR: couldn't re-map user/RT shared memory\n");
        return false;
    }
    fifo = (fifo_t*)shmem_ptr;
    data = fifo->data;
    return true;
}

bool waitForData()
{
    while ( fifo->in == fifo->out ) {
        /* fifo empty, sleep for 10mS */
        delay.tv_sec = 0;
        delay.tv_nsec = 10000000;
        nanosleep(&delay,NULL);
    }

    /* make pointer to fifo entry */
    tmpout = fifo->out;
    newout = tmpout + 1;
    if ( newout >= fifo->depth ) {
        newout = 0;
    }
    dptr = &data[tmpout * (fifo->num_pins+1)];
    /* read data from shmem into buffer */
    for ( int n = 0 ; n < fifo->num_pins ; n++ ) {
        buf[n] = *(dptr++);
    }
    /* and read sample number */
    this_sample = dptr->u;
    if ( fifo->out != tmpout ) {
        /* the sample was overwritten while we were reading it */
        /* so ignore it */
        return false;
    }
    else {
        /* update 'out' for next sample */
        fifo->out = newout;
    }

    if ( this_sample != ++(fifo->last_sample) ) {
        printf ( "overrun\n" );
        fifo->last_sample = this_sample;
    }
    if ( tag ) {
        printf ( "%ld ", this_sample );
    }

    return true;
}

bool readAndPublish()
{
    lowlevel_hal::odo odo_msg;
    bool emergency = false;

    printf("num_pins %d", fifo->num_pins);
    for ( int n = 0 ; n < fifo->num_pins ; n++ )
    {
        switch ( fifo->type[n] ) {
        case HAL_FLOAT:
            if(n==0) odo_msg.pose = buf[n].f;
            else if(n==1) odo_msg.vel = buf[n].f;
            else if(n==2) odo_msg.steering_angle = buf[n].f;
            break;
        case HAL_BIT:
            if(n==3) emergency = buf[n].b;
            break;
        case HAL_U32:
            printf ( "%lu ", (unsigned long)buf[n].u);
            break;
        case HAL_S32:
            printf ( "%ld ", (long)buf[n].s);
            break;
        default:
            /* better not happen */
            return false;
        }
    }

    printf ( "\n" );
    //ROS_INFO("Pose=%lf m, Vel=%lf m/s", odo_msg.pose,odo_msg.vel);
    odo_pub.publish(odo_msg);

    if( emergency!=emergency_msg.data) {
        emergency_msg.data = emergency;
        emergency_pub.publish(emergency_msg);
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "halsampler");
    ros::NodeHandle n_;
    odo_pub = n_.advertise<lowlevel_hal::odo>("halsampler", 1000);
    emergency_pub = n_.advertise<std_msgs::Bool>("button_state_emergency", 1000);

    /* register signal handlers - if the process is killed
       we need to call hal_exit() to free the shared memory */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGPIPE, quit);

    getOptions(argc,argv);

    if( !connectToHal() ) goto out;

    emergency_msg.data = false;

    while ( samples != 0 )
    {
        if( !waitForData() ) continue;
        if( !readAndPublish() ) goto out;
        if ( samples > 0 ) samples--;
    }
    /* run was succesfull */
    exitval = 0;

out:
    ignore_sig = 1;
    if ( shmem_id >= 0 ) rtapi_shmem_delete(shmem_id, comp_id);
    if ( comp_id >= 0 ) hal_exit(comp_id);

    return exitval;
}
