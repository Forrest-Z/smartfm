#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <ros/ros.h>
#include <can_reader/CanMsg.h>

int i = 0;
unsigned char willExit = 0;
int last;
time_t last_time = 0;


void sighand (int sig)
{
  switch (sig) {
  case SIGINT:
    willExit = 1;
    alarm(0);
    break;
  }
}

 
int main (int argc, char *argv[])
{
  ros::init(argc,argv,"can_reader");
  ros::NodeHandle nh;
  ros::Publisher can_msg_pub = nh.advertise<can_reader::CanMsg>("can_msg",100);
  can_reader::CanMsg can_msg;
  canHandle h;
  int ret = -1;
  long id; 
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long t;
  int channel = 0;
  int bitrate = BAUD_500K;
  int j;

  errno = 0;
  channel = 0;
  printf("Reading messages on channel %d\n", channel);
  /*
  if (argc != 2 || (channel = atoi(argv[1]), errno) != 0) {
    printf("usage %s channel\n", argv[0]);
    exit(1);
  } else {
    printf("Reading messages on channel %d\n", channel);
  }

  */
  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);
  alarm(1);

  /* Allow signals to interrupt syscalls(in canReadBlock) */
  siginterrupt(SIGINT, 1);
  
  /* Open channels, parameters and go on bus */
  h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED);
  if (h < 0) {
    printf("canOpenChannel %d failed\n", channel);
    printf("press anykey to quit ... \n");
    getchar();
    return -1;
  }
    
  canSetBusParams(h, bitrate, 4, 3, 1, 1, 0);
  canSetBusOutputControl(h, canDRIVER_NORMAL);
  canBusOn(h);
  printf("Can opened !!!");

  i = 0;
  while (!willExit) {
     
    do { 
      ret = canReadWait(h, &id, &msg, &dlc, &flag, &t, -1);
      switch (ret) {
      case 0:
    	can_msg.header.stamp = ros::Time::now();
    	can_msg.header.frame_id = "";
    	can_msg.header.seq = i;
    	can_msg.id = id;
    	can_msg.dlc = dlc;
    	can_msg.flag = flag;
    	can_msg.time = t;
    	can_msg.data.resize(dlc);
    	for(unsigned int data_ind = 0; data_ind < dlc; data_ind++)
    	{
    		can_msg.data[data_ind] = msg[data_ind];
    	}
    	can_msg_pub.publish(can_msg);
        printf("(%d) id:%ld dlc:%d data: ", i, id, dlc);
        if (dlc > 8) {
          dlc = 8;
        }
        for (j = 0; j < dlc; j++){
          printf("%2.2x ", msg[j]);
        }
        printf(" flags:0x%x time:%ld\n", flag, t);
        i++;
	if (last_time == 0) {
	  last_time = time(0);
	} else if (time(0) > last_time) {
	  last_time = time(0);
	  if (i != last) {
	    printf("rx : %d total: %d\n", i - last, i);
	  }
	  last = i;
	}
        break;
      case canERR_NOMSG:
        break;
      default:
        perror("canReadBlock error");
        break;
      }
    } while (ret == canOK);
    willExit = 1;
  }
   
  canClose(h);
  printf("Can closed !!!");
  sighand(SIGALRM);
  printf("Ready\n");

  return 0;
}

