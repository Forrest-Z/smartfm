#!/usr/bin/python
import subprocess
PACKAGE='read_network_usage'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
import re
from std_msgs.msg import Float32
from read_network_usage.msg import *




def bandwidth_pub():
	pub = rospy.Publisher('network_usage', network_usage_msg)
	rospy.init_node('read_network_usage')
	script_path=rospy.get_param('~script_path', '/home/sxt/smartfm/smartfm/smart-ros-pkg/experimental/read_network_usage/src')
	network_port=rospy.get_param('~network_port','wlan0')
	proc=subprocess.Popen([script_path+'/netspeed.sh', network_port], stdout=subprocess.PIPE)
	seq = 0;
	while True and not rospy.is_shutdown():
		line=proc.stdout.readline()
		if line!='':
			print "test:", line.rstrip()
			data=map(int, re.findall('\d+', line.rstrip()))
			usage_msg = network_usage_msg()
			usage_msg.header.stamp = rospy.get_rostime()
			usage_msg.header.seq = seq
			usage_msg.rx_speed = data[1]/1000.0*2
			usage_msg.tx_speed = data[0]/1000.0*2
			seq = seq + 1
			pub.publish(usage_msg)
		else:
			break


if __name__ =='__main__':
	try:
		bandwidth_pub()
	except rospy.ROSInterruptException:
		pass



