#!/usr/bin/env python
import roslib; roslib.load_manifest('oculus_viewer')
import rospy
from std_msgs.msg import Float64
from subprocess import Popen, PIPE
from decimal import Decimal

def timeOffset():
  pub = rospy.Publisher('timeOffset', Float64)
  rospy.init_node('timeOffset')
  host = rospy.get_param('~host', '203.123.48.218')
  fh = open("NUL","w")
  while not rospy.is_shutdown():
    process = Popen(['ntpdate', '-q', str(host)], stdout=PIPE, stderr=fh)
    stdout = process.communicate()[0]
    stdout = stdout.split(',')
    msg = Float64(999)
    if len(stdout) == 4 and 'offset' in stdout[3]:
      offset = stdout[2].split(' ')
      msg.data = Decimal(offset[2])
      #print(msg)
    else:
      print "Fail to get time offset from host", host, " Check that the host server is set in /etc/chrony/chrony.conf"
      rospy.sleep(5.0)
    pub.publish(msg)
    
if __name__ == '__main__':
  try:
    timeOffset()
  except rospy.ROSInterruptException:
    pass