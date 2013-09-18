/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "rosrt/rosrt.h"

#include <ros/ros.h>

#include <std_msgs/UInt32.h>

#include <boost/thread.hpp>

using namespace rosrt;

void publishThread(ros::Publisher& pub, bool& done)
{
  uint32_t i = 0;
  std_msgs::UInt32 msg;
  while (!done)
  {
    msg.data = i;
    pub.publish(msg);
    ros::WallDuration(0.0001).sleep();
    ++i;
  }
}

void run()
{
  const size_t sub_count = 1;

  ros::NodeHandle nh;
  bool done = false;
  boost::thread_group tg;

  ros::Publisher pubs[sub_count];
  Subscriber<std_msgs::UInt32> subs[sub_count];
  uint32_t counts[sub_count];
  int32_t lasts[sub_count];
  for (size_t i = 0; i < sub_count; ++i)
  {
    std::stringstream topic;
    topic << "test" << i;
    pubs[i] = nh.advertise<std_msgs::UInt32>(topic.str(), 1);
    tg.create_thread(boost::bind(publishThread, boost::ref(pubs[i]), boost::ref(done)));

    subs[i].initialize(2);
    subs[i].subscribe(nh, topic.str());
    counts[i] = 0;
    lasts[i] = -1;
  }


  bool all_done = false;
  while (!all_done)
  {
    all_done = true;

    for (size_t i = 0; i < sub_count; ++i)
    {
      std_msgs::UInt32ConstPtr msg = subs[i].poll();
      if (msg)
      {
        //assert((int32_t)msg->data == counts[i]);
        std::cout<<msg->data<<" "<<counts[i]<<std::endl;
        lasts[i] = msg->data;
        ++counts[i];
      }

      if (counts[i] < 10000)
      {
        all_done = false;
      }
    }
  }

  done = true;
  tg.join_all();
}

void subscribeThread(Subscriber<std_msgs::UInt32>& sub, bool& failed)
{
  resetThreadAllocInfo();

  const int count = 10000;
  int my_count = 0;
  int32_t last = -1;
  while (true)
  {
    std_msgs::UInt32ConstPtr msg = sub.poll();
    if (msg)
    {
      if (last >= (int32_t)msg->data)
      {
        ROS_ERROR_STREAM("Thread " << boost::this_thread::get_id() << " last is greater than msg " << last << " >= " << msg->data);
        failed = true;
        break;
      }

      last = msg->data;
      ++my_count;

      //ROS_INFO_STREAM("Thread " << boost::this_thread::get_id() << " " << my_count);
    }

    if (my_count >= count)
    {
      break;
    }

    ros::WallDuration(0.0001).sleep();
  }

  if (getThreadAllocInfo().total_ops > 0UL)
  {
    ROS_ERROR_STREAM("Thread " << boost::this_thread::get_id() << " performed " << getThreadAllocInfo().total_ops << " memory operations (malloc/free/etc.)");
    failed = true;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_rt_subscriber");
  
  ros::NodeHandle nh;
  rosrt::init();
  run();
  return 0;
}
