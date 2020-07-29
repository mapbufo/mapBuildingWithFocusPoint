#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <boost/bind.hpp>

#include "common.h"
#include "communication_interface.h"
#include "map.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // main algorithm
  CommunicationInterface CIObject(nh);

  // global map
  Map test_map(nh);

  // loop for each 20 ms
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    CIObject.cycle(test_map);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
