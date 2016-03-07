#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

geometry_msgs::Twist vel;

void callbackProximity(const sensor_msgs::RangeConstPtr& msg)
{
  if (msg->range < 0.15)
  {
    vel.linear.x = 0.0;
    ROS_INFO("Alarm! Obstacle!");
  }
  else
  {
    vel.linear.x = 0.05;
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "wonderer");

  ros::Subscriber sub_proximity_center;
  ros::Publisher pub_cmd_vel;
  ros::NodeHandle node;

  sub_proximity_center = node.subscribe("/proximity/center", 10, callbackProximity);
  pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  vel.linear.x = 0.1;

  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    pub_cmd_vel.publish(vel);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
