#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

class ThymioWanderer
{
  ros::Subscriber sub_proximity_center;
  ros::Subscriber sub_proximity_left;
  ros::Subscriber sub_proximity_right;
  ros::Publisher pub_cmd_vel;
  ros::NodeHandle node;

  geometry_msgs::Twist vel;

  enum
  {
    PROXIMITY_RIGHT,
    PROXIMITY_CENTER,
    PROXIMITY_LEFT,
  };

  typedef union
  {
    struct
    {
      uint8_t right: 1;
      uint8_t center: 1;
      uint8_t left: 1;
    };
    uint8_t all;
  } obstacle_map_t;

  obstacle_map_t obstacle_map;

  void callbackProximity(const sensor_msgs::RangeConstPtr& msg, uint8_t sensor_id)
  {
    // Fill in the obstacle binary 'map'
    if (msg->range < 0.10)
    {
      // set bit
      obstacle_map.all |= (1 << sensor_id);
    }
    else
    {
      // clear bit
      obstacle_map.all &= ~(1 << sensor_id);
    }

    ROS_INFO("Obstacle map: %d %d %d", obstacle_map.left, obstacle_map.center, obstacle_map.right);
  }

  void wander()
  {
    // We have 8 possible states in obstacle map
    // L  C  R
    // 0  0  0  = 0  -- keep going (go straight)
    // 0  0  1  = 1  -- turn a bit to the left
    // 0  1  0  = 2  -- turn either left or right
    // 0  1  1  = 3  -- turn more to the left
    // 1  0  0  = 4  -- turn a bit to the right
    // 1  0  1  = 5  -- ? probably can go forward, but have to consider robot dimensions
    // 1  1  0  = 6  -- turn more to the right
    // 1  1  1  = 7  -- ? turn till the view is clear

    float v = 0.2;
    float w = 0.3;

    switch(obstacle_map.all)
    {
      case 0:
        vel.linear.x = v;
        vel.angular.z = 0.0;
      break;
      case 1:
        // slow down and start turning left
        vel.linear.x = 0.0;
        vel.angular.z = w;
      break;
      case 2:
        // stop and turn till view is clear
        vel.linear.x = 0.0;
        vel.angular.z = w;
      break;
      case 3:
        // stop and turn left
        vel.linear.x = 0.0;
        vel.angular.z = w;
      break;
      case 4:
        // slow down and start turning right
        vel.linear.x = 0.0;
        vel.angular.z = -w;
      break;
      case 5:
        // probably can go
        vel.linear.x = v;
        vel.angular.z = 0.0;
      break;
      case 6:
        // stop and turn right
        vel.linear.x = 0.0;
        vel.angular.z = -w;
      break;
      case 7:
        vel.linear.x = 0.0;
        vel.angular.z = w;
      break;
    }
  }

  void init()
  {
    sub_proximity_center = node.subscribe<sensor_msgs::Range>("/proximity/center", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_CENTER));
    sub_proximity_left = node.subscribe<sensor_msgs::Range>("/proximity/left", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_LEFT));
    sub_proximity_right = node.subscribe<sensor_msgs::Range>("/proximity/right", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_RIGHT));

    pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

public:
  ThymioWanderer()
  {
    obstacle_map.all = 0;
    this->init();
  }

  void run()
  {
    ros::Rate loop_rate(100);

    vel.linear.x = 0.1;

    while(ros::ok())
    {
      wander();
      pub_cmd_vel.publish(vel);

      loop_rate.sleep();
      ros::spinOnce();
    }
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "wonderer");

  ThymioWanderer thymio;

  thymio.run();

  return 0;
}
