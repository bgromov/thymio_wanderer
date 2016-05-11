#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

using Eigen::MatrixXd;

class ThymioWanderer
{
  ros::Subscriber sub_proximity_center;
  ros::Subscriber sub_proximity_left;
  ros::Subscriber sub_proximity_right;

  ros::Subscriber sub_ar_markers_;

  ros::Publisher pub_cmd_vel;

  ros::NodeHandle node;

  tf::TransformListener tf_ls_;

  geometry_msgs::Twist vel;

  struct Landmark
  {
    uint32_t id;
    double range;
    double bearing;

    Landmark(): id(-1), range(0.0), bearing(0.0) {}
  };

  std::vector<Landmark> landmarks_;

  enum
  {
    PROXIMITY_RIGHT,
    PROXIMITY_CENTER,
    PROXIMITY_LEFT,
    PROXIMITY_CENTER_RIGHT,
    PROXIMITY_CENTER_LEFT,
    PROXIMITY_REAR_RIGHT,
    PROXIMITY_REAR_LEFT,
    PROXIMITY_GROUND_RIGHT,
    PROXIMITY_GROUND_LEFT,
  };

  typedef union
  {
    struct
    {
      uint16_t right: 1;
      uint16_t center: 1;
      uint16_t left: 1;
      uint16_t center_right: 1;
      uint16_t center_left: 1;
      uint16_t rear_right: 1;
      uint16_t rear_left: 1;
      uint16_t ground_right: 1;
      uint16_t ground_left: 1;
    };
    uint16_t all;
  } obstacle_map_t;

  obstacle_map_t obstacle_map;

  tf::StampedTransform prox_left_;
  tf::StampedTransform prox_right_;

  tf::Pose current_pose_;
  tf::Pose target_pose_;

  double dist_left_;
  double dist_right_;

  void callbackProximity(const sensor_msgs::RangeConstPtr& msg, uint16_t sensor_id)
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

    if (msg->range < 0.5)
    {
      tf::Vector3 range(msg->range, 0.0, 0.0);

      switch(sensor_id)
      {
        case PROXIMITY_LEFT:
          dist_left_ = fabs((prox_left_ * range).getY());
          break;
        case PROXIMITY_RIGHT:
          dist_right_ = fabs((prox_right_ * range).getY());
          break;
      }

      ROS_INFO("Left: %3.4f, Right: %3.4f", dist_left_, dist_right_);
    }

    //ROS_INFO("Obstacle map: %d %d %d", obstacle_map.left, obstacle_map.center, obstacle_map.right);
  }

  void wander()
  {
    // We have 8 possible states (we use only 3 sensors out of 9) in obstacle map
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

  double sign(double num)
  {
    if (num > 0.0) return 1.0;
    else if (num < 0.0) return -1.0;
    else return 0.0;
  }

  double yawFromVector3(const tf::Vector3& v)
  {
    tf::Vector3 heading = v;
    tf::Vector3 x_unit = tf::Vector3(1.0, 0.0, 0.0);
    tf::Vector3 cross = heading.cross(x_unit) ;

    double yaw = -asin(cross.length() / sqrt(heading.length2() * x_unit.length2())) * sign(cross.getZ());

    return yaw;
  }

  void markersCb(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
  {
    std::vector<Landmark> lv;

    // Iterate over markers array
    for (size_t i = 0; i < msg->markers.size(); i++)
    {
      // Extract single marker
      ar_track_alvar_msgs::AlvarMarker m = msg->markers[i];
      tf::StampedTransform m_tf;
      Landmark l;

      try
      {
        // Find transformation from marker to robot's base link
        tf_ls_.lookupTransform("base_link", std::string("ar_marker_") + boost::lexical_cast<std::string>(m.id), ros::Time(0), m_tf);

        // Calculate the center of the cube
        tf::Vector3 cube_center = m_tf * tf::Vector3(0.0, 0.0, -0.025);

//        ROS_INFO("[%u] %3.3f, %3.3f, %3.3f : %3.3f, %3.3f, %3.3f", m.id,
//                 m_tf.getOrigin().getX(), m_tf.getOrigin().getY(), m_tf.getOrigin().getZ(),
//                 cube_center.getX(), cube_center.getY(), cube_center.getZ());

        // Calculate range and bearing and store them in local array
        l.id = m.id;
        l.range = m_tf.getOrigin().length();
        l.bearing = yawFromVector3(m_tf.getOrigin());
        lv.push_back(l);

        ROS_INFO("Landmark [%u] range: %2.3f, bearing: %3.2f", l.id, l.range, l.bearing * 180.0 / M_PI);
      }
      catch(tf::TransformException& e)
      {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }

    // Once everything is fine, copy (swap) local array to landmarks_
    landmarks_.swap(lv);
  }

  void init()
  {
    sub_proximity_center = node.subscribe<sensor_msgs::Range>("/proximity/center", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_CENTER));
    sub_proximity_left = node.subscribe<sensor_msgs::Range>("/proximity/left", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_LEFT));
    sub_proximity_right = node.subscribe<sensor_msgs::Range>("/proximity/right", 10, boost::bind(&ThymioWanderer::callbackProximity, this, _1, PROXIMITY_RIGHT));

    sub_ar_markers_ = node.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 100, boost::bind(&ThymioWanderer::markersCb, this, _1));

    pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

public:
  ThymioWanderer(): dist_left_(0.0), dist_right_(0.0)
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
      try
      {
        tf_ls_.lookupTransform("base_link", "proximity_left_link", ros::Time(0), prox_left_);
        tf_ls_.lookupTransform("base_link", "proximity_right_link", ros::Time(0), prox_right_);
      }
      catch(tf::TransformException& e)
      {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
      }
//      wander();
//      pub_cmd_vel.publish(vel);

//      ROS_INFO("Landmarks in the array: %lu", landmarks_.size());

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
