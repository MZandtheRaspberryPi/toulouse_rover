#ifndef odom_calc_h
#define odom_calc_h

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "toulouse_rover/WheelSpeeds.h"
#include "toulouse_rover/util.h"

namespace odom_calculator
{
struct Velocities
{
  double vx;
  double vy;
  double vth;
};

struct PositionChange
{
  double dt;
  double delta_x;
  double delta_y;
  double delta_th;
};

struct Position
{
  double x;
  double y;
  double th;
};

struct OdomRosMessages
{
  geometry_msgs::TransformStamped odom_transform;
  nav_msgs::Odometry odom;
};

class OdomCalculator
{
public:
  OdomCalculator(ros::NodeHandle& nh, util::WheelConfigurationType odom_type);
  Velocities calc_velocities(double speed_front_left, double speed_front_right, double speed_back_left,
                             double speed_back_right);
  PositionChange calc_position_change(Velocities velocities);

  Position calc_position(PositionChange position_change);

  OdomRosMessages get_ros_odom_messages(Position position, Velocities velocities);

private:
  void wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& speeds_msg);
  tf2_ros::TransformBroadcaster tf_br_;
  util::WheelConfigurationType odom_type_;
  Position position_;
  ros::Time last_call_time_;
  ros::Publisher odom_pub_;
  ros::Subscriber wheel_speed_subscriber_;
};

}  // namespace odom_calculator

#endif