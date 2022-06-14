#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <wheels.h>

namespace odom_calc {

enum OdomType {
  OMNI_WHEELS,
  SKID_STEERING
}

struct Velocities {
  double vx;
  double vy;
  double vth;
};

struct PositionChange {
  double dt;
  double delta_x;
  double delta_y;
  double delta_th;
};

struct Position {
  double x;
  double y;
  double th;
};

struct OdomRosMessages {
  geometry_msgs::TransformStamped odom_transform;
  nav_msgs::Odometry odom;
};

class OdomCalculator {
 private:
  OdomType odom_type_;
  Position position_;
  ros::Time last_call_time_;
};

OdomCalculator::OdomCalculator(OdomType odom_type);

Velocities OdomCalculator::calc_velocities(double speed_front_left,
                                           double speed_front_right,
                                           double speed_back_left,
                                           double speed_back_right);

PositionChange OdomCalculator::calc_position_change(Velocities velocities);

Position OdomCalculator::calc_position(PositionChange position_change);

OdomRosMessages OdomCalculator::get_ros_odom_messages(Position position, Velocities velocities);

}  // namespace odom_calc