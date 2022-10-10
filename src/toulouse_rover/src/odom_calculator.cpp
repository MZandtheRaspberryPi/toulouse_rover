#include "toulouse_rover/odom_calculator.h"

namespace odom_calculator
{
OdomCalculator::OdomCalculator(ros::NodeHandle& nh, util::WheelConfigurationType odom_type,
                               std::string wheel_namespace_str)
  : odom_type_(odom_type), wheel_namespace_(wheel_namespace_str)
{
  Position position_{};

  last_call_time_ = ros::Time::now();
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
  wheel_speed_subscriber_ =
      nh.subscribe("/" + wheel_namespace_ + "/wheel_speeds_actual", 1, &OdomCalculator::wheelSpeedCallback, this);
}

void OdomCalculator::wheelSpeedCallback(const toulouse_rover::WheelSpeeds::ConstPtr& speeds_msg)
{
  double speed_front_left = speeds_msg->front_left_radians_per_sec;
  double speed_front_right = speeds_msg->front_right_radians_per_sec;
  double speed_back_left = speeds_msg->back_left_radians_per_sec;
  double speed_back_right = speeds_msg->back_right_radians_per_sec;

  odom_calculator::Velocities velocities =
      calc_velocities(speed_front_left, speed_front_right, speed_back_left, speed_back_right);

  odom_calculator::PositionChange pos_change = calc_position_change(velocities);

  odom_calculator::Position position = calc_position(pos_change);

  odom_calculator::OdomRosMessages odom_messages = get_ros_odom_messages(position, velocities);

  // send the transform
  tf_br_.sendTransform(odom_messages.odom_transform);

  // publish the message
  odom_pub_.publish(odom_messages.odom);
}

Velocities OdomCalculator::calc_velocities(double speed_front_left, double speed_front_right, double speed_back_left,
                                           double speed_back_right)
{
  Velocities velocities{};
  switch (odom_type_)
  {
    case util::WheelConfigurationType::OMNI_WHEELS:
      velocities.vx_meters_per_sec =
          (speed_front_left + speed_front_right + speed_back_left + speed_back_right) * (util::WHEEL_RADIUS / 4);
      velocities.vy_meters_per_sec =
          (-speed_front_left + speed_front_right + speed_back_left - speed_back_right) * (util::WHEEL_RADIUS / 4);
      velocities.vth_radians_per_sec = (-speed_front_left + speed_front_right - speed_back_left + speed_back_right) *
                                       (util::WHEEL_RADIUS / (4 * (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH)));
      break;
    case util::WheelConfigurationType::SKID_STEERING: {
      float speed_radians_left = (speed_front_left + speed_back_left) / 2;
      float speed_radians_right = (speed_front_right + speed_back_right) / 2;
      float speed_meters_left = util::convert_radians_per_sec_to_meters_per_sec(speed_radians_left);
      float speed_meters_right = util::convert_radians_per_sec_to_meters_per_sec(speed_radians_right);
      velocities.vx_meters_per_sec = (speed_meters_left + speed_meters_right) / 2;
      velocities.vy_meters_per_sec = 0;
      velocities.vth_radians_per_sec = (speed_meters_right - speed_meters_left) / util::WHEEL_SEP_WIDTH;
      break;
    }
    case util::WheelConfigurationType::DIFFERENTIAL_DRIVE: {
      float speed_meters_left = util::convert_radians_per_sec_to_meters_per_sec(speed_back_left);
      float speed_meters_right = util::convert_radians_per_sec_to_meters_per_sec(speed_back_right);
      velocities.vx_meters_per_sec = (speed_meters_left + speed_meters_right) / 2;
      velocities.vy_meters_per_sec = 0;
      velocities.vth_radians_per_sec = (speed_meters_right - speed_meters_left) / util::WHEEL_SEP_WIDTH;
      break;
    }
    default:
      velocities.vx_meters_per_sec = 0;
      velocities.vy_meters_per_sec = 0;
      velocities.vth_radians_per_sec = 0;
  }

  return velocities;
}

// rotate to global frame with 2d rotation matrix
PositionChange OdomCalculator::calc_position_change(Velocities velocities)
{
  PositionChange pos_change{};
  ros::Time current_time = ros::Time::now();
  pos_change.dt = (current_time - last_call_time_).toSec();
  pos_change.delta_x_meters = (velocities.vx_meters_per_sec * cos(position_.th_radians) -
                               velocities.vy_meters_per_sec * sin(position_.th_radians)) *
                              pos_change.dt;
  pos_change.delta_y_meters = (velocities.vx_meters_per_sec * sin(position_.th_radians) +
                               velocities.vy_meters_per_sec * cos(position_.th_radians)) *
                              pos_change.dt;
  pos_change.delta_th_radians = velocities.vth_radians_per_sec * pos_change.dt;

  last_call_time_ = current_time;
  return pos_change;
}

Position OdomCalculator::calc_position(PositionChange position_change)
{
  position_.x_meters += position_change.delta_x_meters;
  position_.y_meters += position_change.delta_y_meters;
  position_.th_radians += position_change.delta_th_radians;
  return position_;
}

OdomRosMessages OdomCalculator::get_ros_odom_messages(Position position, Velocities velocities)
{
  OdomRosMessages odom_msgs{};
  // since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, position.th_radians);
  myQuaternion.normalize();
  // first, we'll create the transform over tf
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = last_call_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = position.x_meters;
  odom_trans.transform.translation.y = position.y_meters;
  odom_trans.transform.translation.z = 0.0;

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);
  odom_trans.transform.rotation = quat_msg;

  odom_msgs.odom_transform = odom_trans;

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = last_call_time_;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = position.x_meters;
  odom.pose.pose.position.y = position.y_meters;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quat_msg;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = velocities.vx_meters_per_sec;
  odom.twist.twist.linear.y = velocities.vy_meters_per_sec;
  odom.twist.twist.angular.z = velocities.vth_radians_per_sec;

  odom_msgs.odom = odom;
  return odom_msgs;
}

}  // namespace odom_calculator
