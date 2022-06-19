#include <odom_calc.h>

namespace odom_calc {

OdomCalculator::OdomCalculator(OdomType odom_type) : odom_type_(odom_type) {
  Position position_{0.0,   // x
                     0.0,   // y
                     0.0};  // th

  last_call_time_ = ros::Time::now();
}

// todo, populate functions, split out fuctions into those for omni, and those
// for slip steering

Velocities OdomCalculator::calc_velocities(double speed_front_left,
                                           double speed_front_right,
                                           double speed_back_left,
                                           double speed_back_right) {
  Velocities velocities{};
  switch (odom_type_) {
    case OMNI_WHEELS:
      velocities.vx = (speed_front_left + speed_front_right + speed_back_left +
                       speed_back_right) *
                      (util::WHEEL_RADIUS / 4);
      velocities.vy = (-speed_front_left + speed_front_right + speed_back_left -
                       speed_back_right) *
                      (util::WHEEL_RADIUS / 4);
      velocities.vth = (-speed_front_left + speed_front_right -
                        speed_back_left + speed_back_right) *
                       (util::WHEEL_RADIUS /
                        (4 * (util::WHEEL_SEP_WIDTH + util::WHEEL_SEP_LENGTH)));
      break;
    case SKID_STEERING: {
      float speed_radians_left = (speed_front_left + speed_back_left) / 2;
      float speed_radians_right = (speed_front_right + speed_back_right) / 2;
      float speed_meters_left =
          util::convert_radians_per_sec_to_meters_per_sec(speed_radians_left);
      float speed_meters_right =
          util::convert_radians_per_sec_to_meters_per_sec(speed_radians_right);
      velocities.vx = (speed_meters_left + speed_meters_right) / 2;
      velocities.vy = 0;
      velocities.vth =
          (speed_meters_right - speed_meters_left) / util::WHEEL_SEP_WIDTH;
      break;
    }
    case DIFFERENTIAL_DRIVE: {
      float speed_meters_left =
          util::convert_radians_per_sec_to_meters_per_sec(speed_back_left);
      float speed_meters_right =
          util::convert_radians_per_sec_to_meters_per_sec(speed_back_right);
      velocities.vx = (speed_meters_left + speed_meters_right) / 2;
      velocities.vy = 0;
      velocities.vth =
          (speed_meters_right - speed_meters_left) / util::WHEEL_SEP_WIDTH;
      break;
    }
    default:
      velocities.vx = 0;
      velocities.vy = 0;
      velocities.vth = 0;
  }

  return velocities;
}

// rotate to global frame with 2d rotation matrix
PositionChange OdomCalculator::calc_position_change(Velocities velocities) {
  PositionChange pos_change{};
  ros::Time current_time = ros::Time::now();
  pos_change.dt = (current_time - last_call_time_).toSec();
  pos_change.delta_x =
      (velocities.vx * cos(position_.th) - velocities.vy * sin(position_.th)) *
      pos_change.dt;
  pos_change.delta_y =
      (velocities.vx * sin(position_.th) + velocities.vy * cos(position_.th)) *
      pos_change.dt;
  pos_change.delta_th = velocities.vth * pos_change.dt;

  last_call_time_ = current_time;
  return pos_change;
}

Position OdomCalculator::calc_position(PositionChange position_change) {
  position_.x += position_change.delta_x;
  position_.y += position_change.delta_y;
  position_.th += position_change.delta_th;
  return position_;
}

OdomRosMessages OdomCalculator::get_ros_odom_messages(Position position,
                                                      Velocities velocities) {
  OdomRosMessages odom_msgs{};
  // since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, position.th);
  myQuaternion.normalize();
  // first, we'll create the transform over tf
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = last_call_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = position.x;
  odom_trans.transform.translation.y = position.y;
  odom_trans.transform.translation.z = 0.0;

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);
  odom_trans.transform.rotation = quat_msg;

  odom_msgs.odom_transform = odom_trans;

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = last_call_time_;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = position.x;
  odom.pose.pose.position.y = position.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quat_msg;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = velocities.vx;
  odom.twist.twist.linear.y = velocities.vy;
  odom.twist.twist.angular.z = velocities.vth;

  odom_msgs.odom = odom;
  return odom_msgs;
}

}  // namespace odom_calc