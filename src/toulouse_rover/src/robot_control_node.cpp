#include <ros/ros.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#include <errno.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

#define SLP_PIN 22  // change pin number here

#include <ros/xmlrpc_manager.h>
// started at 900 can go to 4000
#define SPEED 900

#include <math.h>

#include <chrono>

#include "odom_calc.h"
#include "wheels.h"

const int LOOP_RATE = 20;

i2cpwm_board::ServoArray servo_array{};
geometry_msgs::Twist latest_vel_msg;
void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel) {
  latest_vel_msg = *vel;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig) { g_request_shutdown = 1; }

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params,
                      XmlRpc::XmlRpcValue& result) {
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1) {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1;  // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

double x = 0.0;
double y = 0.0;
double th = 0.0;


int main(int argc, char** argv) {
#ifdef RPI
  wiringPiSetup();
  pinMode(SLP_PIN, OUTPUT);
  ROS_INFO("GPIO has been set as OUTPUT.");

  digitalWrite(SLP_PIN, HIGH);
#endif
  ros::init(argc, argv, "robot_control_node",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  odom_calc::OdomCalculator odom_calculator{odom_calc::DIFFERENTIAL_DRIVE};

  ros::Subscriber velocity_sub_ = nh.subscribe("cmd_vel", 1, velocityCallback);
  // servos_absolute publisher
  ros::Publisher servos_absolute_pub =
      nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

  ros::Rate loop_rate(LOOP_RATE);  // Control rate in Hz

  wheels::FrontLeftWheel front_left_wheel(nh, "front_left_wheel", false,
                                          odom_calc::DIFFERENTIAL_DRIVE);
  wheels::FrontRightWheel front_right_wheel(nh, "front_right_wheel", false,
                                            odom_calc::DIFFERENTIAL_DRIVE);
  wheels::BackLeftWheel back_left_wheel(nh, "back_left_wheel", false,
                                        odom_calc::DIFFERENTIAL_DRIVE);
  wheels::BackRightWheel back_right_wheel(nh, "back_right_wheel", false,
                                          odom_calc::DIFFERENTIAL_DRIVE);

#ifdef RPI
  wiringPiISR(0, INT_EDGE_FALLING, &wheels::frontLeftInterupt);
  wiringPiISR(2, INT_EDGE_FALLING, &wheels::frontRightInterupt);
  wiringPiISR(25, INT_EDGE_FALLING, &wheels::backLeftInterupt);
  wiringPiISR(3, INT_EDGE_FALLING, &wheels::backRightInterupt);
#endif

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Duration(1.0).sleep();

  // Initialize servo array message with 12 servo objects
  for (int i = 1; i <= 16; i++) {
    i2cpwm_board::Servo temp_servo;
    temp_servo.servo = i;
    temp_servo.value = 0;
    servo_array.servos.push_back(temp_servo);
  }
  servos_absolute_pub.publish(servo_array);
  int front_left_wheel_pwm;
  int front_right_wheel_pwm;
  int back_left_wheel_pwm;
  int back_right_wheel_pwm;
  while (!g_request_shutdown) {
    // the work...
    ROS_INFO("x: %f, y: %f, angl_y: %f", latest_vel_msg.linear.x,
           latest_vel_msg.linear.y, latest_vel_msg.angular.z);
    front_left_wheel_pwm = front_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
    front_right_wheel_pwm =
      front_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
    back_left_wheel_pwm = back_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
    back_right_wheel_pwm = back_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
    ROS_INFO("FL: %d, FR: %d, BL: %d, BR: %d", front_left_wheel_pwm,
             front_right_wheel_pwm, back_left_wheel_pwm, back_right_wheel_pwm);
    if (front_left_wheel_pwm < 0) {
      servo_array.servos[14].value = 0;
      servo_array.servos[15].value = std::abs(front_left_wheel_pwm);
    } else {
      servo_array.servos[15].value = 0;
      servo_array.servos[14].value = std::abs(front_left_wheel_pwm);
    }

    if (front_right_wheel_pwm < 0) {
      servo_array.servos[13].value = 0;
      servo_array.servos[12].value = std::abs(front_right_wheel_pwm);
    } else {
      servo_array.servos[12].value = 0;
      servo_array.servos[13].value = std::abs(front_right_wheel_pwm);
    }

    if (back_left_wheel_pwm < 0) {
      servo_array.servos[9].value = 0;
      servo_array.servos[8].value = std::abs(back_left_wheel_pwm);
    } else {
      servo_array.servos[8].value = 0;
      servo_array.servos[9].value = std::abs(back_left_wheel_pwm);
    }

    if (back_right_wheel_pwm < 0) {
      servo_array.servos[10].value = 0;
      servo_array.servos[11].value = std::abs(back_right_wheel_pwm);
    } else {
      servo_array.servos[11].value = 0;
      servo_array.servos[10].value = std::abs(back_right_wheel_pwm);
    }

    servos_absolute_pub.publish(servo_array);
    ros::spinOnce();

    // compute odometry in a typical way given the velocities of the robot
    double speed_front_left = front_left_wheel.getWheelSpeed();
    double speed_front_right = front_right_wheel.getWheelSpeed();
    double speed_back_left = back_left_wheel.getWheelSpeed();
    double speed_back_right = back_right_wheel.getWheelSpeed();

    odom_calc::Velocities velocities = odom_calculator.calc_velocities(
        speed_front_left, speed_front_right, speed_back_left, speed_back_right);

    odom_calc::PositionChange pos_change =
        odom_calculator.calc_position_change(velocities);

    odom_calc::Position position = odom_calculator.calc_position(pos_change);

    odom_calc::OdomRosMessages odom_messages =
        odom_calculator.get_ros_odom_messages(position, velocities);

    // send the transform
    br.sendTransform(odom_messages.odom_transform);

    // publish the message
    odom_pub.publish(odom_messages.odom);
    loop_rate.sleep();
  }
  for (int i = 1; i <= 16; i++) {
    servo_array.servos[i].value = 0;
  }
  servos_absolute_pub.publish(servo_array);
#ifdef RPI
  digitalWrite(SLP_PIN, LOW);
#endif
  ros::shutdown();
}
