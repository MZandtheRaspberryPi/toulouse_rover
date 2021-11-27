#include <ros/ros.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include <tf2_ros/transform_broadcaster.h>
 #include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <cmath> 
#include <std_msgs/Float64.h>
#define SLP_PIN 22 // change pin number here

#include <ros/xmlrpc_manager.h>
// started at 900 can go to 4000
#define SPEED 900

#include <chrono>
#include <math.h>

#include "wheels.h"

const int LOOP_RATE = 10;

i2cpwm_board::ServoArray servo_array{};
geometry_msgs::Twist latest_vel_msg;
void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  latest_vel_msg = *vel;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

double x = 0.0;
double y = 0.0;
double th = 0.0;

int main (int argc, char **argv) {
#ifdef RPI
    wiringPiSetup();
    pinMode(SLP_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    digitalWrite(SLP_PIN, HIGH);
    ros::init(argc, argv, "robot_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    ros::Subscriber velocity_sub_ = nh.subscribe("cmd_vel", 1, velocityCallback);
    // servos_absolute publisher
    ros::Publisher servos_absolute_pub = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

    ros::Rate loop_rate(LOOP_RATE);  // Control rate in Hz 

    FrontLeftWheel front_left_wheel(nh, "front_left_wheel", false);
    wiringPiISR (0, INT_EDGE_FALLING, &frontLeftInterupt);
    FrontRightWheel front_right_wheel(nh, "front_right_wheel", false);
    wiringPiISR (25, INT_EDGE_FALLING, &frontRightInterupt);
    BackLeftWheel back_left_wheel(nh, "back_left_wheel", false);
    wiringPiISR (2, INT_EDGE_FALLING, &backLeftInterupt);
    BackRightWheel back_right_wheel(nh, "back_right_wheel", false);
    wiringPiISR (3, INT_EDGE_FALLING, &backRightInterupt);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster br;
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
    while (!g_request_shutdown) {

       // the work...
      ROS_INFO("x: %f, y: %f, angl_y: %f", latest_vel_msg.linear.x, latest_vel_msg.linear.y, latest_vel_msg.angular.z);
      int front_left_wheel_pwm = front_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int front_right_wheel_pwm = front_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int back_left_wheel_pwm = back_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int back_right_wheel_pwm = back_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
      ROS_INFO("FL: %d, FR: %d, BL: %d, BR: %d", front_left_wheel_pwm, front_right_wheel_pwm, back_left_wheel_pwm, back_right_wheel_pwm );

      if (front_left_wheel_pwm < 0) {
        servo_array.servos[14].value = 0;
        servo_array.servos[15].value = std::abs(front_left_wheel_pwm);
      }
      else {
        servo_array.servos[15].value = 0;
        servo_array.servos[14].value = std::abs(front_left_wheel_pwm);
      }

      if (front_right_wheel_pwm < 0) {
        servo_array.servos[13].value = 0;
        servo_array.servos[12].value = std::abs(front_right_wheel_pwm);
      }
      else {
        servo_array.servos[12].value = 0;
        servo_array.servos[13].value = std::abs(front_right_wheel_pwm);
      }

      if (back_left_wheel_pwm < 0) {
        servo_array.servos[9].value = 0;
        servo_array.servos[8].value = std::abs(back_left_wheel_pwm);
      }
      else {
        servo_array.servos[8].value = 0;
        servo_array.servos[9].value = std::abs(back_left_wheel_pwm);
      }

      if (back_right_wheel_pwm < 0) {
        servo_array.servos[10].value = 0;
        servo_array.servos[11].value = std::abs(back_right_wheel_pwm);
      }
      else {
        servo_array.servos[11].value = 0;
        servo_array.servos[10].value = std::abs(back_right_wheel_pwm);
      }
           
      servos_absolute_pub.publish(servo_array);
      ros::spinOnce();

      current_time = ros::Time::now();
      //compute odometry in a typical way given the velocities of the robot
      double speedFrontLeft = front_left_wheel.getWheelSpeed();
      double speedFrontRight = front_right_wheel.getWheelSpeed();
      double speedBackLeft = back_left_wheel.getWheelSpeed();
      double speedBackRight = back_right_wheel.getWheelSpeed();

      double vx = (speedFrontLeft + speedFrontRight + speedBackLeft + speedBackRight) * (WHEEL_RADIUS/4);
      double vy = ( -speedFrontLeft + speedFrontRight + speedBackLeft - speedBackRight) * (WHEEL_RADIUS/4);
      double vth = ( -speedFrontLeft + speedFrontRight - speedBackLeft + speedBackRight) * (WHEEL_RADIUS/(4 * (WHEEL_SEP_WIDTH + WHEEL_SEP_LENGTH)));

      double dt = (current_time - last_time).toSec();
      double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
      double delta_th = vth * dt;

      x += delta_x;
      y += delta_y;
      th += delta_th;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY( 0, 0, th );
      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      geometry_msgs::Quaternion quat_msg;

      tf2::convert(quat_msg , myQuaternion);
      odom_trans.transform.rotation = quat_msg;

      //send the transform
      br.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = quat_msg;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;

      loop_rate.sleep();
    }
    for (int i = 1; i <= 16; i++) {
      servo_array.servos[i].value = 0;
    }
    servos_absolute_pub.publish(servo_array);

    digitalWrite(SLP_PIN, LOW);
    ros::shutdown();
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
