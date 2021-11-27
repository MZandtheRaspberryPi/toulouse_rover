#include <ros/ros.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
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
      loop_rate.sleep();

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
