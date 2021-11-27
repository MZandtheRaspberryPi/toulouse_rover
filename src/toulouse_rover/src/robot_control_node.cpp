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
#include <std_msgs/Float64.h>
#define SLP_PIN 22 // change pin number here

#include <ros/xmlrpc_manager.h>
// started at 900 can go to 4000
#define SPEED 900

#include <chrono>
#include <math.h>

#include "wheels.h"

float lin_vel_x_;
float lin_vel_y_;
float ang_vel_;

ros::WallTime last_command_time_;

// speed in radians per second of wheels
float left_front_wheel_speed = 1;
float left_back_wheel_speed = 1;
float right_front_wheel_speed = 1;
float right_back_wheel_speed = 1;

int left_front_wheel_pwm = 0;



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

/*
/left_front_wheel/control_effort
/left_front_wheel/pid_debug
/left_front_wheel/pid_enable
/left_front_wheel/setpoint
/left_front_wheel/state
/left_front_wheel/wheel_pid/parameter_descriptions
/left_front_wheel/wheel_pid/parameter_updates
/right_front_wheel/control_effort
/right_front_wheel/pid_debug
/right_front_wheel/pid_enable
/right_front_wheel/setpoint
/right_front_wheel/state
/right_front_wheel/wheel_pid/parameter_description
/right_front_wheel/wheel_pid/parameter_updates
*/

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

    ros::Rate loop_rate(2);  // Control rate in Hz 

    FrontLeftWheel front_left_wheel(nh, "front_left_wheel", false);
    wiringPiISR (0, INT_EDGE_FALLING, &frontLeftInterupt);
    FrontRightWheel front_right_wheel(nh, "front_right_wheel", false);
    wiringPiISR (25, INT_EDGE_FALLING, &frontRightInterupt);
    BackLeftWheel back_left_wheel(nh, "back_left_wheel", false);
    wiringPiISR (2, INT_EDGE_FALLING, &backLeftInterupt);
    BackRightWheel back_right_wheel(nh, "back_right_wheel", false);
    wiringPiISR (3, INT_EDGE_FALLING, &backRightInterupt);
    ros::Duration(1.0).sleep();
/*
/left_front_wheel/control_effort
/left_front_wheel/pid_debug
/left_front_wheel/pid_enable
/left_front_wheel/setpoint
/left_front_wheel/state
/left_front_wheel/wheel_pid/parameter_descriptions
/left_front_wheel/wheel_pid/parameter_updates
*/
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
      int front_left_wheel_pwm = front_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int front_right_wheel_pwm = front_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int back_left_wheel_pwm = back_left_wheel.ctrlWheelCmdVel(latest_vel_msg);
      int back_right_wheel_pwm = back_right_wheel.ctrlWheelCmdVel(latest_vel_msg);
      ROS_INFO("FL: %d, FR: %d, BL: %d, BR: %d", front_left_wheel_pwm, front_right_wheel_pwm, back_left_wheel_pwm, back_right_wheel_pwm );
      
      /* 
      if (left_front_wheel_pwm < 0) {
        servo_array.servos[14].value = 0;
        servo_array.servos[15].value = left_front_wheel_pwm;
      }
      else {
        servo_array.servos[15].value = 0;
        servo_array.servos[14].value = left_front_wheel_pwm;
        //servo_array.servos[14].value = 3000;
      }
      */
      
      // 2000 pwm yields about 15 radians per second. this is like 5 pi, so 2.5 rotations of wheel per second
      // 900 pwm yields about 6.2 radians per second. this is like 2 pi, so 1 rotation of wheel per second
      // with a loop rate of 10hz, i see about 2 (somteimes 3) rotations per loop. so 20 encoder per sec, so ties out.
      //servo_array.servos[14].value = 900;
      servos_absolute_pub.publish(servo_array);
      ros::spinOnce();
      loop_rate.sleep();
      /*
      servo_array.servos[14].value = SPEED;
      servo_array.servos[9].value = SPEED;
      servo_array.servos[10].value = SPEED;
      servo_array.servos[13].value = SPEED;
      servos_absolute_pub.publish(servo_array);
      ROS_INFO("Motor On");
      ros::Duration(1.0).sleep();
      
      servo_array.servos[14].value = 0;
      servo_array.servos[9].value = 0;
      servo_array.servos[10].value = 0;
      servo_array.servos[13].value = 0;
      servos_absolute_pub.publish(servo_array);
      ROS_INFO("Motor Off");
      ros::Duration(1.0).sleep();
      ROS_INFO("Left front wheel is at %d\n", globalCounter[0]);
      */
    }
    // these are all by array index, not by servo number
    // servo number starts at 1, but array index starts at 0
    // 14 is front left, front as in by battery
    // if 14 is high pwm like 1200, and 15 is low, then front left wheel forward
    // if 15 is high pwm like 1200, and 14 is low, then front left wheel backward

    // if 13 is pwm high like 1200, and 12 is low then front right wheel forward
    // if 13 is low, and 12 is high pwm, then front right wheel backwards

    // if 9 is pwm high and 8 is low, then back left wheel turns forward
    // if 8 is pwm high, and 9 is pwm low, then back left wheel turns backward

    // if 10 is pwm high and 11 is pwm low, then back right wheel turns forard
    // if 11 is pwm high and 10 is pwm low, then back right wheel turns backward
    servo_array.servos[14].value = 0;
    servo_array.servos[9].value = 0;
    servo_array.servos[10].value = 0;
    servo_array.servos[13].value = 0;
    servos_absolute_pub.publish(servo_array);

    digitalWrite(SLP_PIN, LOW);
    ros::shutdown();
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
