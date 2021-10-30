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

float lin_vel_x_;
float lin_vel_y_;
float ang_vel_;

ros::WallTime last_command_time_;

// speed in radians per second of wheels
float left_front_wheel = 1;
float left_back_wheel = 1;
float right_front_wheel = 1;
float right_back_wheel = 1;

static constexpr int const& WHEEL_SEP_LENGTH = 130; // how far wheels are apart length
static constexpr int const& WHEEL_SEP_WIDTH = 92; // how far wheels are apart width
static constexpr int const& WHEEL_RADIUS = 24; // radius of wheels

i2cpwm_board::ServoArray servo_array{};

void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  last_command_time_ = ros::WallTime::now();
  lin_vel_x_ = vel->linear.x;
  lin_vel_y_ = vel->linear.y;
  ang_vel_ = vel->angular.z;
}

int check_wheel_direction(float wheel_speed) {
  if (wheel_speed > 0) {
    return 1;
  }
  else if (wheel_speed < 0) {
    return -1;
  }
  else {
    return 0;
  }
}

// globalCounter:
// Global variable to count interrupts
// Should be declared volatile to make sure the compiler doesn't cache it.

static volatile double globalCounter [4];
static volatile double globalPrevCounter [4];

void myInterrupt0 (void) {
  globalCounter [0] += check_wheel_direction(left_front_wheel);
}
void myInterrupt1 (void) {
  globalCounter [1] += check_wheel_direction(left_back_wheel);
}
void myInterrupt2 (void) {
  globalCounter [2] += check_wheel_direction(right_front_wheel);
}
void myInterrupt3 (void) {
  globalCounter [3] += check_wheel_direction(right_back_wheel);
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

namespace left_front_wheel_control {
// Global so it can be passed from the callback fxn to main
static double control_effort = 0.0;
static bool reverse_acting = false;
}

// Callback when something is published on 'control_effort'
void controlEffortCallbackLFW(const std_msgs::Float64& control_effort_input)
{
  // the stabilizing control effort
  if (left_front_wheel_control::reverse_acting)
  {
    left_front_wheel_control::control_effort = -control_effort_input.data;
  }
  else
  {
    left_front_wheel_control::control_effort = control_effort_input.data;
  }
}

int main (int argc, char **argv) {
#ifdef RPI
    for (int pin = 0 ; pin < 4 ; ++pin) {
      globalCounter [pin] = 0.0 ;
      globalPrevCounter [pin] = 0.0 ;
    }
    wiringPiSetup();
    pinMode(SLP_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    digitalWrite(SLP_PIN, HIGH);

    wiringPiISR (0, INT_EDGE_FALLING, &myInterrupt0) ;
    wiringPiISR (25, INT_EDGE_FALLING, &myInterrupt1) ;
    wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ;
    wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ;

    ros::init(argc, argv, "test_motors_ros", ros::init_options::NoSigintHandler);
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

    // Advertise a plant state msg
    std_msgs::Float64 lfw_state;
    std_msgs::Float64 lfw_set;
    ros::Publisher lfw_state_pub = nh.advertise<std_msgs::Float64>("/left_front_wheel/state", 1);
    ros::Publisher lfw_set_pub = nh.advertise<std_msgs::Float64>("/left_front_wheel/setpoint", 1);
    ros::Subscriber lfw_sub_ = nh.subscribe("/left_front_wheel/control_effort", 1, controlEffortCallbackLFW);
    ros::Rate loop_rate(8);  // Control rate in Hz 

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
    auto t_start = std::chrono::high_resolution_clock::now();
    double lfw_change = 0.0;
    while (!g_request_shutdown) {


      ros::spinOnce();
       // the work...
      auto t_end = std::chrono::high_resolution_clock::now();

      double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
      lfw_change = globalCounter[0] - globalPrevCounter[0];
      ROS_INFO("change in encoder count: %f", lfw_change);
      lfw_state.data = lfw_change * 2 * M_PI / 20 / elapsed_time_ms * 1000; // 2 pi radians is 20 encoder counts, so this will give us radians per second
      lfw_state_pub.publish(lfw_state);
      globalPrevCounter[0] = globalCounter[0];
      
      if (left_front_wheel_control::control_effort < 0) {
        servo_array.servos[14].value = 0;
        servo_array.servos[15].value = left_front_wheel_control::control_effort * 4000 / 100;
      }
      else {
        servo_array.servos[15].value = 0;
        servo_array.servos[14].value = left_front_wheel_control::control_effort * 4000 / 100;

      }
      
      // 2000 pwm yields about 15 radians per second. this is like 5 pi, so 2.5 rotations of wheel per second
      // 900 pwm yields about 6.2 radians per second. this is like 2 pi, so 1 rotation of wheel per second
      // with a loop rate of 10hz, i see about 2 (somteimes 3) rotations per loop. so 20 encoder per sec, so ties out.
      //servo_array.servos[14].value = 900;
      servos_absolute_pub.publish(servo_array);
      t_start = std::chrono::high_resolution_clock::now();

      if (lfw_set.data != lin_vel_x_) {
        lfw_set.data = lin_vel_x_;
        lfw_set_pub.publish(lfw_set);
      }


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
