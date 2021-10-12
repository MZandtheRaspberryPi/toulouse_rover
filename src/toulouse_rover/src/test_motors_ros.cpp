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
#define SLP_PIN 22 // change pin number here

float lin_vel_x_;
float lin_vel_y_;
float ang_vel_;
ros::WallTime last_command_time_;

// speed in radians per second of wheels
float left_front_wheel;
float left_back_wheel;
float right_front_wheel;
float right_back_wheel;

static constexpr int const& WHEEL_SEP_LENGTH = 130; // how far wheels are apart length
static constexpr int const& WHEEL_SEP_WIDTH = 92; // how far wheels are apart width
static constexpr int const& WHEEL_RADIUS = 24; // radius of wheels

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
//      Global variable to count interrupts
//      Should be declared volatile to make sure the compiler doesn't cache it.

static volatile int globalCounter [4];

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

void mySigintHandler(int sig)
{
  // Do some custom action.
  servo_array.servos[14].value = 0;
  servo_array.servos[9].value = 0;
  servo_array.servos[10].value = 0;
  servo_array.servos[13].value = 0;
  servos_absolute_pub.publish(servo_array);

  digitalWrite(SLP_PIN, LOW);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


int main (int argc, char **argv) {
    ros::init(argc, argv, "test_motors_ros", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
    ros::Subscriber velocity_sub_ = nh.subscribe("cmd_vel", 1, velocityCallback);
#ifdef RPI
    wiringPiSetupGpio();
    pinMode(SLP_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    digitalWrite(SLP_PIN, HIGH);

    for (int pin = 0 ; pin < 4 ; ++pin)
      globalCounter [pin] = 0 ;

    wiringPiISR (0, INT_EDGE_FALLING, &myInterrupt0) ;
    wiringPiISR (25, INT_EDGE_FALLING, &myInterrupt1) ;
    wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ;
    wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ;
    // servos_absolute publisher
    ros::Publisher servos_absolute_pub = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

    i2cpwm_board::ServoArray servo_array{};
    // Initialize servo array message with 12 servo objects
    for (int i = 1; i <= 16; i++) {
      i2cpwm_board::Servo temp_servo;
      temp_servo.servo = i;
      temp_servo.value = 0;
      servo_array.servos.push_back(temp_servo);
    }

    servos_absolute_pub.publish(servo_array);
    while (ros::ok())
    {
        servo_array.servos[14].value = 1200;
        servo_array.servos[9].value = 1200;
	servo_array.servos[10].value = 1200;
	servo_array.servos[13].value = 1200;
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
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
