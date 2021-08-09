#include <ros/ros.h>
#ifdef __arm__
#include <wiringPi.h>
#endif
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

#define SLP_PIN 22 // change pin number here


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_wiringpi_ros");
    ros::NodeHandle nh;
#ifdef __arm__
    wiringPiSetupGpio();
    pinMode(SLP_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");
    
    digitalWrite(SLP_PIN, HIGH);


    // servos_absolute publisher
    servos_absolute_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);
  
    // Initialize servo array message with 12 servo objects
    for (int i = 1; i <= smnc_.num_servos; i++) {
      i2cpwm_board::Servo temp_servo;
      temp_servo.servo = i;
      temp_servo.value = 0;
      servo_array_.servos.push_back(temp_servo);
    }


    while (ros::ok())
    {
        
        ROS_INFO("Motor On");
        ros::Duration(1.0).sleep();

        ROS_INFO("Motor Off");
        ros::Duration(1.0).sleep();
    }
    digitalWrite(SLP_PIN, LOW);
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
