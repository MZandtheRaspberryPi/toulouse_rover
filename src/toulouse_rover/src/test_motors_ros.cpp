#include <ros/ros.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

#define SLP_PIN 22 // change pin number here


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_wiringpi_ros");
    ros::NodeHandle nh;
#ifdef RPI
    wiringPiSetupGpio();
    pinMode(SLP_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    digitalWrite(SLP_PIN, HIGH);


    // servos_absolute publisher
    ros::Publisher servos_absolute_pub = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

    i2cpwm_board::ServoArray servo_array{};
    // Initialize servo array message with 12 servo objects
    for (int i = 0; i <= 15; i++) {
      i2cpwm_board::Servo temp_servo;
      temp_servo.servo = i;
      temp_servo.value = 0;
      servo_array.servos.push_back(temp_servo);
    }

    servos_absolute_pub.publish(servo_array);
    while (ros::ok())
    {
        servo_array.servos[15].value = 1200;
        servo_array.servos[9].value = 1200;
	servo_array.servos[11].value = 1200;
	servo_array.servos[13].value = 1200;
        servos_absolute_pub.publish(servo_array);
        ROS_INFO("Motor On");
        ros::Duration(1.0).sleep();

        servo_array.servos[15].value = 0;
        servo_array.servos[9].value = 0;
        servo_array.servos[11].value = 0;
        servo_array.servos[13].value = 0;
        servos_absolute_pub.publish(servo_array);
        ROS_INFO("Motor Off");
        ros::Duration(1.0).sleep();
    }
    servo_array.servos[15].value = 0;
    servo_array.servos[9].value = 0;
    servo_array.servos[11].value = 0;
    servo_array.servos[13].value = 0;
    servos_absolute_pub.publish(servo_array);

    digitalWrite(SLP_PIN, LOW);
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
