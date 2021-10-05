#include <ros/ros.h>
#ifdef RPI
#include <wiringPi.h>
#endif
#define LED_PIN 0 // change pin number here
int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_wiringpi_ros");
    ros::NodeHandle nh;
    ROS_INFO("set up node");

#ifdef RPI
    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");
    while (ros::ok())
    {
        digitalWrite(LED_PIN, HIGH);
        ROS_INFO("Set GPIO HIGH");
        ros::Duration(1.0).sleep();
        digitalWrite(LED_PIN, LOW);
        ROS_INFO("Set GPIO LOW");
        ros::Duration(1.0).sleep();
    }
#else
    // here you define the behavior of your node on non ARM systems
#endif
}
