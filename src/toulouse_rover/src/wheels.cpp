#include "wheels.h"

WheelController::WheelController(ros::NodeHandle& nh, std::string wheel_namespace, int interuptPin) : interuptPin_(interuptPin), wheel_namespace_(wheel_namespace)
{
    commandRadPerSec_ = 0;
    #ifdef RPI
        // setup wiring pi interupt for this wheel
        // assume wiring pi setup has been called
        wiringPiISR (0, INT_EDGE_FALLING, &wheelInterupt);
    #endif
    loop_rate_(CHECK_RATE_CTRL);
    setupPubsSubs(wheel_namespace);

}

void WheelController::setupPubsSubs(std::string wheel_namespace)
{

    ros::Publisher state_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/state", 1);
    ros::Publisher set_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/setpoint", 1);
    ros::Subscriber ctrl_sub_ = nh.subscribe("/" + wheel_namespace + "/control_effort", 1, &WheelController::controlEffortCallback, this);

}

void WheelController::controlEffortCallback(const std_msgs::Float64& control_effort_input) 
{
    control_effort_.push(control_effort_input.data);
}



void WheelController::pubSpeedError() 
{
    std_msgs::Float64 state;
    int changeEncoderCounts = encoderCounts_ - priorEncoderCounts_;
    priorEncoderCounts_ = encoderCounts_;
    ros::Time now = ros::Time::now();
    ros::Duration elapsed_time_s = now - encoderStartTime_;
    encoderStartTime_ = now;
    ROS_INFO("%s: change in encoder count: %f", wheel_namespace_, changeEncoderCounts);
    state.data = changeEncoderCounts * 2 * M_PI / encTicksPerRotation_ / elapsed_time_s; // 2 pi radians is 20 encoder counts, so this will give us radians per second
    state_pub_.publish(lfw_state);
    ros::spinOnce();
}

int WheelController::ctrlWheel(float speed)
{
    if (speed != commandRadPerSec_)
    {
        commandRadPerSec_ = speed;
    }

    if (speed == 0.) 
    {
        return 0;
    }

    pubSpeedError();
    
    // waiting for update from PID on how to control system
    while (ros::ok() && control_effort_.empty())
    {
        loop_rate_.sleep();
    }

    if (!ros::ok())
    {
        return 0;
    }

    float control_effort = control_effort_.front();
    control_effort_.pop();
    int pwm = getPWM(control_effort);
    return pwm;
}

int WheelController::getPWM(float control_effort)
{
    float scaled_control_effort = MIN_PWM + SLOPE * (control_effort - MIN_PID_CONTROL)
    if (commandRadPerSec_ < 0)
    {
        scaled_control_effort *= -1;
    }
    return std::static_cast<int>(scaled_control_effort);
}

void WheelController::wheelInterupt() 
{
    if (commandRadPerSec_ > 0) {
        encoderCounts_++;
    }
    else {
        encoderCounts_--;
    }
}

 
 
