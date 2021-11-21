#include "wheels.h"


void genericInterupt(int index)
{
    if (globalSpeedCounter[index] > 0) {
        globalEncCounter[index]++;
    }
    else if (globalSpeedCounter[index] == 0.0) {
        return;
    }
    else {
        globalEncCounter[index]--;
    }
}

void frontLeftInterupt()
{
     genericInterupt(0);
}

void frontRightInterupt()
{
    genericInterupt(1);
}
void backRightInterupt()
{
    genericInterupt(2);
}

void backLeftInterupt()
{
    genericInterupt(3);
}


WheelController::WheelController(ros::NodeHandle& nh, std::string wheel_namespace, int interuptPin) : interuptPin_(interuptPin), wheel_namespace_(wheel_namespace)
{
    setupPubsSubs(nh, wheel_namespace);
    if (wheel_namespace == "front_left_wheel")
    {
         encoderIndex_ = 0;
    }
    else if (wheel_namespace == "front_right_wheel")
    {
        encoderIndex_ = 1;
    }
    else if (wheel_namespace == "back_right_wheel")
    {
        encoderIndex_ = 2;
    }
    else if (wheel_namespace == "back_left_wheel")
    {
        encoderIndex_ = 3;
    }

    priorEncoderCounts_ = 0;
    control_effort_ = 0.;
}

void WheelController::setupPubsSubs(ros::NodeHandle& nh, std::string wheel_namespace)
{
    state_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/state", 1);
    set_pub_ = nh.advertise<std_msgs::Float64>("/" + wheel_namespace + "/setpoint", 1);

    ctrl_sub_ = nh.subscribe("/" + wheel_namespace + "/control_effort", 1, &WheelController::controlEffortCallback, this);
}

void WheelController::controlEffortCallback(const std_msgs::Float64& control_effort_input) 
{
    control_effort_ = control_effort_input.data;
}



void WheelController::pubSpeedError() 
{
    std_msgs::Float64 state;
    double currentEncoderCounts =  globalEncCounter[encoderIndex_];
    ROS_INFO("%f", currentEncoderCounts);
    double changeEncoderCounts = currentEncoderCounts - priorEncoderCounts_;
    priorEncoderCounts_ = currentEncoderCounts;
    ros::Time now = ros::Time::now();
    ros::Duration elapsed_time_s = now - encoderStartTime_;
    encoderStartTime_ = now;
    ROS_INFO("%s: change in encoder count: %f", wheel_namespace_.c_str(), changeEncoderCounts);
    //state.data = changeEncoderCounts * 2 * M_PI / encTicksPerRotation_ / elapsed_time_s.toSec(); // 2 pi radians is 20 encoder counts, so this will give us radians per second
    state.data = changeEncoderCounts / elapsed_time_s.toSec();
    state_pub_.publish(state);
    ros::spinOnce();
}

int WheelController::ctrlWheel(float speed)
{
    if (speed != globalSpeedCounter[encoderIndex_])
    {
	globalSpeedCounter[encoderIndex_] = speed;
    }

    if (speed == 0.) 
    {
        return 0;
    }

    std_msgs::Float64 setpoint;
    setpoint.data = speed;
    set_pub_.publish(setpoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep(); // giving PID time to get this setpoint before publishing state
    pubSpeedError();

    int pwm = getPWM(control_effort_);
    return pwm;
}

int WheelController::getPWM(float control_effort)
{   
    ROS_INFO("ctrl: %f SLP: %f ", control_effort, SLOPE);
    float scaled_control_effort = MIN_PWM + SLOPE * (control_effort - MIN_PID_CONTROL);
    if (globalSpeedCounter[encoderIndex_] < 0)
    {
        scaled_control_effort *= -1;
    }
    int pwm = static_cast<int>(scaled_control_effort);
    return pwm;
}

