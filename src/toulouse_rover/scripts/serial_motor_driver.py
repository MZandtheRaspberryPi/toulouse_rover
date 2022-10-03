#!/usr/bin/env python3
import serial

import rospy
from toulouse_rover.msg import WheelPwmSpeeds

SERIAL_PORT = None


def pwm_speed_callback(wheel_pwm_speeds_msg):

    motor_enable_pwm = (wheel_pwm_speeds_msg.motor_enable).to_bytes(1, byteorder='big')
    back_left_pwm_1 = (wheel_pwm_speeds_msg.back_left_pwm_1).to_bytes(1, byteorder='big')
    back_left_pwm_2 = (wheel_pwm_speeds_msg.back_left_pwm_2).to_bytes(1, byteorder='big')
    back_right_left_pwm_1 = (wheel_pwm_speeds_msg.back_right_left_pwm_1).to_bytes(1, byteorder='big')
    back_right_left_pwm_2 = (wheel_pwm_speeds_msg.back_right_left_pwm_2).to_bytes(1, byteorder='big')

    rospy.loginfo("sending {} for back_left_1".format(back_left_pwm_1))
    rospy.loginfo("sending {} for back_left_2".format(back_left_pwm_2))

    SERIAL_PORT.write("<".encode("ascii"))
    SERIAL_PORT.write(motor_enable_pwm)
    SERIAL_PORT.write(back_left_pwm_1)
    SERIAL_PORT.write(back_left_pwm_2)
    SERIAL_PORT.write(back_right_left_pwm_1)
    SERIAL_PORT.write(back_right_left_pwm_2)
    SERIAL_PORT.write(">".encode("ascii"))

def main():
    global SERIAL_PORT
    rospy.init_node('serial_motor_driver')

    rospy.Subscriber("/wheels/wheels_pwm_cmd", WheelPwmSpeeds, pwm_speed_callback)

    SERIAL_PORT = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.050)
    rospy.spin()


if __name__ == "__main__":
    main()
