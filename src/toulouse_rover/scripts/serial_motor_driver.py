#!/usr/bin/env python3
import serial

import rospy
from toulouse_rover.msg import WheelPwmSpeeds

SERIAL_PORT = None
ENDIANNESS = "big"
START_BYTE = 0x3.to_bytes(1, ENDIANNESS)
END_BYTE = 0x4.to_bytes(1, ENDIANNESS)


def pwm_speed_callback(wheel_pwm_speeds_msg):
    global START_BYTE, END_BYTE, SERIAL_PORT, ENDIANNESS

    motor_enable_pwm = (wheel_pwm_speeds_msg.motor_enable).to_bytes(1, byteorder=ENDIANNESS)
    back_left_pwm_1 = (wheel_pwm_speeds_msg.back_left_pwm_1).to_bytes(1, byteorder=ENDIANNESS)
    back_left_pwm_2 = (wheel_pwm_speeds_msg.back_left_pwm_2).to_bytes(1, byteorder=ENDIANNESS)
    back_right_pwm_1 = (wheel_pwm_speeds_msg.back_right_pwm_1).to_bytes(1, byteorder=ENDIANNESS)
    back_right_pwm_2 = (wheel_pwm_speeds_msg.back_right_pwm_2).to_bytes(1, byteorder=ENDIANNESS)
    rospy.loginfo("sending {} for back_right_pwm_1".format(back_right_pwm_1))
    rospy.loginfo("sending {} for back_right_pwm_2".format(back_right_pwm_2))

    SERIAL_PORT.write(START_BYTE)
    SERIAL_PORT.write(motor_enable_pwm)
    SERIAL_PORT.write(back_left_pwm_1)
    SERIAL_PORT.write(back_left_pwm_2)
    SERIAL_PORT.write(back_right_pwm_1)
    SERIAL_PORT.write(back_right_pwm_2)
    SERIAL_PORT.write(END_BYTE)
    SERIAL_PORT.flush()


def parse_serial_msg():
    val = SERIAL_PORT.read()
    if val == START_BYTE:
        bytes_from_port = []
        while not rospy.is_shutdown():
            if SERIAL_PORT.in_waiting > 0:
                val = SERIAL_PORT.read()
            else:
                continue
            if val == END_BYTE:
                break
            val = int.from_bytes(val, byteorder=ENDIANNESS)
            bytes_from_port.append(val)
        log_str = "received: "
        for byte in bytes_from_port:
            log_str += "{},".format(byte)
        rospy.loginfo(log_str[:-1])


def main():
    global SERIAL_PORT, START_BYTE, END_BYTE, ENDIANNESS
    rospy.init_node('serial_motor_driver')

    SERIAL_PORT = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.050)
    rospy.Subscriber("/wheels/wheels_pwm_cmd", WheelPwmSpeeds, pwm_speed_callback)

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        if SERIAL_PORT.in_waiting > 0:
            parse_serial_msg()
        rate.sleep()


if __name__ == "__main__":
    main()
