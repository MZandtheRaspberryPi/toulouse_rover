#!/usr/bin/env python3
import serial

import rospy
from toulouse_rover.msg import WheelPwmSpeeds

SERIAL_PORT = None
ENDIANNESS = "little"


END_BYTE = 0xC0.to_bytes(1, ENDIANNESS)
ESC_BYTE = 0xDB.to_bytes(1, ENDIANNESS)
ESC_END_BYTE = 0xDC.to_bytes(1, ENDIANNESS)
ESC_ESC_BYTE = 0xDD.to_bytes(1, ENDIANNESS)


def write_byte_and_escape_sequence(byte_to_send):
    if byte_to_send == END_BYTE:
        SERIAL_PORT.write(ESC_BYTE)
        SERIAL_PORT.write(ESC_END_BYTE)
    elif byte_to_send == ESC_BYTE:
        SERIAL_PORT.write(ESC_BYTE)
        SERIAL_PORT.write(ESC_ESC_BYTE)
    else:
        SERIAL_PORT.write(byte_to_send)

def adjust_received_bytes_for_esc_sequence(bytes_received: list) -> list:
    num_bytes_received = len(bytes_received)
    transformed_bytes_received = []
    ints_from_serial = []

    for i in range(num_bytes_received):
        if bytes_received[i] == ESC_END_BYTE and i > 0 and bytes_received[i - 1] == ESC_BYTE:
            transformed_bytes_received.pop()
            transformed_bytes_received.append(END_BYTE)
        elif bytes_received[i] == ESC_ESC_BYTE and i > 0 and bytes_received[i - 1] == ESC_BYTE:
            transformed_bytes_received.pop()
            transformed_bytes_received.append(ESC_BYTE)
        else:
            transformed_bytes_received.append(bytes_received[i])
    
    for byte_obj in transformed_bytes_received:
        ints_from_serial.append(int.from_bytes(byte_obj, ENDIANNESS))

    return ints_from_serial


def pwm_speed_callback(wheel_pwm_speeds_msg):

    motor_enable_pwm = (wheel_pwm_speeds_msg.motor_enable).to_bytes(1, byteorder=ENDIANNESS)
    back_left_pwm_1 = (wheel_pwm_speeds_msg.back_left_pwm_1).to_bytes(1, byteorder=ENDIANNESS)
    back_left_pwm_2 = (wheel_pwm_speeds_msg.back_left_pwm_2).to_bytes(1, byteorder=ENDIANNESS)
    back_right_pwm_1 = (wheel_pwm_speeds_msg.back_right_pwm_1).to_bytes(1, byteorder=ENDIANNESS)
    back_right_pwm_2 = (wheel_pwm_speeds_msg.back_right_pwm_2).to_bytes(1, byteorder=ENDIANNESS)
    rospy.loginfo("sending {} for back_right_pwm_1".format(back_right_pwm_1))
    rospy.loginfo("sending {} for back_right_pwm_2".format(back_right_pwm_2))

    write_byte_and_escape_sequence(motor_enable_pwm)
    write_byte_and_escape_sequence(back_left_pwm_1)
    write_byte_and_escape_sequence(back_left_pwm_2)
    write_byte_and_escape_sequence(back_right_pwm_1)
    write_byte_and_escape_sequence(back_right_pwm_2)
    SERIAL_PORT.write(END_BYTE)
    SERIAL_PORT.flush()


def parse_serial_msg():

    bytes_from_serial = []
    while not rospy.is_shutdown():
        if SERIAL_PORT.in_waiting > 0:
            val = SERIAL_PORT.read()
        else:
            continue
        if val == END_BYTE:
            break
        bytes_from_serial.append(val)
    
    ints_from_serial = adjust_received_bytes_for_esc_sequence(bytes_from_serial) 
    log_str = "received: "
    for int_data in ints_from_serial:
        log_str += "{},".format(int_data)
    rospy.logdebug(log_str[:-1])


def main():
    global SERIAL_PORT, START_BYTE, END_BYTE, ENDIANNESS
    rospy.init_node('serial_motor_driver')

    SERIAL_PORT = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.050)
    rospy.Subscriber("/wheels/wheels_pwm_cmd", WheelPwmSpeeds, pwm_speed_callback)

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        if SERIAL_PORT.in_waiting > 0:
            parse_serial_msg()
        rate.sleep()


if __name__ == "__main__":
    main()
