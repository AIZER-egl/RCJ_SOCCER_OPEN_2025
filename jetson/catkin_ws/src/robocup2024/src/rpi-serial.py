#!/usr/bin/env python3

import rospy
import serial
import math
import threading
from std_msgs.msg import UInt64

ser = serial.Serial('/dev/ttyACM0', 115200)
rospy.init_node("rpiSerial")
rospy.loginfo("RPI SERIAL set up")
pub = rospy.Publisher('rpi2jetson', UInt64, queue_size=10)


def encode_hex(value):
    hex_num = hex(value)
    hex_string = hex_num[2:]
    return hex_string.upper()


def decode_hex(hex_string):
    value = 0
    for i in range(len(hex_string)):
        char = hex_string[i]
        if char.isdigit():
            value += int(char) * pow(16, len(hex_string) - i - 1)
        else:
            value += (ord(char) - 55) * pow(16, len(hex_string) - i - 1)
    return value


def callback(data):
    rospy.loginfo("Received data %lu", data.data)
    encoded_hex = encode_hex(data.data)
    ser.write(bytes(encoded_hex, 'utf-8'))
    ser.write(b'\n')


def serial_reader():
    while not rospy.is_shutdown():
        data = ser.readline()

        if data:
            data = data.decode('utf-8').strip()
            decoded_number = decode_hex(data)
            # rospy.loginfo("Decoded values from rpi pico %lu", decoded_number)

            if isinstance(decoded_number, int) and (math.pow(2, 64) - 1) > decoded_number > 0:
                pub.publish(decoded_number)


serial_thread = threading.Thread(target=serial_reader)
serial_thread.start()

rospy.Subscriber('jetson2rpi', UInt64, callback)

rospy.spin()
