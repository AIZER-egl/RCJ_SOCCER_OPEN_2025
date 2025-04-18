#!/usr/bin/env python3

import rospy
import serial
import struct
import sys
import threading
from std_msgs.msg import ByteMultiArray

SERIAL_PACKET_SIZE = 18
SERIAL_TIMEOUT = 0.1

serial_port = None
serial_lock = threading.Lock()

def serial_read_thread(pub_serial_rx):
    global serial_port
    rospy.loginfo("Serial read thread started.")
    read_buffer = bytearray()

    while not rospy.is_shutdown() and serial_port and serial_port.is_open:
        try:
            with serial_lock:
                bytes_waiting = serial_port.in_waiting
                if bytes_waiting > 0:
                    data = serial_port.read(bytes_waiting)
                    if data:
                        read_buffer.extend(data)
                        # rospy.logdebug(f"Read {len(data)} bytes, buffer size: {len(read_buffer)}")

            while len(read_buffer) >= SERIAL_PACKET_SIZE:
                packet = read_buffer[:SERIAL_PACKET_SIZE]
                del read_buffer[:SERIAL_PACKET_SIZE]

                rospy.logdebug(f"Received packet ({len(packet)} bytes): {packet.hex()}")

                msg = ByteMultiArray()
                try:
                    format_string = f'{SERIAL_PACKET_SIZE}b'
                    signed_values_tuple = struct.unpack(format_string, packet)                   
                    msg.data = signed_values_tuple
                except struct.error as e:
                    rospy.logerr(f"Failed to unpack serial data for ROS msg: {e}. Packet size: {len(packet)}")

                pub_serial_rx.publish(msg)
                rospy.loginfo(f"Published {len(packet)} bytes to /pico/serial_rx")

        except serial.SerialException as e:
            rospy.logerr(f"Serial read error: {e}")
            break
        except Exception as e:
            rospy.logerr(f"Error in serial read thread: {e}")
            break

        rospy.sleep(0.01)

    rospy.loginfo("Serial read thread finished.")

def serial_tx_callback(msg):
    global serial_port, serial_lock, SERIAL_PACKET_SIZE

    try:
        rospy.loginfo(f"DEBUG: Callback entered. msg.data type: {type(msg.data)}, value: {msg.data}")
    except Exception as e:
        rospy.logerr(f"DEBUG: Error accessing msg.data at start of callback: {e}")
        return

    rospy.loginfo(f"Received {len(msg.data)} bytes on /pico/serial_tx")

    if serial_port and serial_port.is_open:
        try:

            unsigned_values = [(val + 256) % 256 for val in msg.data]
            data_to_send = bytearray(unsigned_values)

            if len(data_to_send) != SERIAL_PACKET_SIZE and len(data_to_send) != 1:
                 rospy.logwarn(f"Received data packet of unexpected size ({len(data_to_send)} bytes), expected {SERIAL_PACKET_SIZE} or 1. Sending anyway.")

            with serial_lock:
                bytes_written = serial_port.write(data_to_send)
                rospy.loginfo(f"Wrote {bytes_written} bytes to serial port: {data_to_send.hex()} (hex)")

        except serial.SerialException as e:
            rospy.logerr(f"Serial write error: {e}")
        except ValueError as e:
             rospy.logerr(f"Error converting data for serial port (ValueError): {e}. Original data: {msg.data}")
        except TypeError as e:
             rospy.logerr(f"Error converting data for serial port (TypeError): {e}. Original data type: {type(msg.data)}")
        except Exception as e:
             rospy.logerr(f"Error writing to serial port: {e}")


    else:
        rospy.logwarn("Serial port not available, cannot send data.")

def main():
    global serial_port
    rospy.init_node('pico_serial_bridge')

    port_name = rospy.get_param('~port', '/dev/ttyACM0')
    baud_rate = rospy.get_param('~baudrate', 115200)

    rospy.loginfo(f"Attempting to connect to serial port {port_name} at {baud_rate} baud.")

    try:
        serial_port = serial.Serial(port_name, baud_rate, timeout=SERIAL_TIMEOUT)
        rospy.loginfo("Serial port opened successfully.")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port {port_name}: {e}")
        sys.exit(1)
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred opening serial port: {e}")
        sys.exit(1)

    pub_serial_rx = rospy.Publisher('/pico/serial_rx', ByteMultiArray, queue_size=10)

    rospy.Subscriber('/pico/serial_tx', ByteMultiArray, serial_tx_callback)

    read_thread = threading.Thread(target=serial_read_thread, args=(pub_serial_rx,))
    read_thread.daemon = True
    read_thread.start()

    rospy.loginfo("Pico Serial Bridge node started.")
    rospy.spin()

    rospy.loginfo("Shutting down Pico Serial Bridge node.")
    if serial_port and serial_port.is_open:
        serial_port.close()
        rospy.loginfo("Serial port closed.")
    if read_thread.is_alive():
        read_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
