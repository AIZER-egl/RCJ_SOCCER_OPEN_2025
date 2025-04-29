#!/usr/bin/env python3

import rospy
import serial
import struct
import sys
import threading
import logging # Importar el módulo logging
from std_msgs.msg import ByteMultiArray

SERIAL_PACKET_SIZE = 26
SERIAL_TIMEOUT = 0.1

serial_port = None
serial_lock = threading.Lock()

file_logger = logging.getLogger('SerialIO')
file_logger.setLevel(logging.INFO)
file_logger.propagate = False
try:
	fh = logging.FileHandler('serial_io.log', mode='w')
except Exception as e:
	rospy.logerr(f"Error al configurar el archivo de log serial_io.log: {e}")
	fh = None

if fh:
	formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
	fh.setFormatter(formatter)
	file_logger.addHandler(fh)


def serial_read_thread(pub_serial_rx):
	global serial_port, serial_lock, SERIAL_PACKET_SIZE, file_logger # Acceder al logger global
	rospy.loginfo("Serial read thread started.")

	if not serial_port or not serial_port.is_open:
		rospy.logerr("Serial port is not initialized or not open. Exiting thread.")
		return

	if serial_port.timeout is None:
		rospy.logwarn("Serial port timeout is not set! readline() could block indefinitely")

	while not rospy.is_shutdown():
		line_bytes = None
		try:
			rospy.logdebug("Read thread: Attempting to acquire lock...")
			t_before_read_lock = rospy.Time.now()
			with serial_lock:
				t_after_read_lock = rospy.Time.now()
				lock_wait_duration = (t_after_read_lock - t_before_read_lock).to_sec()
				rospy.logdebug(f"Read thread: Lock acquired (waited {lock_wait_duration:.4f}s). Calling readline...")

				t_before_readline = rospy.Time.now()
				line_bytes = serial_port.readline()
				t_after_readline = rospy.Time.now()
				readline_duration = (t_after_readline - t_before_readline).to_sec()

				rospy.loginfo(f"Read thread: Readline returned after {readline_duration:.4f}s. Lock held for this duration.")

			if line_bytes:
				if fh:
					file_logger.info(f"[IN]: {line_bytes.hex()}")

				rospy.loginfo(f"Received package with length: {len(line_bytes)} bytes")
				cleaned_bytes = line_bytes.rstrip()

				if len(cleaned_bytes) == SERIAL_PACKET_SIZE:
					rospy.loginfo(f"Received packet with expected length {len(cleaned_bytes)} bytes.")
				else:
					rospy.logwarn(f"Received packet with unexpected length: {len(cleaned_bytes)} bytes "
								  f"(expected {SERIAL_PACKET_SIZE}). Raw data (hex): {line_bytes.hex()}") # Log raw data

				signed_byte_list = []
				for byte_val in cleaned_bytes:
					if byte_val > 127:
						signed_byte_list.append(byte_val - 256)
					else:
						signed_byte_list.append(byte_val)

				msg = ByteMultiArray()
				msg.data = signed_byte_list
				pub_serial_rx.publish(msg)
				rospy.loginfo(f"Published {len(signed_byte_list)} bytes to /pico/serial_rx")

		except serial.SerialException as e:
			rospy.logerr(f"Serial read error: {e}. Closing port and exiting thread.")
			if serial_port and serial_port.is_open:
				with serial_lock:
					if serial_port.is_open:
						serial_port.close()
			break
		except Exception as e:
			rospy.logerr(f"Unexpected error in serial read thread: {e}", exc_info=True) # Añadir traceback
			rospy.sleep(1.0)

	rospy.loginfo("Serial read thread finished.")

def serial_tx_callback(msg):
	global serial_port, serial_lock, SERIAL_PACKET_SIZE, file_logger

	try:
		rospy.loginfo(f"DEBUG: Callback entered. msg.data type: {type(msg.data)}, value: {msg.data}")
	except Exception as e:
		rospy.logerr(f"DEBUG: Error accessing msg.data at start of callback: {e}")
		return

	rospy.loginfo(f"Received {len(msg.data)} numbers on /pico/serial_tx")
	t_callback_start = rospy.Time.now()

	if serial_port and serial_port.is_open:
		try:
			unsigned_byte_list = []
			for signed_val in msg.data:
				if -128 <= signed_val <= 255:
					unsigned_byte_list.append(signed_val & 0xff)
				else:
					rospy.logwarn(f"Valor inesperado {signed_val} encontrado. Usando 0.")
					unsigned_byte_list.append(0)

			data_to_send = bytes(unsigned_byte_list)

			expected_len = SERIAL_PACKET_SIZE + 1
			if len(data_to_send) != expected_len:
				rospy.logwarn(f"Data packet size to send is {len(data_to_send)} bytes, expected {expected_len}.")

			if fh:
				file_logger.info(f"[OUT]: {data_to_send.hex()}")

			rospy.loginfo(f"Attempting to acquire lock...")
			t_before_lock = rospy.Time.now()
			with serial_lock:
				t_after_lock = rospy.Time.now()
				lock_wait_duration = (t_after_lock - t_before_lock).to_sec()
				rospy.loginfo(f"Lock acquired (waited {lock_wait_duration:.4f}s). Calling write...")

				t_before_write = rospy.Time.now()
				bytes_written = serial_port.write(data_to_send) # Enviar los bytes convertidos
				t_after_write = rospy.Time.now()
				write_duration = (t_after_write - t_before_write).to_sec()
				rospy.loginfo(f"Write returned {bytes_written} bytes (took {write_duration:.4f}s). Calling flush...")

				t_before_flush = rospy.Time.now()
				serial_port.flush()
				t_after_flush = rospy.Time.now()
				flush_duration = (t_after_flush - t_before_flush).to_sec()
				rospy.loginfo(f"Flush returned (took {flush_duration:.4f}s). Releasing lock.")

		except serial.SerialException as e:
			rospy.logerr(f"Serial write error: {e}")
		except ValueError as e:
			rospy.logerr(f"Internal error during bytes conversion (ValueError): {e}. Original data: {msg.data}")
		except TypeError as e:
			rospy.logerr(f"Error converting data for serial port (TypeError): {e}. Original data type: {type(msg.data)}")
		except Exception as e:
			rospy.logerr(f"Error writing to serial port: {e}", exc_info=True) # Añadir traceback

	else:
		rospy.logwarn("Serial port not available, cannot send data.")


	t_callback_end = rospy.Time.now()
	callback_duration = (t_callback_end - t_callback_start).to_sec()
	rospy.loginfo(f"Callback finished (total duration {callback_duration:.4f}s)")

def main():
	global serial_port, file_logger # Acceder al logger global

	rospy.init_node('pico_serial_bridge')

	port_name = rospy.get_param('~port', '/dev/ttyACM0')
	baud_rate = rospy.get_param('~baudrate', 115200)

	rospy.loginfo(f"Attempting to connect to serial port {port_name} at {baud_rate} baud.")
	rospy.loginfo(f"Serial I/O data will be logged to: serial_io.log") # Informar al usuario

	try:
		serial_port = serial.Serial(port_name, baud_rate, timeout=SERIAL_TIMEOUT)
		rospy.loginfo("Serial port opened successfully.")
	except serial.SerialException as e:
		rospy.logerr(f"Failed to open serial port {port_name}: {e}")
		if fh: fh.close()
		sys.exit(1)
	except Exception as e:
		rospy.logerr(f"An unexpected error occurred opening serial port: {e}")
		if fh: fh.close()
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
	if fh:
		logging.shutdown()


if __name__ == '__main__':
	main()
