import serial
import threading

def get_serial_output():
	with open("robot_output.txt", "a", buffering=1) as output_file:
		while True:
			data = arduino.readline()
			if data:
				output_file.write(data)
				print(data.rstrip('\n'))


# (port, baudrate, connection timeout)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)

t1 = threading.Thread(target=get_serial_output, name='sout')

t1.start()

while True:
	message = raw_input()
	arduino.write(message + '\n')
