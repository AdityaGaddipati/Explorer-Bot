import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import re

baud = 57600
serial_port = '/dev/ttyUSB0'  # set the correct port before run it

ser = serial.Serial(port=serial_port, baudrate=baud)
ser.timeout = 2  # set read timeout

x = []
y = []
plt.grid(True)
plt.ion()

while True:
	size = ser.inWaiting()
	if size:
		data = ser.read(size)
		#print (str(int(1000*time.clock()))+" "+data) 		
		#print(str(re.search(r'\d+', data).group()))

		temp = re.findall(r"[-+]?\d*\.\d+|\d+", data)
		y.append(float(temp[3]))		
		x.append(int(1000*time.clock()))		
		#print data
		
		plt.scatter(x,y);		
		plt.show()
    		plt.pause(0.0001)

