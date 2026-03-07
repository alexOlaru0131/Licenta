import serial
import time
import csv

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

command = 0b11110110
ser.write(bytes([command]))

		

