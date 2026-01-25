import serial
import time
import csv

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

command = 0b11110110
ser.write(bytes([command]))

motor = {
				"M1": [],
				"M2": [],
				"M3": [],
				"M4": [],
				"time": [],
				}
				
start = time.time()
while 1:
	byte1 = ser.read(1)
	while byte1 == b'\xAA':
		byte2 = ser.read(2)
		#print(f"M1: {byte2}")
		motor["M1"].append((byte2[1] << 8) | byte2[0])
		byte1 = ser.read(1)
		if byte1 == b'\xAB':
			byte2 = ser.read(2)
			#print(f"M2: {byte2}")
			motor["M2"].append((byte2[1] << 8) | byte2[0])
			byte1 = ser.read(1)
			if byte1 == b'\xAC':
				byte2 = ser.read(2)
				#print(f"M3: {byte2}")
				motor["M3"].append((byte2[1] << 8) | byte2[0])
				byte1 = ser.read(1)
				if byte1 == b'\xAD':
					byte2 = ser.read(2)
					#print(f"M4: {byte2}")
					motor["M4"].append((byte2[1] << 8) | byte2[0])
					
					end = time.time()
					motor["time"].append(int(end - start))
	if (int(end - start)) >= 90: break
					
print(motor["M1"])
print(motor["M2"])
print(motor["M3"])
print(motor["M4"])

rows = []

fields = ["M1", "M2", "M3", "M4", "time"]
for i in range(len(motor["M1"])):
	row = []
	row.append(motor["M1"][i])
	row.append(motor["M2"][i])
	row.append(motor["M3"][i])
	row.append(motor["M4"][i])
	row.append(motor["time"][i])
	
	rows.append(row)

print(rows)

with open('motor.csv', 'w', newline='') as f:
	writer = csv.writer(f)
	writer.writerow(fields)
	writer.writerows(rows)
		

