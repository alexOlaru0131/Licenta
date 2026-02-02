from imports import *

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

i = 0
servo_command = 0

def save_data():
	while 1:
		if i > 0 and i <= 30:
			code = 0xAB
			if i == 30:
				servo_command = 7
			ser.write(bytes([code]))
			ser.write(bytes([servo_command]))
			servo_command += 1
			servo_command = servo_command % 16
			
			code = 0xAA
			command = 0b11110000
			ser.write(bytes([command]))
			
		if i == 31:
			command = 0b11110010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 61:
			command = 0b11110100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 61:
			command = 0b11110110
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 91:
			command = 0b11111000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
		
		if i == 121:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 151:
			command = 0b11000100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 181:
			command = 0b11001000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 211:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 241:
			command = 0b00110100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 271:
			command = 0b00111000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 301:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 331:
			command = 0b11110100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 361:
			command = 0b11111000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i > 390: 
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			break
			
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
						
						i += 1
						motor["time"].append(i)
		
		print(i)
						
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
		