from imports import *

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def save_data():
	i = 0
	servo_command = 0

	motor = {
		"M1": [],
		"M2": [],
		"M3": [],
		"M4": [],
		"time": [],
		}

	while 1:
		if i >= 0 and i < 300:
			code = 0xAB
			if i >= 151:
				servo_command = 7
			ser.write(bytes([code]))
			ser.write(bytes([servo_command]))
			if i % 10 == 0:
				servo_command += 1
			servo_command = servo_command % 16
			
			code = 0xAA
			command = 0b11110000
			ser.write(bytes([command]))
			
		if i == 200:
			command = 0b11110010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 300:
			command = 0b11110100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 400:
			command = 0b11110110
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 500:
			command = 0b11111000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))

		if i == 600:
			command = 0b11111010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
		
		if i == 700:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 800:
			command = 0b11000010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))

		if i == 900:
			command = 0b11000100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 1000:
			command = 0b11000110
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
		
		if i == 1100:
			command = 0b11001000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))

		if i == 1200:
			command = 0b11001010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 1300:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 1400:
			command = 0b00110010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))

		if i == 1500:
			command = 0b00110100
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 1600:
			command = 0b00110110
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
		
		if i == 1700:
			command = 0b00111000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))

		if i == 1800:
			command = 0b00111010
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			
		if i == 1900:
			command = 0b00000000
			code = 0xAA
			ser.write(bytes([code]))
			ser.write(bytes([command]))
			break
			
		byte1 = ser.read(1)
		while byte1 == b'\xAA':
			byte2_M1 = ser.read(2)
			#print(f"M1: {byte2}")
			motor["M1"].append((byte2_M1[1] << 8) | byte2_M1[0])
			byte1 = ser.read(1)
			if byte1 == b'\xAB':
				byte2_M2 = ser.read(2)
				#print(f"M2: {byte2}")
				motor["M2"].append((byte2_M2[1] << 8) | byte2_M2[0])
				byte1 = ser.read(1)
				if byte1 == b'\xAC':
					byte2_M3 = ser.read(2)
					#print(f"M3: {byte2}")
					motor["M3"].append((byte2_M3[1] << 8) | byte2_M3[0])
					byte1 = ser.read(1)
					if byte1 == b'\xAD':
						byte2_M4 = ser.read(2)
						#print(f"M4: {byte2}")
						motor["M4"].append((byte2_M4[1] << 8) | byte2_M4[0])
						
						i += 1
						motor["time"].append(i)

		print(f"M1:{(byte2_M1[1] << 8) | byte2_M1[0]}, M2:{(byte2_M2[1] << 8) | byte2_M2[0]}, M3:{(byte2_M3[1] << 8) | byte2_M3[0]}, M4:{(byte2_M4[1] << 8) | byte2_M4[0]}")
		
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

if __name__ == "__main__":
	save_data()
		