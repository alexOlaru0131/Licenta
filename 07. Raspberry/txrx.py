from imports import *
import motors

moves = Queue()

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

too_close_flag = Event()

def run():
	while True:
		message = 0x0
		try:
			message = ser.read(1)
		except Exception as e:
			print(f"{e}")
		
		if message is not 0x0:
			match message:
				case 0xFA:
					message = ser.read(1)
					motors.M1.rotations = message
				
				case 0xFB:
					message = ser.read(1)
					motors.M2.rotations = message

				case 0xFC:
					message = ser.read(1)
					motors.M3.rotations = message

				case 0xFD:
					message = ser.read(1)
					motors.M4.rotations = message
				
				case 0xFF:
					message = ser.read(1)
					too_close_flag.set()

				case _: pass
					
		while not moves.empty():
			try:
				move = moves.get_nowait()
			except Exception as e:
				print(f"{e}")
				return False

		if move:
			print(move)
			match move[0]:
				case "wheels":
					command = 0x0
					match move[1]:
						case 0:
							command = command | (3 << 6) # virare dreapta

						case 1:
							command = command | (3 << 4) # virare stanga

						case 2:
							command = command | (15 << 4) # toate rotile

						case _: pass
					
					match move[2]:
						case 0: pass # 0%

						case 1:
							command = command | (1 << 1) # 20%

						case 2:
							command = command | (2 << 1) # 40%
						
						case 3:
							command = command | (3 << 1) # 60%
						
						case 4:
							command = command | (4 << 1) # 80%

						case 5:
							command = command | (5 << 1) # 100%

						case _: pass

					command = command | move[3]

					code = 0xAA
					ser.write(bytes([code]))
					ser.write(bytes([command]))

					print(command)
				
				case "servo":
					command = move[0] % 18
					code = 0xAB
					ser.write(bytes([code]))
					ser.write(bytes([command]))
				
				case _: pass