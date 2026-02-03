from imports import *

moves = Queue()

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def run():
	while True:
		message = 0xAA
		try:
			message = ser.read(1)
		except Exception as e:
			print(f"{e}")

		move = ()
		while not moves.empty():
			try:
				move = moves.get_nowait()
			except Exception as e:
				print(f"{e}")
				return False

		if move:
			match move[1]:
				case "wheels":
					match move[0]:
						case 0:
							command = 0b11001000
							code = 0xAA
							ser.write(bytes([code]))
							ser.write(bytes([command]))

						case 1:
							command = 0b00111000
							code = 0xAA
							ser.write(bytes([code]))
							ser.write(bytes([command]))

						# TO DO: IMPLEMENT THE REMAINING ACTIONS

						case _: pass
				
				case "servo":
					command = move[0]
					code = 0xAB
					ser.write(bytes([code]))
					ser.write(bytes([command]))
				
				case _: pass