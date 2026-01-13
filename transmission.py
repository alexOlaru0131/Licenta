from imports import *
import mpc

def transmission_thread():
        # ser = serial.Serial()
        ser = None

        while True:
                try:
                    command = mpc.commands_list.get()
                except Exception:
                    pass

                message = 0
                match command:
                        case 0:
                                message |= 1 << 5 | 1 << 7 | 1 << 1
                                # ser.write(bytearray(message))
                                print(message)

                        case 1:
                                message |= 1 << 4 | 1 << 6 | 1 << 1
                                # ser.write(bytearray(message))
                                print(message)

                        # PWM 20%
                        case 2:
                                message |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7
                                # ser.write(bytearray(message))
                                print(message)

                        # PWM 40%
                        case 3:
                                message |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7
                                # ser.write(bytearray(message))
                                print(message)

                        # PWM 60%
                        case 4:
                                message |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 2 << 1
                                # ser.write(bytearray(message))
                                print(message)

                        # PWM 80%
                        case 5:
                                message |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 2 << 1 | 3 << 1
                                # ser.write(bytearray(message))
                                print(message)

                        # PWM 100%
                        case 6:
                                message |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 1 << 1
                                # ser.write(bytearray(message))
                                print(message)