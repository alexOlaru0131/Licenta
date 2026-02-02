from imports import *
import vision_process
import transmit

def run():
        orientation = 7

        if vision_process.turn_back_flag.is_set():
            orientation -= 1
            orientation = 0 if orientation < 0 else orientation
            transmit.moves.put((orientation, "servo"))
            # print(orientation)
        
        else:
            orientation = 7