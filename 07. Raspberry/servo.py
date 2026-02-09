from imports import *
import vision_process
import txrx

orientation = Queue()
orientation.put(7)

def run():
    global orientation

    if vision_process.turn_back_flag.is_set():
        while not orientation.empty():
            try:
                o = orientation.get()
            except Exception:
                break
        
        o -= 1
        if o < 0: o = 0

        txrx.moves.put(("servo", o))
        orientation.put(o)

        return True
    
    while not orientation.empty():
        try:
            o = orientation.get()
        except Exception:
            break

    o = 7
    orientation.put(o)
    return True