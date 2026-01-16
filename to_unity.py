from imports import *

points_to_unity = Queue()

def run(port) -> bool:
    unity_comms = UnityComms(port)
    if not points_to_unity.empty():
        points = points_to_unity.get()
        print(f"points: {points}")
        unity_comms.GetPoints(
            data=[
                {
                    "x": float(p[0]),
                    "y": float(p[1])
                }
                for p in points
            ]
        )
