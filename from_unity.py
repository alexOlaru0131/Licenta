from imports import *

tof_image_from_unity = Queue()

def run(port) -> bool:
    unity_comms = UnityComms(port=port)
    # print(port)
    rigid = unity_comms.SendRigid()
    # print(rigid)
    
    tof_image = rigid["dist"]
    tof_image = np.flip(tof_image, axis=0)
    tof_image = tof_image / 10

    tof_image_from_unity.put(tof_image)

    # print(tof_image)