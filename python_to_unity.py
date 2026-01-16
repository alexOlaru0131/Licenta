from imports import *
import vision_process
import three_dim_env
import mpc
import plot
import from_unity
import to_unity

HEIGHT_MEAN_SIZE = 5
WIDTH_MEAN_SIZE = 5

height = 180
width = 240

DISTANCE_ON_STEP_20 = 0.02
DISTANCE_ON_STEP_40 = 0.04
DISTANCE_ON_STEP_60 = 0.06
DISTANCE_ON_STEP_80 = 0.08
DISTANCE_ON_STEP_100 = 0.1
ROTATION_ON_STEP = 0.03

PORT = 9000

def run() -> None:
    while True:
        from_unity_thread = Thread(
             target=from_unity.run, args=(PORT,)
        )
        from_unity_thread.daemon = True
        from_unity_thread.start()
        from_unity_thread.join()

        if not from_unity.tof_image_from_unity.empty():
            tof_image = from_unity.tof_image_from_unity.get()

        # print(tof_image.shape())
 
        vision_thread = Thread(
            target=vision_process.vision_thread,
            args=(tof_image, tof_image)
        )
        vision_thread.daemon = True
        vision_thread.start()
        vision_thread.join()

        if not vision_process.turn_back_flag.is_set():

            plot.tof_image_queue.put(tof_image)
            plot.mean_map_queue.put(vision_process.mean_map)
            plot.track_map_queue.put(vision_process.track_map)
            plot.track_map_interpolated_queue.put(vision_process.track_map_interpolated)

            # three_dim_env_thread = Thread(
            #     target=three_dim_env.three_dim_env_thread, args=(tof_image,)
            # )
            # three_dim_env_thread.daemon = True
            # three_dim_env_thread.start()
            # three_dim_env_thread.join()

            # plot.P_queue.put(three_dim_env.P)
            # plot.dist_queue.put(three_dim_env.dist)
            
            points = []
            while not vision_process.points_list.empty():
                try:
                    point = list(vision_process.points_list.get_nowait())
                    points.append([point[0], point[1]])
                except Exception:
                    break

            to_unity.points_to_unity.put(points)
            to_unity_thread = Thread(
                target=to_unity.run,
                args=(PORT,)
            )
            to_unity_thread.daemon = True
            to_unity_thread.start()
            to_unity_thread.join()

        vision_process.mean_map = np.zeros_like(vision_process.mean_map)
        vision_process.track_map = np.zeros_like(vision_process.track_map)
        vision_process.floor_map = np.zeros_like(vision_process.floor_map)
        vision_process.track_map_interpolated = np.zeros_like(
            vision_process.track_map_interpolated
        )
        vision_process.turn_back_flag.clear()

        break

if __name__ == "__main__":
        main_thread = Thread(target=run, args=())
        main_thread.start()

        plot.plotter()