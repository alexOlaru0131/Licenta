from imports import *
import vision_process
import transmission
import three_dim_env
import mpc

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

def main():
    for path_index in range(10):

        tof_image = cv.imread(
            "images/tof" + str(path_index + 1) + ".png", cv.IMREAD_GRAYSCALE
        )
        ir_image = cv.imread(
            "images/ir" + str(path_index + 1) + ".png", cv.IMREAD_GRAYSCALE
        )

        vision_thread = Thread(
            target=vision_process.vision_thread, args=(tof_image, ir_image)
        )
        vision_thread.daemon = True
        vision_thread.start()
        # print(f"Got here 1 {path_index} {vision_thread.is_alive()}")
        vision_thread.join()

        transmission_thread = Thread(
            target=transmission.transmission_thread,
        )
        transmission_thread.daemon = True
        transmission_thread.start()

        if not vision_process.turn_back_flag.is_set():

            plt.subplot(2, 3, 1)
            plt.imshow(tof_image)

            plt.subplot(2, 3, 3)
            plt.imshow(vision_process.mean_map)

            plt.subplot(2, 3, 4)
            plt.imshow(vision_process.track_map)

            plt.subplot(2, 3, 5)
            plt.imshow(vision_process.track_map_interpolated)

            plt.subplot(2, 3, 2)
            plt.imshow(ir_image)

            three_dim_env_thread = Thread(
                target=three_dim_env.three_dim_env_thread, args=(vision_process.mean_map,)
            )
            three_dim_env_thread.daemon = True
            three_dim_env_thread.start()
            three_dim_env_thread.join()

            ax = plt.subplot(2, 3, 6, projection="3d")
            ax.scatter(three_dim_env.P[:,0],
                        three_dim_env.P[:,1],
                        three_dim_env.P[:,2],
                        c = three_dim_env.dist,
                        cmap='viridis',
                        )
            
            # plt.show()
            points = []
            while not vision_process.points_list.empty():
                try:
                    point = list(vision_process.points_list.get_nowait())
                    # print(point)
                    points.append([point[0] / 100, point[1] / 100])
                except Exception:
                    break

            # print(points)
            # print(points)
            
            ax.view_init(elev=30)

            points_x = []
            points_y = []
            for point in points:
                points_x.append(point[0])
                points_y.append(point[1])
            ax.scatter(points_x, points_y, 0)
            distances = [0 for i in range(5)]
            gotTargets = [False for i in range(5)]
            target_point = 0
            robot_rotation = 0

            # print(f'Target point: {target_point}')
            observation = []
            # print(f"State: {i}")
            # print(f"Points: {points}")

            for i in range(len(distances)):
                if -0.1 < points[i][1] < 0.1:
                    gotTargets[i] = True

            # print(f"Bools: {gotTargets}")

            for point in points:
                observation.append(point[0])
                observation.append(point[1])

            for i in range(len(points)):
                # print(i)
                # print(calculated_first_dist)
                point = points[i]
                distance = np.sqrt(np.pow(0 - point[0], 2) + np.pow(0 - point[1], 2))
                distances[i] = distance
                observation.append(distance)

            angles = []
            for point in points:
                angle = np.atan2(point[1], point[0])
                angles.append(angle)
                observation.append(angle)

            for trueVal in gotTargets:
                observation.append(trueVal)

            staticFrictionCoefficient = dynamicFrictionCoefficient = 0
            observation.append(staticFrictionCoefficient)
            observation.append(dynamicFrictionCoefficient)

            # print(observation)

            observation_format = np.array(observation, dtype=np.float32).reshape(1, -1)

            # print(observation)

            session = onnxruntime.InferenceSession(
                "C:/LicentaRL/unity-rl-env/results/ppo/Training.onnx"
            )

            mask_input = session.get_inputs()[1]
            num_actions = mask_input.shape[1]

            action_masks = np.ones((1, num_actions), dtype=np.float32)

            # print(action_masks)

            # outs = session.get_outputs()
            # print("Output names:")
            # for o in outs:
            #     print(" ", o.name, o.shape)
            # print("Obs shape:", observation.shape)
            # print("Obs:", observation)
            output = session.run(
                ["discrete_actions"],
                {
                    "obs_0": observation_format,
                    "action_masks": action_masks,
                },
            )[0]
            output = int(output[0])
            # print(f"Action: {output}")

            match output:
                case 0:
                    robot_rotation += ROTATION_ON_STEP

                case 1:
                    robot_rotation -= ROTATION_ON_STEP
                
                case _: pass
            
            # print(f"{observation}")
            mpc_thread = Thread(
                target=mpc.mpc_thread, args=(observation, output, robot_rotation)
            )
            mpc_thread.daemon = True
            mpc_thread.start()
            mpc_thread.join()

            paths = []
            while not mpc.points_list.empty():
                try:
                    path = mpc.points_list.get_nowait()
                    paths.append(path)
                    # print(f"{len(path)}")
                    # print(path)
                    # mpc_points.append()
                except Queue:
                    break

            for path in paths:
                points_x = []
                points_y = []
                for i in range(len(path) // 2):
                    points_x.append(path[2*i])
                    points_y.append(path[2*i + 1])
                ax.scatter(points_x, points_y, 0)

            # print(f"Distances: {distances}")
            # print(f"Angles: {angles}")
        time.sleep(1)

        plt.show()

        vision_process.mean_map = np.zeros_like(vision_process.mean_map)
        vision_process.track_map = np.zeros_like(vision_process.track_map)
        vision_process.floor_map = np.zeros_like(vision_process.floor_map)
        vision_process.track_map_interpolated = np.zeros_like(
            vision_process.track_map_interpolated
        )
        vision_process.turn_back_flag.clear()
        # print(f"Got here 2 {path_index} {vision_thread.is_alive()}")

        break

# main
# -> rularea programului
if __name__ == "__main__":
    main_thread = Thread(target=main, args=())
    main_thread.start()
