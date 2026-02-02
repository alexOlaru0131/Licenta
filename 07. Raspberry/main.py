from imports import *
import vision_process
import transmit
import servo

MAX_DISTANCE = 4000
confidence_value = 30

DISTANCE_ON_STEP_20 = 0.02
DISTANCE_ON_STEP_40 = 0.04
DISTANCE_ON_STEP_60 = 0.06
DISTANCE_ON_STEP_80 = 0.08
DISTANCE_ON_STEP_100 = 0.1
ROTATION_ON_STEP = 0.03

def on_confidence_changed(value):
    global confidence_value
    confidence_value = value

def main():
    cam = ac.ArducamCamera()
    cfg_path = None

    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    transmit_thread = Process(
        target=transmit.run,
        args=()
    )
    transmit_thread.daemon = True
    transmit_thread.start()

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            # confidence_buf = frame.confidence_data

            result_image = depth_buf / 1000
            # print(result_image)

            # cv.normalize(confidence_buf, confidence_buf, 1, 0, cv.NORM_MINMAX)
            # confidence_buf = cv.flip(confidence_buf, 0)
            tof_image = cv.flip(result_image, 0)

            cv.imshow("ToF", tof_image)
            current_depth = tof_image

            vision_thread = Process(
                target=vision_process.vision_thread,
                args=(tof_image,)
            )
            vision_thread.start()
            vision_thread.join()

            points = []
            while not vision_process.points_list.empty():
                try:
                    point = list(vision_process.points_list.get_nowait())
                    points.append([point[0], point[1]])
                except Exception:
                    break

            mean_map = np.zeros((36, 48))
            while not vision_process.mean_map_queue.empty():
                try:
                    mean_map = vision_process.mean_map_queue.get_nowait()
                except Exception as e:
                    print(f"{e}")
                    break

            track_map = np.zeros((36, 48))
            while not vision_process.track_map_queue.empty():
                try:
                    track_map = vision_process.track_map_queue.get()
                except Exception:
                    break

            cv.imshow("Mean map", mean_map)
            cv.imshow("Track map", track_map)

            points_x = []
            points_y = []
            for point in points:
                points_x.append(point[0])
                points_y.append(point[1])

            distances = [0 for i in range(5)]
            gotTargets = [False for i in range(5)]
            target_point = 0
            robot_rotation = 0

            # print(f'Target point: {target_point}')
            observation = []
            # print(f"State: {i}")
            # print(f"Points: {points}")

            if points == []:
                for i in range(5):
                    points.append([0, 0])

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
                "/home/alexo/LicentaGit/Licenta/07. Raspberry/Training.onnx"
            )

            mask_input = session.get_inputs()[1]
            num_actions = mask_input.shape[1]

            action_masks = np.ones((1, num_actions), dtype=np.float32)

            output = session.run(
                ["discrete_actions"],
                {
                    "obs_0": observation_format,
                    "action_masks": action_masks,
                },
            )[0]
            output = int(output[0])
            # print(f"Action: {output}")
            transmit.moves.put((output, "wheels"))

            match output:
                case 0:
                    robot_rotation += ROTATION_ON_STEP

                case 1:
                    robot_rotation -= ROTATION_ON_STEP
                
                case _: pass
            
            # print(orientation)

            cam.releaseFrame(frame)

        key = cv.waitKey(1)
        if key == ord("q"):
            break

    cam.stop()
    cam.close()

if __name__ == "__main__":
    main_thread = Thread(target=main, args=())
    main_thread.start()