from imports import *

HEIGHT_MEAN_SIZE = 5
WIDTH_MEAN_SIZE = 5

height = 180
width = 240

points_list = Queue(maxsize=5)

mean_map = np.zeros((height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE))
mean_map_clear = np.zeros((height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE))
track_map = np.zeros((height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE))
floor_map = np.zeros((height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE))
track_map_interpolated = np.zeros(
    (height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE)
)

DISTANCE_MAX_VALUE = 0.2

turn_back_flag = Event()

def create_track(image) -> bool:
    global track_map_interpolated

    track_map[height // HEIGHT_MEAN_SIZE - 1][(width // WIDTH_MEAN_SIZE) // 2] = 1
    track_map[max_arg[0]][max_arg[1]] = 1
    track_map_interpolated = cv.bitwise_or(track_map_interpolated, track_map)

    finished_path = False
    dfs_path = []
    dfs_points_list = []
    dfs_path.append((height // HEIGHT_MEAN_SIZE - 1, (width // WIDTH_MEAN_SIZE) // 2))
    check_map = np.zeros((height // HEIGHT_MEAN_SIZE, width // WIDTH_MEAN_SIZE))

    i = 0
    while finished_path == False:
        start_point = dfs_path[i]
        check_map[start_point[0]][start_point[1]] = 1
        # print(start_point)
        # print(max_arg)
        n = 1
        try:
            if (
                check_map[start_point[0] + n][start_point[1] + n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] + n, start_point[1] + n))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0] - n][start_point[1] + n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] - n, start_point[1] + n))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0] + n][start_point[1] - n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] + n, start_point[1] - n))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0] - n][start_point[1] - n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] - n, start_point[1] - n))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0] + n][start_point[1]] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] + 1, start_point[1]))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0]][start_point[1] + n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0], start_point[1] + 1))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0] - n][start_point[1]] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0] - 1, start_point[1]))
        except Exception:
            pass
        try:
            if (
                check_map[start_point[0]][start_point[1] - n] == 0
                and start_point[0] + n >= max_arg[0]
            ):
                dfs_points_list.append((start_point[0], start_point[1] - 1))
        except Exception:
            pass

        min_distance_plane = 1000
        for point in dfs_points_list:
            check_map[point[0]][point[1]] = 1
            distance = mean_map[point[0]][point[1]]
            distance_plane = np.sqrt(
                np.pow(max_arg[0] - point[0], 2)
                + np.pow(max_arg[1] - point[1], 2)
                + np.pow(mean_map[max_arg[0]][max_arg[1]] - distance, 2)
            )

            if distance_plane < min_distance_plane and point[0] >= max_arg[0]:
                min_distance_plane = distance_plane
                temp0 = point[0]
                temp1 = point[1]

        # print(len(dfs_points_list))
        # print(dfs_points_list)
        dfs_points_list = []
        dfs_path.append((temp0, temp1))
        # print(dfs_path)
        track_map[temp0][temp1] = 1
        if temp0 == max_arg[0] and temp1 == max_arg[1]:
            break

        i += 1
        if i >= 100:
            turn_back_flag.set()
            return False

    n_dist = len(dfs_path) // 5

    start_point = (height // HEIGHT_MEAN_SIZE - 1, (width // WIDTH_MEAN_SIZE) // 2)
    points_list.put(
        (0, mean_map[height // HEIGHT_MEAN_SIZE - 1][(width // WIDTH_MEAN_SIZE) // 2])
    )
    
    point1 = dfs_path[2 * n_dist]
    point1 = ((start_point[0] + point1[0]) // 2, (start_point[1] + point1[1]) // 2)
    # print(point1)
    points_list.put((point1[1] - width // WIDTH_MEAN_SIZE // 2, mean_map[point1]))
    track_map_interpolated[point1] = 1

    point2 = dfs_path[3 * n_dist]
    point2 = ((point1[0] + point2[0]) // 2, (point1[1] + point2[1]) // 2)
    points_list.put((point2[1] - width // WIDTH_MEAN_SIZE // 2, mean_map[point2]))
    track_map_interpolated[point2] = 1

    point3 = dfs_path[4 * n_dist]
    point3 = ((point2[0] + point3[0]) // 2, (point2[1] + point3[1]) // 2)
    points_list.put((point3[1] - width // WIDTH_MEAN_SIZE // 2, mean_map[point3]))
    track_map_interpolated[point3] = 1

    points_list.put(
        (max_arg[0] - width // WIDTH_MEAN_SIZE // 2, mean_map[max_arg[0]][max_arg[1]])
    )

    return True

def extract_mean(tof_image) -> bool:
    global max_arg

    max = 0
    max_arg = []

    for i in range(height // HEIGHT_MEAN_SIZE):
        for j in range(width // WIDTH_MEAN_SIZE):
            # print(f'{i} {j}')
            tof_segment = tof_image[
                i * HEIGHT_MEAN_SIZE : (i + 1) * HEIGHT_MEAN_SIZE,
                j * WIDTH_MEAN_SIZE : (j + 1) * WIDTH_MEAN_SIZE,
            ]
            mean_map[i][j] = np.mean(tof_segment)

            # print(tof_segment)
    for i in range(height // HEIGHT_MEAN_SIZE):
        for j in range(width // WIDTH_MEAN_SIZE):
            if (
                mean_map[height // HEIGHT_MEAN_SIZE - i - 1][
                    width // WIDTH_MEAN_SIZE - j - 1
                ]
                > max
            ):
                max = mean_map[height // HEIGHT_MEAN_SIZE - i - 1][
                    width // WIDTH_MEAN_SIZE - j - 1
                ]
                max_arg = [
                    height // HEIGHT_MEAN_SIZE - i - 1,
                    width // WIDTH_MEAN_SIZE - j - 1,
                ]

    print(max_arg)
    
    return True


def vision_thread(tof_image, ir_image) -> bool:
    global mean_map, track_map

    valid_distance = extract_mean(tof_image)
    if not valid_distance:
        print("The robot will go back!")
        return False
    created_track = create_track(mean_map)
    if not created_track:
        print("The robot will go back!")
        return False

    # print("Thread finished")
