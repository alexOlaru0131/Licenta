from imports import *

steps = 5
time_horizon = 30
points_list = Queue(maxsize=1000)
commands_list = Queue(maxsize=1000)

DISTANCE_ON_STEP_20 = 0.02
DISTANCE_ON_STEP_40 = 0.04
DISTANCE_ON_STEP_60 = 0.06
DISTANCE_ON_STEP_80 = 0.08
DISTANCE_ON_STEP_100 = 0.1
ROTATION_ON_STEP = 0.03

def sample_sequence(time_horizon, current_action):
        sequence = []
        for i in range(time_horizon):
                if random.random() < .5:
                        sequence.append(current_action)
                else:
                        sequence.append(random.randint(0, 7))
        return sequence

def simulate_step(observation, action, robot_rotation):
        next_obs = observation

        # print(f"Here2")
        # print(f"{action}")

        match action:
            case 0:
                # logica de trimitere mesaj la STM
                c = np.cos(ROTATION_ON_STEP)
                s = np.sin(ROTATION_ON_STEP)
                for i in range(5):
                    next_obs[2*i] = c * next_obs[2*i] + s * next_obs[2*i + 1]
                    next_obs[2*i + 1] = -s * next_obs[2*i] + c * next_obs[2*i + 1]
                robot_rotation += ROTATION_ON_STEP

            case 1:
                # logica de trimitere mesaj la STM
                c = np.cos(-ROTATION_ON_STEP)
                s = np.sin(-ROTATION_ON_STEP)
                for i in range(5):
                    next_obs[2*i] = c * next_obs[2*i] + s * next_obs[2*i + 1]
                    next_obs[2*i + 1] = -s * next_obs[2*i] + c * next_obs[2*i + 1]
                robot_rotation -= ROTATION_ON_STEP

            # PWM 20%
            case 2:
                # logica de trimitere mesaj la STM
                noise_x = random.random() / 100
                noise_y = random.random() / 100
                x = DISTANCE_ON_STEP_20 * np.sin(robot_rotation) + noise_x
                y = DISTANCE_ON_STEP_20 * np.cos(robot_rotation) + noise_y
                # print(f'x: {x} y: {y}')
                for i in range(5):
                    next_obs[2*i] -= x
                    next_obs[2*i + 1] -= y

            # PWM 40%
            case 3:
                # logica de trimitere mesaj la STM
                noise_x = random.random() / 100
                noise_y = random.random() / 100
                x = DISTANCE_ON_STEP_40 * np.cos(robot_rotation) + noise_x
                y = DISTANCE_ON_STEP_40 * np.sin(robot_rotation) + noise_y
                # print(f'x: {x} y: {y}')
                for i in range(5):
                    next_obs[2*i] -= x
                    next_obs[2*i + 1] -= y

            # PWM 60%
            case 4:
                # logica de trimitere mesaj la STM
                noise_x = random.random() / 100
                noise_y = random.random() / 100
                x = DISTANCE_ON_STEP_60 * np.cos(robot_rotation) + noise_x
                y = DISTANCE_ON_STEP_60 * np.sin(robot_rotation) + noise_y
                # print(f'x: {x} y: {y}')
                for i in range(5):
                    next_obs[2*i] -= x
                    next_obs[2*i + 1] -= y

            # PWM 80%
            case 5:
                # logica de trimitere mesaj la STM
                noise_x = random.random() / 100
                noise_y = random.random() / 100
                x = DISTANCE_ON_STEP_80 * np.cos(robot_rotation) + noise_x
                y = DISTANCE_ON_STEP_80 * np.sin(robot_rotation) + noise_y
                # print(f'x: {x} y: {y}')
                for i in range(5):
                    next_obs[2*i] -= x
                    next_obs[2*i + 1] -= y

            # PWM 100%
            case 6:
                # logica de trimitere mesaj la STM
                noise_x = random.random() / 100
                noise_y = random.random() / 100
                x = DISTANCE_ON_STEP_100 * np.sin(robot_rotation) + noise_x
                y = DISTANCE_ON_STEP_100 * np.cos(robot_rotation) + noise_y
                # print(f'x: {x} y: {y}')
                for i in range(5):
                    next_obs[2*i] -= x
                    next_obs[2*i + 1] -= y
                #     print(f"{x} {y}")
            
        for i in range(5):
                angle = np.atan2(next_obs[2*i], next_obs[2*i + 1])
                distance = np.sqrt(np.pow(0 - next_obs[2*i], 2) + np.pow(0 - observation[2*i + 1], 2))
                next_obs[15 + i] = angle
                next_obs[10 + i] = distance

                if np.abs(next_obs[2*i]) < 0.1 and np.abs(next_obs[2*i + 1]) < 0.1:
                      next_obs[20+i] = True

        return next_obs, robot_rotation

def cost_function(observation) -> float:
        cost = .0

        for i in range(5):
              if observation[20 + i] == False:
                    cost += observation[10 + i]

        target = 0
        for i in range(5):
              if observation[20 + i] == True:
                    target = i
                    break
        
        # print(f"{observation[15 + target]}")
        if np.abs(observation[15 + target]) > 0.5:
              cost += 100

        return cost

def check_distance(observation):
        checked_obs = observation.copy()
        for i in range(5):
            if checked_obs[2*i] < 0.1 and np.abs(checked_obs[2*i+1]) < 0.2:
                  checked_obs[20 + i] = True

        return checked_obs

def mpc_thread(observation, current_action, robot_rotation):
        # 0 - 9 point x, y
        # 10 - 14 distances
        # 15 - 19 angles
        # 20 - 24 bools for points

        # print(f"{robot_rotation}")

        best_path = []
        current_rotation = robot_rotation
        commands_list.put(current_action)

        # print(f"{observation}")
        
        # best_obs_list = []
        best_next_obs = observation.copy()
        mpc_path = []
        for i in range(steps):
                sequence = sample_sequence(time_horizon, current_action)
                total_cost = 0
                best_cost = float("inf")
                current_obs = best_next_obs.copy()
                mpc_path.append(current_obs[0:10].copy())
                # print(current_obs)
                for j in range(time_horizon):
                        # print(f"Observation: {next_obs}\n")
                        next_obs, current_rotation = simulate_step(current_obs, sequence[j], current_rotation)
                        checked_obs = check_distance(next_obs)
                        task_cost = cost_function(checked_obs)
                        total_cost += task_cost
                        # print(f"{total_cost}")
                        
                        current_obs = checked_obs.copy()
                        # print(f"Here2 {j} ")

                        if total_cost < best_cost:
                                best_cost = total_cost
                                best_next_obs = next_obs.copy()
                                current_best_action = sequence[j]
                                current_action = sequence[j]

                #       best_obs_list = obs_list.copy()

                commands_list.put(current_best_action)

                # break

        # print(len(best_path))
        # print(best_path)
        for path in mpc_path:
                points_list.put(path)
                # print(path)
                # print(commands_list.qsize())
        
        # for obs in best_obs_list:
        #       print(f"{obs}\n")
        
        # print(commands_list)

        