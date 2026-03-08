from imports import *

class RenderCallback(BaseCallback):

    def __init__(self, render_freq=50):
        super().__init__()
        self.render_freq = render_freq

    def _on_step(self) -> bool:

        if self.n_calls % self.render_freq == 0:
            env = self.training_env.envs[0]
            env.render()

        return True

class RobotGlobalEnvironment(gym.Env):
    def __init__(self):
        super().__init__()

        self.v_max = 0.975
        self.w_max = 1
        self.action_space = spaces.Box(
            low = np.array([-1, -1], dtype=np.float32),
            high = np.array([1, 1], dtype=np.float32),
            dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=np.array([-10.0, -10.0, -1.0, -1.0], dtype=np.float32),
            high=np.array([10.0, 10.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self._agent_atts = np.array([0, 0, 0], dtype=float)
        self._targets_locations = np.array([0, 0, 0, 0, 0, 0], dtype=float)
        self.current_target = 0

        self.max_steps = 500
        self.steps = 0
        self.prev_dist = 0

        self.history = []
        self.fig = None
        self.ax = None
        self.render_mode = ["human"]

    def render(self):

        if self.fig is None:
            plt.ion()
            self.fig, self.ax = plt.subplots()

        self.ax.clear()

        x = self._agent_atts[0]
        y = self._agent_atts[1]
        theta = self._agent_atts[2]

        self.ax.scatter(x, y, color="blue", s=100)

        self.ax.arrow(
            x,
            y,
            0.2 * np.cos(theta),
            0.2 * np.sin(theta),
            head_width=0.05,
            color="blue"
        )

        for i in range(3):
            tx = self._targets_locations[2*i]
            ty = self._targets_locations[2*i+1]
            self.ax.scatter(tx, ty, color="red", s=80)

        if len(self.history) > 1:
            hx, hy = zip(*self.history)
            self.ax.plot(hx, hy, color="green")

        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-0.5, 4)

        self.ax.set_aspect("equal")
        self.ax.grid()

        plt.pause(0.01)
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._agent_atts = np.array([0, 0, np.pi / 2], dtype=float)
        self._targets_locations = np.array(
            [
                random.uniform(-0.2, 0.2),
                random.uniform(0.8, 1.2),
                random.uniform(-0.4, 0.4),
                random.uniform(1.8, 2.2),
                random.uniform(-0.6, 0.6),
                random.uniform(2.8, 3.2),
            ]
            )
        self.current_target = 0
        self.steps = 0

        target_x = self._targets_locations[0]
        target_y = self._targets_locations[1]
        self.prev_dist = np.sqrt(
            (self._agent_atts[0] - target_x) ** 2 +
            (self._agent_atts[1] - target_y) ** 2
        )

        self.history = []
        
        observation = self._get_obs()
        info = self._get_info()

        return observation, info
        
    def _get_obs(self):
        target_x = self._targets_locations[2 * self.current_target]
        target_y = self._targets_locations[2 * self.current_target + 1]

        dx = target_x - self._agent_atts[0]
        dy = target_y - self._agent_atts[1]
        theta = self._agent_atts[2]

        return np.array(
            [dx, dy, np.cos(theta), np.sin(theta)],
            dtype=np.float32
        )

    def _get_info(self):
        distances = []
        for i in range(3):
            tx = self._targets_locations[2 * i]
            ty = self._targets_locations[2 * i + 1]
            dist = np.sqrt(
                (self._agent_atts[0] - tx) ** 2 +
                (self._agent_atts[1] - ty) ** 2
            )
            distances.append(dist)

        return {
            "distances": distances,
            "current_target": self.current_target
        }
    
    def step(self, action):
        self.steps += 1
        truncated = self.steps >= self.max_steps
        terminated = False
        
        v = action[0] * self.v_max
        w = action[1] * self.w_max

        reward = 0
        real_dt = 0.4
        step_dt = 0.01

        steps = int(real_dt / step_dt)

        for _ in range(steps):
            self._agent_atts[0] += v * np.cos(self._agent_atts[2]) * step_dt
            self._agent_atts[1] += v * np.sin(self._agent_atts[2]) * step_dt
            self._agent_atts[2] += w * step_dt
            self._agent_atts[2] = (self._agent_atts[2] + np.pi) % (2 * np.pi) - np.pi

        target_x = self._targets_locations[2*self.current_target]
        target_y = self._targets_locations[2*self.current_target + 1]

        dist = np.sqrt(
            (self._agent_atts[0] - target_x)**2 +
            (self._agent_atts[1] - target_y)**2
        )
        reward = self.prev_dist - dist

        dx = target_x - self._agent_atts[0]
        dy = target_y - self._agent_atts[1]
        theta = self._agent_atts[2]
        target_angle = np.arctan2(dy, dx)
        heading_error = target_angle - theta
        heading_error = (heading_error + np.pi) % (2*np.pi) - np.pi

        reward += 0.1 * np.cos(heading_error)

        if dist < 0.1:
            reward += 10.0
            self.current_target += 1

            if self.current_target >= 3:
                reward += 50.0
                terminated = True
                self.current_target -= 1
            else:
                next_tx = self._targets_locations[2 * self.current_target]
                next_ty = self._targets_locations[2 * self.current_target + 1]
                dist = np.sqrt(
                    (self._agent_atts[0] - next_tx) ** 2 +
                    (self._agent_atts[1] - next_ty) ** 2
                )
        
        self.prev_dist = dist
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, truncated, info

if __name__ == "__main__":
    rge = RobotGlobalEnvironment()
    check_env(rge)

    def make_env():
        env = RobotGlobalEnvironment()
        env = Monitor(env)
        return env

    vec_env = DummyVecEnv([make_env])

    MODEL_PATH = "ppo_robot"
    if os.path.exists(MODEL_PATH + ".zip"):
        model = PPO.load(MODEL_PATH, env=vec_env)
    else:
        policy_kwargs = dict(
                        net_arch=[256, 256]
                        )
        model = PPO(
                    "MlpPolicy",
                    vec_env,
                    policy_kwargs=policy_kwargs,
                    verbose=1,
                    tensorboard_log="./logs"
                    )
    callback = RenderCallback()
    model.learn(
                total_timesteps=300_000,
                tb_log_name="ppo_robot",
                callback=callback
                )
    model.save("robot_nn")

    # obs = vec_env.reset()

    # while True:
    #     action, _ = model.predict(obs, deterministic=True)
    #     obs, reward, dones, infos = vec_env.step(action)

    #     vec_env.render()

    #     if dones[0]:
    #         obs = vec_env.reset()

    # tensorboard --logdir logs
    # http://localhost:6006