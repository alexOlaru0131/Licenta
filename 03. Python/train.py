from imports import *

class RobotGlobalEnvironment(gym.Env):
    def __init__(self):
        super().__init__()

        self.v_max = 30
        self.w_max = 2
        self.action_space = spaces.Box(
            low = np.array([0, 0], dtype=np.float32),
            high = np.array([self.v_max, self.w_max], dtype=np.float32),
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
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._agent_atts = np.array([0, 0, random.uniform(-np.pi / 2, np.pi / 2)], dtype=float)
        self._targets_locations = np.array(
            [
                random.uniform(-0.2, 0.2),
                random.uniform(0.9, 1.0),
                random.uniform(-0.3, 0.3),
                random.uniform(1.9, 2.0),
                random.uniform(-0.4, 0.4),
                random.uniform(2.9, 3.0),
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
        
        v = np.clip(action[0], 0, self.v_max)
        w = np.clip(action[1], -self.w_max, self.w_max)

        reward = 0
        truncated = False
        terminated = False

        r = 3.25
        d = 8
        dt = 0.01

        self._agent_atts[0] += v * np.cos(self._agent_atts[2]) * dt
        self._agent_atts[1] += v * np.sin(self._agent_atts[2]) * dt
        self._agent_atts[2] += w * dt
        self._agent_atts[2] = (self._agent_atts[2] + np.pi) & (2 * np.pi) - np.pi

        target_x = self._targets_locations[2*self.current_target]
        target_y = self._targets_locations[2*self.current_target + 1]

        dist = np.sqrt(
            (self._agent_atts[0] - target_x)**2 +
            (self._agent_atts[1] - target_y)**2
        )
        reward = self.prev_dist - dist
        reward -= 0.01

        if dist < 0.1:
            reward += 10.0
            self.current_target += 1

            if self.current_target >= 2:
                reward += 50.0
                terminated = True
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
    obs, info = rge.reset()
    print(obs, info)