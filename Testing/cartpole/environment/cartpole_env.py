import gym
import numpy as np
from gym import spaces


class CustomEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self, arg1, arg2):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = spaces.Discrete(2)
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.

    def step(self, action):
        ...
        return observation, reward, done, info

    def reset(self):
        ...
        return observation  # reward, done, info can't be included

    def render(self, mode="human"):
        ...

    def close(self):
        ...