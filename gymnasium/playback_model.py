#!/usr/bin/env python3

import sys

import gymnasium as gym
from stable_baselines3 import SAC

if __name__ == "__main__":
    model_path: str = sys.argv[1]

    environment = "Humanoid-v5"

    env = gym.make(environment, render_mode="human")
    model = SAC.load(model_path)

    observation, info = env.reset()
    for i in range(1000):
        # action = env.action_space.sample()  # agent policy that uses the observation and info
        action, _ = model.predict(observation, deterministic=True)
        observation, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            observation, info = env.reset()

    env.close()
