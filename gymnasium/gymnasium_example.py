from datetime import datetime

from stable_baselines3 import SAC

import gymnasium as gym

training_timesteps = 30_000_000
environment = "Humanoid-v5"
timestamp = datetime.now().strftime("%d-%m-%Y_%H:%M:%S")

env = gym.make(environment)
observation, info = env.reset()

model = SAC(
    "MlpPolicy",
    env,
    verbose=1,
    train_freq=(1, "episode"),
)
model.learn(total_timesteps=training_timesteps)
model.save(f"models/{model.__class__.__name__}_{environment}_{training_timesteps}step_{timestamp}")
env.close()

env = gym.make(environment, render_mode="human")
observation, info = env.reset()
for i in range(1000):
    # action = env.action_space.sample()  # agent policy that uses the observation and info
    action, _ = model.predict(observation, deterministic=True)
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()
