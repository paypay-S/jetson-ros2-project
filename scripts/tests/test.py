from stable_baselines3 import PPO
import numpy as np

model = PPO.load("model.zip")

obs = np.zeros(1080)

action, _ = model.predict(obs)

print(action)