from stable_baselines3 import PPO

model = PPO.load("model.zip")

print("model loaded successfully")