import time
import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)

pca.frequency = 50

center = 4700
left = 3700
right = 5700

print("right")
pca.channels[0].duty_cycle = right
time.sleep(3)

print("left")
pca.channels[0].duty_cycle = left
time.sleep(3)

print("center")
pca.channels[0].duty_cycle = center
time.sleep(3)

pca.deinit()