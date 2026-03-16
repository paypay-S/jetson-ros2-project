import time
import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)

pca.frequency = 50

STOP = 5200
FORWARD = 5800
REVERSE = 4000
print("ESC arm (stop)")
pca.channels[1].duty_cycle = STOP
time.sleep(5)

print("forward")
pca.channels[1].duty_cycle = FORWARD
time.sleep(2)

print("stop")
pca.channels[1].duty_cycle = STOP
time.sleep(2)

print("reverse")
pca.channels[1].duty_cycle = REVERSE
time.sleep(2)

print("stop")
pca.channels[1].duty_cycle = STOP
time.sleep(2)

pca.deinit()