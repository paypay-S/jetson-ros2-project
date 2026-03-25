import sys
try:
    import board
    print("board import: OK")
except Exception as e:
    print(f"board import: FAILED - {e}")

try:
    import busio
    print("busio import: OK")
except Exception as e:
    print(f"busio import: FAILED - {e}")

try:
    from adafruit_pca9685 import PCA9685
    print("adafruit_pca9685 import: OK")
except Exception as e:
    print(f"adafruit_pca9685 import: FAILED - {e}")

try:
    import busio
    import board
    i2c = busio.I2C(board.SCL, board.SDA)
    print("I2C initialization: OK")
except Exception as e:
    print(f"I2C initialization: FAILED - {e}")
