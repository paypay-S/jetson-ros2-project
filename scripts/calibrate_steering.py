#!/usr/bin/env python3
import time
import sys
import os

# Adafruit libraries
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except ImportError:
    print("Error: Adafruit-Blinka or adafruit-circuitpython-pca9685 not installed.")
    sys.exit(1)

def main():
    print("=== F1TENTH Steering & ESC Calibration Tool ===")
    print("This tool directly controls PCA9685 channels to find the correct duty cycles.")
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50
    except Exception as e:
        print(f"Failed to initialize PCA9685: {e}")
        sys.exit(1)

    steer_ch = 0
    esc_ch = 1
    
    # Defaults
    steer_val = 4700
    esc_val = 5200 # STOP
    
    print("\nControls:")
    print("  'a' / 'd' : Steering Left / Right (+-50)")
    print("  'w' / 's' : ESC Forward / Reverse (+-50)")
    print("  'space'   : Reset to Center/Stop")
    print("  'q'       : Quit")
    print("-" * 30)

    import termios
    import tty

    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    print(f"Current State: Steer={steer_val}, ESC={esc_val}")
    
    pca.channels[steer_ch].duty_cycle = steer_val
    pca.channels[esc_ch].duty_cycle = esc_val

    try:
        while True:
            char = getch()
            if char == 'q':
                break
            elif char == 'a':
                steer_val = max(3000, steer_val - 50)
            elif char == 'd':
                steer_val = min(7000, steer_val + 50)
            elif char == 'w':
                esc_val = min(6500, esc_val + 50)
            elif char == 's':
                esc_val = max(3500, esc_val - 50)
            elif char == ' ':
                steer_val = 4700
                esc_val = 5200
            
            print(f"\rManual Set -> Steer: {steer_val} | ESC: {esc_val}    ", end="")
            pca.channels[steer_ch].duty_cycle = steer_val
            pca.channels[esc_ch].duty_cycle = esc_val

    except KeyboardInterrupt:
        pass
    finally:
        print("\nCleaning up...")
        pca.channels[steer_ch].duty_cycle = 4700
        pca.channels[esc_ch].duty_cycle = 5200
        pca.deinit()
        print("Done.")

if __name__ == "__main__":
    main()
