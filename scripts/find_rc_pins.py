import Jetson.GPIO as GPIO
import time
import sys

# 調査対象のピン（一般的なGPIOとして使えるものをピックアップ）
# 7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40
MONITOR_PINS = [7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40]

def monitor():
    GPIO.setmode(GPIO.BOARD)
    
    # 全てのピンを入力として設定
    for pin in MONITOR_PINS:
        try:
            GPIO.setup(pin, GPIO.IN)
        except Exception:
            pass # 使えないピンは飛ばす
            
    # 各ピンのパルス幅を計測
    pulse_counts = {pin: 0 for pin in MONITOR_PINS}
    last_states = {pin: GPIO.input(pin) for pin in MONITOR_PINS}
    
    print("-" * 50)
    print("【ピン調査開始】以下の指示に従ってください：")
    print("1. プロポのステアリング(ハンドル)を左右に何度も動かしてください")
    print("2. プロポのスロットル(アクセル)を前後に何度も動かしてください")
    print("3. 何らかの反応があれば下のリストが更新されます。約15秒間継続します")
    print("-" * 50)

    start_time = time.time()
    while time.time() - start_time < 15:
        for pin in MONITOR_PINS:
            try:
                current_state = GPIO.input(pin)
                if current_state != last_states[pin]:
                    pulse_counts[pin] += 1
                    last_states[pin] = current_state
            except Exception:
                continue
                
        # 1秒ごとに中間報告
        elapsed = int(time.time() - start_time)
        if int(time.time() * 10) % 10 == 0:
            active_pins = [pin for pin, count in pulse_counts.items() if count > 50] # ある程度変化があるもの
            sys.stdout.write(f"\r経過時間: {elapsed}s | 反応あり候補ピン: {active_pins}    ")
            sys.stdout.flush()
        
    GPIO.cleanup()
    print("\n" + "-" * 50)
    print("調査終了。反応があったピンのリスト：")
    for pin, count in pulse_counts.items():
        if count > 50:
            print(f"Pin {pin:2} : 反応回数 {count:5} (候補)")
    print("-" * 50)
    print("この結果を教えてください。")

if __name__ == "__main__":
    try:
        monitor()
    except KeyboardInterrupt:
        GPIO.cleanup()
