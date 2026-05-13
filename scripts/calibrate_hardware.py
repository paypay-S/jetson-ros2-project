#!/usr/bin/env python3
import sys
import time
import os
import re

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except ImportError:
    print("Error: Required libraries (adafruit-circuitpython-pca9685) not found.")
    sys.exit(1)

# params.yaml のパス
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = os.path.join(SCRIPT_DIR, '../ros2_ws/src/f1tenth_rl/config/params.yaml')

# チャンネル設定
STEER_CH = 0
ESC_CH = 1

def calibrate_step(pca, param_name, current_val, description, channel):
    """1つのパラメータをキャリブレーションする"""
    print(f"\n--- {param_name} の設定 ---")
    print(description)
    print("数値を入力して微調整し、Enterのみを押すと決定して次へ進みます。")
    print("---------------------------------------------------------")
    
    val = current_val
    pca.channels[channel].duty_cycle = val

    while True:
        user_input = input(f"[{param_name}] 現在の値 {val} > ").strip()
        
        if not user_input:
            # Enterのみ押されたら決定
            print(f"=>{param_name} を {val} に決定しました。\n")
            return val
            
        try:
            new_val = int(user_input)
            if not (2000 <= new_val <= 8000):
                print("値が範囲外(2000-8000)です。安全のため適用しません。")
                continue
            val = new_val
            pca.channels[channel].duty_cycle = val
        except ValueError:
            print("半角数字のみ入力するか、決定する場合はEnterを押してください。")

def main():
    print("=== F1TENTH Hardware Setup Wizard ===")
    print("タイヤやモーターを動かしながら、最適な数値を見つけて保存します。")
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50
    except Exception as e:
        print(f"I2C/PCA9685の初期化に失敗しました: {e}")
        return

    # 初期の安全値
    pca.channels[STEER_CH].duty_cycle = 4950
    pca.channels[ESC_CH].duty_cycle = 5200

    results = {}

    try:
        # --- ステアリング ---
        results['steer_center'] = calibrate_step(
            pca, 'steer_center', 4950, 
            "タイヤが完全に真っ直ぐ（センター）になる値を探してください。", STEER_CH
        )
        results['steer_left'] = calibrate_step(
            pca, 'steer_left', 3500, 
            "タイヤが左に限界まで切れ、かつサーボに負荷がかかりすぎない値を探してください。", STEER_CH
        )
        results['steer_right'] = calibrate_step(
            pca, 'steer_right', 6000, 
            "タイヤが右に限界まで切れ、かつサーボに負荷がかかりすぎない値を探してください。", STEER_CH
        )
        
        # センターに戻す
        pca.channels[STEER_CH].duty_cycle = results['steer_center']

        # --- モーター ---
        results['esc_stop'] = calibrate_step(
            pca, 'esc_stop', 5200, 
            "モーターが回転せず、完全に停止している値を探してください。", ESC_CH
        )
        results['esc_forward'] = calibrate_step(
            pca, 'esc_forward', 5800, 
            "前進時（普段走行するくらいの速度）の基準となる値を探してください。\n（※車体を浮かせるか、広い場所で行ってください）", ESC_CH
        )
        results['esc_reverse'] = calibrate_step(
            pca, 'esc_reverse', 4000, 
            "後退がしっかり始まる基準となる値を探してください。", ESC_CH
        )

        print("\nすべてのキャリブレーションが完了しました。")
        print("以下の値で params.yaml を更新します:")
        for k, v in results.items():
            print(f"  {k}: {v}")
            
        update_yaml(results)

    except KeyboardInterrupt:
        print("\nキャリブレーションを中断しました。")
    finally:
        print("ハードウェアを安全停止します...")
        pca.channels[STEER_CH].duty_cycle = results.get('steer_center', 4950)
        pca.channels[ESC_CH].duty_cycle = results.get('esc_stop', 5200)
        pca.deinit()

def update_yaml(calibrated_vals):
    if not os.path.exists(PARAMS_FILE):
        print(f"エラー: 設定ファイルが見つかりません: {PARAMS_FILE}")
        return

    with open(PARAMS_FILE, 'r') as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        updated_line = line
        # キーにマッチするか各パラメータをチェック
        for key, val in calibrated_vals.items():
            # 先頭の空白を含めて 'steer_center: 4950' のような行を探す
            pattern = rf"^(\s*{key}:\s*)\d+"
            if re.match(pattern, line):
                updated_line = re.sub(pattern, rf"\g<1>{val}", line)
                break
        new_lines.append(updated_line)

    with open(PARAMS_FILE, 'w') as f:
        f.writelines(new_lines)

    print(f"\n成功: {PARAMS_FILE} を更新しました！")
    print("反映させるには以下のコマンドでビルドしてください：")
    print("cd ros2_ws && colcon build --packages-select f1tenth_rl --symlink-install")

if __name__ == "__main__":
    main()
