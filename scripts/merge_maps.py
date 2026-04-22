#!/usr/bin/env python3
import os
import yaml
import numpy as np
from PIL import Image
import argparse
import glob

def merge_maps(session_dir, output_name="merged_map"):
    # 1. すべてのYAMLファイルを探す
    yaml_files = glob.glob(os.path.join(session_dir, "**/map_*.yaml"), recursive=True)
    if not yaml_files:
        print(f"No map YAML files found in {session_dir}")
        return

    maps_data = []
    
    # 世界座標の範囲を特定するための変数
    min_world_x = float('inf')
    max_world_x = float('-inf')
    min_world_y = float('inf')
    max_world_y = float('-inf')
    resolution = None

    print(f"Loading {len(yaml_files)} maps...")

    for yf in sorted(yaml_files):
        with open(yf, 'r') as f:
            data = yaml.safe_load(f)
        
        # 解像度のチェック（すべて同じであることを期待）
        res = data['resolution']
        if resolution is None:
            resolution = res
        elif not np.isclose(resolution, res):
            print(f"Warning: Resolution mismatch in {yf}. Expected {resolution}, got {res}")
            continue

        origin = data['origin'] # [x, y, yaw]
        img_path = os.path.join(os.path.dirname(yf), data['image'])
        
        try:
            img = Image.open(img_path).convert('L')
            img_arr = np.array(img)
        except Exception as e:
            print(f"Error loading image {img_path}: {e}")
            continue

        h, w = img_arr.shape
        # ROSの画像座標系：(0,0)は左下
        # 世界座標での範囲
        orig_x, orig_y = origin[0], origin[1]
        width_m = w * resolution
        height_m = h * resolution
        
        map_info = {
            'array': img_arr,
            'origin': origin,
            'w_px': w,
            'h_px': h,
            'min_x': orig_x,
            'max_x': orig_x + width_m,
            'min_y': orig_y,
            'max_y': orig_y + height_m
        }
        maps_data.append(map_info)

        min_world_x = min(min_world_x, map_info['min_x'])
        max_world_x = max(max_world_x, map_info['max_x'])
        min_world_y = min(min_world_y, map_info['min_y'])
        max_world_y = max(max_world_y, map_info['max_y'])

    if not maps_data:
        return

    # キャンバスサイズ計算
    canvas_w = int(np.ceil((max_world_x - min_world_x) / resolution))
    canvas_h = int(np.ceil((max_world_y - min_world_y) / resolution))

    print(f"Canvas size: {canvas_w}x{canvas_h} px")
    print(f"World bounds: X[{min_world_x}, {max_world_x}], Y[{min_world_y}, {max_world_y}]")

    # 初期値は「未知 (205)」
    # ROS map values: 0 (occ), 205 (unknown), 254 (free)
    canvas = np.full((canvas_h, canvas_w), 205, dtype=np.uint8)

    for m in maps_data:
        # このマップの左下隅のキャンバス内ピクセル座標
        offset_x_px = int(round((m['min_x'] - min_world_x) / resolution))
        offset_y_px = int(round((m['min_y'] - min_world_y) / resolution))
        
        # 画像の貼り付け（ROSの画像は上が北だが、numpy/Pillowは上がインデックス0。
        # ROSの保存したPGMは上が北なので、そのまま貼り付ければ良いが、y軸の反転に注意が必要な場合がある。
        # ただし、ROSのmap_saverが出力するPGMは、そのまま表示した時に上が北。
        # したがって、y=0(下)を画像の下に配置する必要がある。
        
        # キャンバス内での領域
        # Numpy座標 (row, col)
        # ROS origin (min_x, min_y) は左下ピクセル。
        # Numpyでキャンバスを表示したとき、row=0が「上」になるようにしたい。
        # y座標が大きいほど、rowインデックスは小さくなる。
        
        # y_pixel_in_canvas = (max_world_y - world_y) / resolution
        # 左上端の座標
        start_row = canvas_h - (offset_y_px + m['h_px'])
        end_row = start_row + m['h_px']
        start_col = offset_x_px
        end_col = start_col + m['w_px']

        # ターゲット領域を切り出し、各ピクセルを合成
        # 合成ルール:
        # 1. 元が未知(205)なら、新しい値を採用。
        # 2. 新しい値が障害物(0)なら、上書き。
        # 3. 新しい値が空き(254)で、元が障害物(0)でなければ、上書き。
        
        target = canvas[start_row:end_row, start_col:end_col]
        new_val = m['array']
        
        # 障害物(0)で上書き
        mask_occ = (new_val == 0)
        target[mask_occ] = 0
        
        # 空きスペース(254)で上書き（ただし障害物でない場所のみ）
        mask_free = (new_val == 254) & (target != 0)
        target[mask_free] = 254

    # 保存
    output_pgm = os.path.join(session_dir, f"{output_name}.pgm")
    output_yaml = os.path.join(session_dir, f"{output_name}.yaml")
    
    out_img = Image.fromarray(canvas)
    out_img.save(output_pgm)
    
    # YAML作成
    yaml_content = {
        'image': f"{output_name}.pgm",
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [float(min_world_x), float(min_world_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    
    with open(output_yaml, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)

    print(f"Merged map saved to: {output_pgm}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge ROS occupancy grid maps.")
    parser.add_argument("dir", help="Directory containing map folders")
    parser.add_argument("--output", default="merged_map", help="Output filename base")
    
    args = parser.parse_args()
    merge_maps(args.dir, args.output)
