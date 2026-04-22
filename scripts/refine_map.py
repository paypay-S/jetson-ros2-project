#!/usr/bin/env python3
import os
import yaml
import numpy as np
from PIL import Image
import scipy.ndimage as ndimage
import argparse

def refine_map(yaml_path, output_suffix="_refined", iterations=1, denoise=True):
    print(f"Refining map: {yaml_path} (iterations={iterations}, denoise={denoise})")
    
    # 1. Load YAML
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    img_dir = os.path.dirname(yaml_path)
    img_path = os.path.join(img_dir, config['image'])
    resolution = config['resolution']
    origin = config['origin']  # [x, y, yaw]

    # 2. Load Image
    img = Image.open(img_path).convert('L')
    img_arr = np.array(img)
    rows, cols = img_arr.shape

    # Masks
    occupied_mask = (img_arr == 0)
    free_mask = (img_arr == 254)

    print("Processing image...")

    # 3. Structural element (8-connectivity for better wall preservation)
    struct = ndimage.generate_binary_structure(2, 2)
    
    # 4. Fill gaps in walls (Closing)
    # これを強くするほど隙間が繋がる
    refined_occupied = ndimage.binary_closing(occupied_mask, structure=struct, iterations=iterations)
    
    # 5. Denoise (Opening)
    # これを行うと小さなノイズが消えるが、細い壁も消える可能性がある
    if denoise:
        refined_occupied = ndimage.binary_opening(refined_occupied, structure=struct, iterations=1)

    # 6. Build New Image
    new_arr = np.full((rows, cols), 205, dtype=np.uint8)
    new_arr[free_mask] = 254  # Keep free space
    new_arr[refined_occupied] = 0

    # 7. Automatic Trimming
    data_mask = (new_arr != 205)
    if not np.any(data_mask):
        print("Error: No data found in map after refinement.")
        return

    coords = np.argwhere(data_mask)
    y_min, x_min = coords.min(axis=0)
    y_max, x_max = coords.max(axis=0)

    # Padding
    pad = 5
    y_min = max(0, y_min - pad)
    x_min = max(0, x_min - pad)
    y_max = min(rows - 1, y_max + pad)
    x_max = min(cols - 1, x_max + pad)

    # Trimming
    trimmed_arr = new_arr[y_min:y_max+1, x_min:x_max+1]
    trimmed_h, trimmed_w = trimmed_arr.shape
    
    # 8. Origin recalculation
    dx_px = x_min
    dy_px = (rows - 1) - y_max
    
    new_origin_x = origin[0] + (dx_px * resolution)
    new_origin_y = origin[1] + (dy_px * resolution)
    new_origin = [float(new_origin_x), float(new_origin_y), origin[2]]

    # 9. Save
    base_name = os.path.splitext(os.path.basename(yaml_path))[0]
    out_img_name = f"{base_name}{output_suffix}.pgm"
    out_yaml_name = f"{base_name}{output_suffix}.yaml"
    
    out_img_path = os.path.join(img_dir, out_img_name)
    out_yaml_path = os.path.join(img_dir, out_yaml_name)

    out_img = Image.fromarray(trimmed_arr)
    out_img.save(out_img_path)

    new_config = config.copy()
    new_config['image'] = out_img_name
    new_config['origin'] = new_origin
    
    with open(out_yaml_path, 'w') as f:
        yaml.dump(new_config, f, default_flow_style=False)

    print(f"Refinement complete!")
    print(f"Saved to: {out_img_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean and trim ROS occupancy grid maps.")
    parser.add_argument("yaml", help="Path to the map YAML file")
    parser.add_argument("--suffix", default="_refined", help="Suffix for the output file")
    parser.add_argument("--iterations", type=int, default=1, help="Strength of gap filling (closing)")
    parser.add_argument("--no-denoise", action="store_true", help="Skip noise removal (opening)")
    
    args = parser.parse_args()
    refine_map(args.yaml, args.suffix, args.iterations, not args.no_denoise)
