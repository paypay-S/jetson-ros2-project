#!/usr/bin/env python3
import os
import re

def main():
    print("=== F1TENTH Normalization Calibration Tool ===")
    print("This tool updates the normalization constants in params.yaml.")
    print("Please input the statistics from your training (config.py).")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    params_path = os.path.join(project_root, "ros2_ws/src/f1tenth_rl/config/params.yaml")
    
    if not os.path.exists(params_path):
        print(f"Error: {params_path} not found.")
        return

    # Current defaults or placeholders
    params = {
        "obs_lidar_mean": "4.869",
        "obs_lidar_std": "3.577",
        "obs_lidar_residual_mean": "-0.008",
        "obs_lidar_residual_std": "0.084",
        "obs_speed_mean": "0.574",
        "obs_speed_std": "0.096",
        "obs_steer_mean": "-0.010",
        "obs_steer_std": "0.122"
    }

    # Interactive input
    for key in params.keys():
        val = input(f"Enter value for {key} (default {params[key]}): ").strip()
        if val:
            params[key] = val

    # Read and update YAML (using regex to preserve comments)
    with open(params_path, 'r') as f:
        content = f.read()

    for key, val in params.items():
        pattern = rf"({key}:\s*)[\d\.-]+"
        content = re.sub(pattern, rf"\g<1>{val}", content)

    # Write back
    with open(params_path, 'w') as f:
        f.write(content)

    print("-" * 30)
    print(f"Successfully updated {params_path}")
    print("Please rebuild or restart your ROS 2 nodes.")

if __name__ == "__main__":
    main()
