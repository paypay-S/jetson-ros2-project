import os
import sys
import time
import numpy as np
import onnxruntime as ort
import torch
from stable_baselines3 import PPO
import argparse

def measure_latency(model_path, num_runs=100):
    print(f"--- Benchmark: {model_path} ---")
    
    # 1. Load Model
    if model_path.endswith('.onnx'):
        session = ort.InferenceSession(model_path)
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        # Input shape typically [batch, dim] or [None, dim]
        dim = input_shape[1]
        
        def run_inference(obs):
            return session.run(None, {input_name: obs})
            
    else:
        model = PPO.load(model_path, device="cpu")
        dim = model.observation_space.shape[0]
        
        def run_inference(obs):
            return model.predict(obs, deterministic=True)

    print(f"Input dimension: {dim}")
    
    # 2. Warm up
    dummy_input = np.random.randn(1, dim).astype(np.float32)
    for _ in range(10):
        run_inference(dummy_input)
        
    # 3. Measurement
    latencies = []
    for _ in range(num_runs):
        start = time.perf_counter()
        run_inference(dummy_input)
        end = time.perf_counter()
        latencies.append((end - start) * 1000) # ms
        
    avg_latency = np.mean(latencies)
    std_latency = np.std(latencies)
    min_latency = np.min(latencies)
    max_latency = np.max(latencies)
    
    print(f"Average Latency: {avg_latency:.4f} ms")
    print(f"Std Deviation  : {std_latency:.4f} ms")
    print(f"Min Latency    : {min_latency:.4f} ms")
    print(f"Max Latency    : {max_latency:.4f} ms")
    print(f"Frequency      : {1000/avg_latency:.2f} Hz")
    print("-" * 30)
    return avg_latency

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sb3", type=str, help="Path to SB3 .zip model")
    parser.add_argument("--onnx", type=str, help="Path to .onnx model")
    parser.add_argument("--runs", type=int, default=100)
    args = parser.parse_args()
    
    if args.sb3:
        measure_latency(args.sb3, args.runs)
    if args.onnx:
        measure_latency(args.onnx, args.runs)
