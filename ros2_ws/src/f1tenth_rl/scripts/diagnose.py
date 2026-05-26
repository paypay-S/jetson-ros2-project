#!/usr/bin/env python3
"""
diagnose.py - F1TENTH 自律走行システム 総合診断スクリプト

使い方:
  # 別ターミナルで先にシステム（simulation or 実機）を起動してから実行
  python3 src/f1tenth_rl/scripts/diagnose.py [racing_line_csv_path]
"""

import sys
import time
import math
import threading
import csv
import os

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import rclpy.duration
import numpy as np


class DiagnosticNode(Node):
    def __init__(self, racing_line_path=''):
        super().__init__('f1tenth_diagnostic')

        # 診断データ収集
        self.drive_msgs = []
        self.odom_msgs = []
        self.scan_received = False
        self.tf_poses = []
        self.start_time = time.time()

        # レーシングライン読み込み
        self.waypoints = []
        if racing_line_path and os.path.exists(racing_line_path):
            with open(racing_line_path) as f:
                for row in csv.reader(f):
                    try:
                        self.waypoints.append((float(row[0]), float(row[1])))
                    except ValueError:
                        continue
            self.get_logger().info(f"レーシングライン読み込み完了: {len(self.waypoints)} ウェイポイント")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # サブスクライバ
        self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        # 30秒後に診断レポートを出力
        self.create_timer(30.0, self._report)
        self.create_timer(1.0, self._check_tf)

        print("\n" + "="*60)
        print("  F1TENTH 自律走行システム 総合診断")
        print("  30秒間データを収集します...")
        print("="*60 + "\n")

    def _drive_cb(self, msg):
        self.drive_msgs.append({
            'time': time.time() - self.start_time,
            'steer': msg.drive.steering_angle,
            'speed': msg.drive.speed
        })

    def _odom_cb(self, msg):
        self.odom_msgs.append({
            'time': time.time() - self.start_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
        })

    def _scan_cb(self, msg):
        self.scan_received = True

    def _check_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            self.tf_poses.append({
                'time': time.time() - self.start_time,
                'x': t.transform.translation.x,
                'y': t.transform.translation.y
            })
        except Exception:
            pass

    def _nearest_waypoint_dist(self, x, y):
        if not self.waypoints:
            return None
        dists = [math.hypot(x - wx, y - wy) for wx, wy in self.waypoints]
        return min(dists)

    def _report(self):
        print("\n" + "="*60)
        print("  📊 診断レポート")
        print("="*60)

        # 1. LiDAR
        status = "✅ 受信中" if self.scan_received else "❌ 未受信"
        print(f"\n【1】LiDAR (/scan): {status}")

        # 2. /drive コマンド
        print(f"\n【2】制御コマンド (/drive):")
        if not self.drive_msgs:
            print("  ❌ コマンドが全く届いていません！rl_driverが動いていない可能性があります。")
        else:
            steers = [d['steer'] for d in self.drive_msgs]
            speeds = [d['speed'] for d in self.drive_msgs]
            hz = len(self.drive_msgs) / 30.0
            print(f"  ✅ 受信数: {len(self.drive_msgs)}件 ({hz:.1f} Hz)")
            print(f"  舵角: min={math.degrees(min(steers)):+.1f}°, max={math.degrees(max(steers)):+.1f}°, avg={math.degrees(sum(steers)/len(steers)):+.1f}°")
            print(f"  速度: min={min(speeds):.2f} m/s, max={max(speeds):.2f} m/s, avg={sum(speeds)/len(speeds):.2f} m/s")

            # 舵角が常に一方向に偏っていないかチェック
            avg_steer_deg = math.degrees(sum(steers) / len(steers))
            if abs(avg_steer_deg) > 10.0:
                print(f"  ⚠️  警告: 平均舵角が {avg_steer_deg:+.1f}° に偏っています（ステア中立ズレの可能性）")
            else:
                print(f"  ✅ 舵角バランス良好（平均 {avg_steer_deg:+.1f}°）")

        # 3. オドメトリ / 位置変化
        print(f"\n【3】自己位置 (/odom):")
        if not self.odom_msgs:
            print("  ❌ オドメトリが届いていません")
        else:
            xs = [d['x'] for d in self.odom_msgs]
            ys = [d['y'] for d in self.odom_msgs]
            travel = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]) for i in range(1, len(xs)))
            print(f"  ✅ 受信数: {len(self.odom_msgs)}件")
            print(f"  走行距離(積算): {travel:.2f} m")
            print(f"  X範囲: {min(xs):.2f} ～ {max(xs):.2f} m")
            print(f"  Y範囲: {min(ys):.2f} ～ {max(ys):.2f} m")

            if travel < 0.1:
                print("  ⚠️  警告: 30秒間でほぼ移動していません！車両が動いていない可能性があります。")
            else:
                print(f"  ✅ 車両は正常に移動しています")

            # レーシングラインとの距離
            if self.waypoints:
                dists = [self._nearest_waypoint_dist(x, y) for x, y in zip(xs, ys)]
                avg_dist = sum(dists) / len(dists)
                print(f"\n  📍 レーシングラインとの平均距離: {avg_dist:.3f} m")
                if avg_dist < 1.0:
                    print(f"  ✅ 優秀！ラインに非常に近く追従しています")
                elif avg_dist < 2.0:
                    print(f"  ⚠️  ラインからやや離れています（チューニング推奨）")
                else:
                    print(f"  ❌ ラインから大きく逸脱しています！自己位置推定かPure Pursuitを確認してください")

        # 4. SLAM TF
        print(f"\n【4】SLAM 自己位置 (map→base_link TF):")
        if not self.tf_poses:
            print("  ⚠️  SLAMのTFが取得できていません（Cartographer未起動 → Dead Reckoningで動作中）")
        else:
            print(f"  ✅ SLAM TF取得成功！ {len(self.tf_poses)}件")
            last = self.tf_poses[-1]
            print(f"  最新位置: x={last['x']:.3f}, y={last['y']:.3f}")

        # 5. 総合判定
        print("\n" + "="*60)
        ok = self.scan_received and len(self.drive_msgs) > 10
        if ok and self.odom_msgs:
            xs = [d['x'] for d in self.odom_msgs]
            ys = [d['y'] for d in self.odom_msgs]
            travel = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]) for i in range(1, len(xs)))
            ok = ok and travel > 0.1
        if ok:
            print("  🎉 総合判定: ✅ システムは正常に動作しています！")
        else:
            print("  ⚠️  総合判定: ❌ 一部のコンポーネントに問題があります（上記を確認してください）")
        print("="*60 + "\n")

        rclpy.shutdown()


def main():
    racing_line = sys.argv[1] if len(sys.argv) > 1 else ''
    rclpy.init()
    node = DiagnosticNode(racing_line)
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
