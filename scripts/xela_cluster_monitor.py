#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XELA Cluster Monitor - WebSocket経由でクラスタリングデータを監視するスクリプト

Usage:
    rosrun xela_server_ros xela_cluster_monitor
    rosrun xela_server_ros xela_cluster_monitor --ip 192.168.0.21 --port 5000
"""

import argparse
import json
import signal
import sys
import time

import websocket


class ClusterMonitor:
    """XELAセンサーのクラスタリングデータを監視するクラス"""

    def __init__(self, ip: str, port: int, verbose: bool = False):
        self.ip = ip
        self.port = port
        self.verbose = verbose
        self.start_time = time.time()
        self.message_count = 0
        self.cluster_count = 0
        self.running = True
        self.ws = None

    def on_message(self, ws, message):
        """WebSocketメッセージを処理"""
        self.message_count += 1
        elapsed = time.time() - self.start_time

        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return

        # 各センサーのextradotsを確認
        for sensor_id in ["1", "2"]:
            if sensor_id not in data:
                continue

            sensor_data = data[sensor_id]
            extradots = sensor_data.get("extradots", [])

            if extradots:
                self.cluster_count += 1
                print(f"\n[{elapsed:.2f}s] === Sensor {sensor_id} クラスター検出 ===")
                for i, cluster in enumerate(extradots):
                    if len(cluster) >= 3:
                        x = float(cluster[0])
                        y = float(cluster[1])
                        force = float(cluster[2])
                        print(f"  クラスター{i+1}: X={x:.2f}, Y={y:.2f}, 力={force:.2f}")
                    else:
                        print(f"  クラスター{i+1}: {cluster}")

        # 詳細モードの場合、special フィールドも表示
        if self.verbose and self.message_count % 100 == 0:
            print(f"\n[{elapsed:.1f}s] メッセージ数: {self.message_count}, クラスター検出数: {self.cluster_count}")

    def on_error(self, ws, error):
        """エラーハンドラ"""
        print(f"WebSocketエラー: {error}", file=sys.stderr)

    def on_close(self, ws, close_status_code, close_msg):
        """接続終了ハンドラ"""
        print(f"\nWebSocket接続終了 (status: {close_status_code})")

    def on_open(self, ws):
        """接続開始ハンドラ"""
        print(f"WebSocket接続成功: ws://{self.ip}:{self.port}")
        print("センサーを押すとクラスタリングデータが表示されます...")
        print("終了するには Ctrl+C を押してください\n")

    def signal_handler(self, signum, frame):
        """シグナルハンドラ"""
        print(f"\n\n=== 統計 ===")
        print(f"総メッセージ数: {self.message_count}")
        print(f"クラスター検出数: {self.cluster_count}")
        print(f"実行時間: {time.time() - self.start_time:.1f}秒")
        self.running = False
        if self.ws:
            self.ws.close()
        sys.exit(0)

    def run(self):
        """監視を開始"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        url = f"ws://{self.ip}:{self.port}"
        print(f"XELAセンサーに接続中: {url}")

        self.ws = websocket.WebSocketApp(
            url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
            on_open=self.on_open,
        )

        while self.running:
            try:
                self.ws.run_forever()
            except Exception as e:
                print(f"接続エラー: {e}", file=sys.stderr)
                if self.running:
                    print("5秒後に再接続します...")
                    time.sleep(5)


def get_local_ip():
    """ローカルIPアドレスを取得"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("1.2.3.4", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def main():
    parser = argparse.ArgumentParser(
        description="XELA Cluster Monitor - クラスタリングデータを監視",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-i", "--ip",
        default=None,
        help="XELA Serverのアドレス (デフォルト: 自動検出)",
    )
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=5000,
        help="XELA Serverのポート (デフォルト: 5000)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="詳細モード",
    )

    args = parser.parse_args()

    # IPアドレスの自動検出
    if args.ip is None:
        args.ip = get_local_ip()
        print(f"IPアドレス自動検出: {args.ip}")

    monitor = ClusterMonitor(
        ip=args.ip,
        port=args.port,
        verbose=args.verbose,
    )
    monitor.run()


if __name__ == "__main__":
    main()
