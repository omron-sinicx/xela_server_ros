#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XELA Friction Logger Node

Subscribes to force and friction topics from 2 sensors and logs data to CSV.

Topics subscribed:
  - /xela_force_converter/sensor_1/total_force (geometry_msgs/Vector3Stamped)
  - /xela_force_converter/sensor_2/total_force (geometry_msgs/Vector3Stamped)
  - /xela_friction/sensor_1/friction (std_msgs/Float32)
  - /xela_friction/sensor_2/friction (std_msgs/Float32)

CSV Header:
  timestamp,s1_fx,s1_fy,s1_fz,s1_ft,s1_fn,s1_mu,s2_fx,s2_fy,s2_fz,s2_ft,s2_fn,s2_mu

Usage:
    rosrun xela_server_ros xela_friction_logger.py
    rosrun xela_server_ros xela_friction_logger.py --duration 60 --fps 30
"""

import os
import csv
import signal
import time
import math
from datetime import datetime, timedelta
from typing import Optional

import rospy
import argparse

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped


class XelaFrictionLogger:
    """Log force and friction data from 2 XELA sensors to CSV"""

    def __init__(
        self, output_dir: str, fps: int = 50, duration: Optional[float] = None
    ):
        self.output_dir = output_dir
        self.duration = duration
        self.fps = fps
        self.start_time = None
        self.running = True

        # Latest data from sensors
        self.sensor_data = {
            1: {"fx": 0.0, "fy": 0.0, "fz": 0.0, "mu": 0.0},
            2: {"fx": 0.0, "fy": 0.0, "fz": 0.0, "mu": 0.0},
        }

        # CSV file
        self.csv_file = None
        self.csv_writer = None

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # ROS init
        rospy.init_node("xela_friction_logger", anonymous=False)

        # Subscribe to force topics
        self.sub_force_1 = rospy.Subscriber(
            "/xela_force_converter/sensor_1/total_force",
            Vector3Stamped,
            self.force_callback_1,
        )
        self.sub_force_2 = rospy.Subscriber(
            "/xela_force_converter/sensor_2/total_force",
            Vector3Stamped,
            self.force_callback_2,
        )

        # Subscribe to friction topics
        self.sub_friction_1 = rospy.Subscriber(
            "/xela_friction/sensor_1/friction",
            Float32,
            self.friction_callback_1,
        )
        self.sub_friction_2 = rospy.Subscriber(
            "/xela_friction/sensor_2/friction",
            Float32,
            self.friction_callback_2,
        )

        rospy.loginfo("XELA Friction Logger started")
        rospy.loginfo(f"Output directory: {output_dir}")
        rospy.loginfo(f"Recording at {self.fps} FPS")

    def force_callback_1(self, msg: Vector3Stamped):
        self.sensor_data[1]["fx"] = msg.vector.x
        self.sensor_data[1]["fy"] = msg.vector.y
        self.sensor_data[1]["fz"] = msg.vector.z

    def force_callback_2(self, msg: Vector3Stamped):
        self.sensor_data[2]["fx"] = msg.vector.x
        self.sensor_data[2]["fy"] = msg.vector.y
        self.sensor_data[2]["fz"] = msg.vector.z

    def friction_callback_1(self, msg: Float32):
        self.sensor_data[1]["mu"] = msg.data

    def friction_callback_2(self, msg: Float32):
        self.sensor_data[2]["mu"] = msg.data

    def create_csv_file(self):
        """Create CSV file with header"""
        # Use JST (UTC+9) for filename
        timestamp = (datetime.utcnow() + timedelta(hours=9)).strftime(
            "%Y-%m-%d_%H:%M:%S"
        )
        filename = f"{timestamp}_friction_log.csv"
        filepath = os.path.join(self.output_dir, filename)

        self.csv_file = open(filepath, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        header = [
            "timestamp",
            "s1_fx",
            "s1_fy",
            "s1_fz",
            "s1_ft",
            "s1_fn",
            "s1_mu",
            "s2_fx",
            "s2_fy",
            "s2_fz",
            "s2_ft",
            "s2_fn",
            "s2_mu",
        ]
        self.csv_writer.writerow(header)

        rospy.loginfo(f"Created CSV file: {filepath}")

    def write_data(self):
        """Write current data to CSV"""
        if self.csv_writer is None:
            return

        now = rospy.Time.now().to_sec()

        row = [now]
        for sensor_id in [1, 2]:
            data = self.sensor_data[sensor_id]
            fx = data["fx"]
            fy = data["fy"]
            fz = data["fz"]
            mu = data["mu"]

            # Calculate tangential and normal force
            ft = math.sqrt(fx**2 + fy**2)
            fn = abs(fz)

            row.extend([fx, fy, fz, ft, fn, mu])

        self.csv_writer.writerow(row)

    def run(self):
        """Main loop at specified FPS"""
        self.create_csv_file()
        self.start_time = time.time()
        rate = rospy.Rate(self.fps)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        rospy.loginfo("Recording started. Press Ctrl+C to stop.")

        while self.running and not rospy.is_shutdown():
            self.write_data()

            # Check duration limit
            if self.duration is not None:
                elapsed = time.time() - self.start_time
                if elapsed >= self.duration:
                    rospy.loginfo(f"Duration limit reached ({self.duration}s)")
                    break

            rate.sleep()

        self.cleanup()

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        rospy.loginfo("\nShutting down...")
        self.running = False

    def cleanup(self):
        """Close CSV file"""
        if self.csv_file:
            self.csv_file.close()

        elapsed = time.time() - self.start_time if self.start_time else 0
        rospy.loginfo(f"Recording stopped. Duration: {elapsed:.1f}s")
        rospy.loginfo(f"Files saved to: {self.output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="XELA Friction Logger - Record force and friction data to CSV"
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output directory (default: xela_server_ros/results/)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=None,
        help="Recording duration in seconds (default: unlimited)",
    )
    parser.add_argument(
        "--fps", type=int, default=50, help="Recording frame rate in Hz (default: 50)"
    )

    # Parse known args to handle ROS remapping args
    args, _ = parser.parse_known_args()

    # Default output directory
    if args.output is None:
        import rospkg

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("xela_server_ros")
        output_dir = os.path.join(pkg_path, "results")
    else:
        output_dir = args.output

    logger = XelaFrictionLogger(output_dir, args.fps, args.duration)
    logger.run()


if __name__ == "__main__":
    main()
