#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XELA Calibration Client - Record sensor data to CSV files

Usage:
    rosrun xela_server_ros xela_calibration_client
    rosrun xela_server_ros xela_calibration_client --duration 60 --fps 10
"""

import os
import csv
import signal
import time
from datetime import datetime
from typing import Dict, List, Optional
import math

import rospy
import argparse

from xela_server_ros.msg import SensStream, SensorClusters


class XelaCalibrationClient:
    """Record XELA sensor data to CSV files"""

    def __init__(
        self, output_dir: str, fps: int = 50, duration: Optional[float] = None
    ):
        self.output_dir = output_dir
        self.duration = duration
        self.fps = fps
        self.start_time = None
        self.running = True

        # Data storage per sensor
        self.sensor_data: Dict[int, dict] = {}
        self.sensor_writers: Dict[int, csv.writer] = {}
        self.sensor_files: Dict[int, object] = {}
        self.sensor_models: Dict[int, str] = {}

        # Cluster data storage
        self.cluster_data: Dict[int, List[List[float]]] = {}

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # ROS subscribers
        rospy.init_node("xela_calibration_client", anonymous=True)
        self.sub_sensor = rospy.Subscriber(
            "xServTopic", SensStream, self.sensor_callback
        )
        self.sub_cluster = rospy.Subscriber(
            "xServClusters", SensorClusters, self.cluster_callback
        )

        rospy.loginfo("XELA Calibration Client started")
        rospy.loginfo(f"Output directory: {output_dir}")
        rospy.loginfo(f"Recording at {self.fps} FPS")

    def generate_header(self, num_taxels: int = 16) -> List[str]:
        """
        Generate CSV header based on taxel count.

        Header format:
        time,tax00x,...,tax33x,tax00y,...,tax33y,tax00z,...,tax33z,
        frc00x,...,frc33x,frc00y,...,frc33y,frc00z,...,frc33z,
        tax_x,tax_y,tax_z,tax,frc_x,frc_y,frc_z,frc,
        cluster_x,cluster_y,cluster_frc
        """
        header = ["time"]

        rows = int(math.sqrt(num_taxels))
        cols = rows

        # Taxel raw values: all x first, then all y, then all z
        for axis in ["x", "y", "z"]:
            for r in range(rows):
                for c in range(cols):
                    header.append(f"tax{r}{c}{axis}")

        # Force values (calibrated, Newton): all x first, then all y, then all z
        for axis in ["x", "y", "z"]:
            for r in range(rows):
                for c in range(cols):
                    header.append(f"frc{r}{c}{axis}")

        # Summed taxel values and norm
        header.extend(["tax_x", "tax_y", "tax_z", "tax"])

        # Summed force values and norm
        header.extend(["frc_x", "frc_y", "frc_z", "frc"])

        # Cluster data from server
        header.extend(["cluster_x", "cluster_y", "cluster_frc"])

        return header

    def get_csv_writer(self, sensor_id: int, model: str) -> csv.writer:
        """Get or create CSV writer for a sensor"""
        if sensor_id not in self.sensor_writers:
            # Use JST (UTC+9) for filename
            from datetime import timedelta

            timestamp = (datetime.utcnow() + timedelta(hours=9)).strftime(
                "%Y-%m-%d_%H:%M:%S"
            )
            filename = f"{timestamp}_{model}_{sensor_id}.csv"
            filepath = os.path.join(self.output_dir, filename)

            f = open(filepath, "w", newline="")
            writer = csv.writer(f)
            writer.writerow(self.generate_header())

            self.sensor_files[sensor_id] = f
            self.sensor_writers[sensor_id] = writer
            self.sensor_models[sensor_id] = model

            rospy.loginfo(f"Created CSV file: {filepath}")

        return self.sensor_writers[sensor_id]

    def sensor_callback(self, msg: SensStream):
        """Handle sensor data from xServTopic"""
        for sensor in msg.sensors:
            sensor_id = sensor.sensor_pos
            model = sensor.model

            # Store latest data
            self.sensor_data[sensor_id] = {
                "time": sensor.time,
                "model": model,
                "taxels": [(t.x, t.y, t.z) for t in sensor.taxels],
                "forces": [(f.x, f.y, f.z) for f in sensor.forces],
            }

    def cluster_callback(self, msg: SensorClusters):
        """Handle cluster data from xServClusters"""
        sensor_id = msg.sensor_id
        clusters = []
        for cluster in msg.clusters:
            clusters.append([cluster.x, cluster.y, cluster.force])
        self.cluster_data[sensor_id] = clusters

    def compute_sums_and_norm(self, data: List[tuple]) -> tuple:
        """
        Compute sum of each axis component and the L2 norm.

        Args:
            data: List of (x, y, z) tuples

        Returns:
            (sum_x, sum_y, sum_z, norm): Axis sums and L2 norm of the sum vector
        """
        if not data:
            return (0.0, 0.0, 0.0, 0.0)

        sum_x = sum(d[0] for d in data)
        sum_y = sum(d[1] for d in data)
        sum_z = sum(d[2] for d in data)
        norm = math.sqrt(sum_x**2 + sum_y**2 + sum_z**2)

        return (sum_x, sum_y, sum_z, norm)

    def write_data(self):
        """Write current sensor data to CSV files"""
        for sensor_id, data in self.sensor_data.items():
            model = data.get("model", "unknown")
            writer = self.get_csv_writer(sensor_id, model)

            taxels = data.get("taxels", [])
            forces = data.get("forces", [])
            record_time = data.get("time", time.time())

            # Build row
            row = [record_time]

            # Taxel raw values: all x, then all y, then all z
            for axis_idx in range(3):  # x=0, y=1, z=2
                for t in taxels:
                    row.append(t[axis_idx] if len(t) > axis_idx else 0)
                # Pad if less than 16 taxels
                while len(row) < 1 + (axis_idx + 1) * 16:
                    row.append(0)

            # Force values: all x, then all y, then all z
            for axis_idx in range(3):
                for f in forces:
                    row.append(f[axis_idx] if len(f) > axis_idx else 0.0)
                # Pad if less than 16 forces
                while len(row) < 1 + 48 + (axis_idx + 1) * 16:
                    row.append(0.0)

            # Summed taxel values and norm
            tax_x, tax_y, tax_z, tax_norm = self.compute_sums_and_norm(taxels)
            row.extend([tax_x, tax_y, tax_z, tax_norm])

            # Summed force values and norm
            frc_x, frc_y, frc_z, frc_norm = self.compute_sums_and_norm(forces)
            row.extend([frc_x, frc_y, frc_z, frc_norm])

            # Cluster data from server
            clusters = self.cluster_data.get(sensor_id, [])
            if clusters:
                cluster = clusters[0]
                row.extend([cluster[0], cluster[1], cluster[2]])
            else:
                row.extend([0.0, 0.0, 0.0])

            writer.writerow(row)

    def run(self):
        """Main loop at specified FPS"""
        self.start_time = time.time()
        rate = rospy.Rate(self.fps)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        rospy.loginfo("Recording started. Press Ctrl+C to stop.")

        while self.running and not rospy.is_shutdown():
            if self.sensor_data:
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
        """Close all CSV files"""
        for f in self.sensor_files.values():
            f.close()

        elapsed = time.time() - self.start_time if self.start_time else 0
        rospy.loginfo(f"Recording stopped. Duration: {elapsed:.1f}s")
        rospy.loginfo(f"Files saved to: {self.output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="XELA Calibration Client - Record sensor data to CSV"
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

    client = XelaCalibrationClient(output_dir, args.fps, args.duration)
    client.run()


if __name__ == "__main__":
    main()
