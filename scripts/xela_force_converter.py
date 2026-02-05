#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XELA Force Converter Node

Subscribes to xServTopic, converts digital taxel values to forces
using calibration parameters, and publishes for EACH sensor:
  - sensor_<pos>/forces (16x3): calibrated force values per taxel
  - sensor_<pos>/total_force (1x3): sum of all taxel forces

Usage:
    rosrun xela_server_ros xela_force_converter.py

    With sensor-specific calibration:
    rosrun xela_server_ros xela_force_converter.py _sensor_1_calib:=<path_to_json> _sensor_2_calib:=<path_to_json>
"""

import os
import json
import threading
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Trigger, TriggerResponse
from xela_server_ros.msg import SensStream


class XelaForceConverter:
    """Node to convert XELA sensor digital values to calibrated forces."""

    def __init__(self):
        rospy.init_node("xela_force_converter", anonymous=False)

        # Get package path for default calibration
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.package_dir = os.path.dirname(script_dir)

        # Load calibration parameters for each sensor
        self.sensor_calibrations = {}
        self.load_sensor_calibrations()

        # Dynamic publishers for each sensor (created on first message)
        self.sensor_publishers = {}

        # Baseline calibration state
        self.baseline_mode = False  # True when collecting baseline
        self.baseline_duration = rospy.get_param("~baseline_duration", 10.0)
        self.baseline_buffer = {}  # {sensor_id: {"x": [], "y": [], "z": []}}
        self.baseline_values = {}  # {sensor_id: {"x": mean, "y": mean, "z": mean}}
        self.use_baseline = False  # True after baseline is set
        self.baseline_lock = threading.Lock()

        # Services for baseline calibration
        self.start_baseline_srv = rospy.Service(
            "~start_baseline", Trigger, self.handle_start_baseline
        )
        self.clear_baseline_srv = rospy.Service(
            "~clear_baseline", Trigger, self.handle_clear_baseline
        )

        # Subscriber to xela sensor stream
        self.sub = rospy.Subscriber(
            "/xServTopic", SensStream, self.callback, queue_size=10
        )

        rospy.loginfo("XELA Force Converter Node initialized")
        rospy.loginfo("  Publishing topics for all detected sensors")
        rospy.loginfo(f"  Baseline duration: {self.baseline_duration} seconds")
        rospy.loginfo("  Services: ~start_baseline, ~clear_baseline")

    def load_sensor_calibrations(self):
        """Load calibration parameters for each sensor from specified JSON files."""
        # Check for sensor-specific calibration JSON files
        # Parameters: sensor_1_calib, sensor_2_calib, etc.
        for sensor_id in range(1, 10):  # Support up to 9 sensors
            param_name = f"~sensor_{sensor_id}_calib"
            calib_file = rospy.get_param(param_name, None)

            if calib_file:
                # Resolve relative paths
                if not os.path.isabs(calib_file):
                    calib_file = os.path.join(self.package_dir, calib_file)

                if os.path.exists(calib_file):
                    self.sensor_calibrations[sensor_id] = self._load_calib_file(
                        calib_file
                    )
                    rospy.loginfo(
                        f"  Sensor {sensor_id}: Loaded calibration from {calib_file}"
                    )
                else:
                    rospy.logwarn(
                        f"  Sensor {sensor_id}: Calibration file not found: {calib_file}"
                    )


    def _load_calib_file(self, calib_path: str) -> dict:
        """Load calibration parameters from a JSON file."""
        with open(calib_path, "r") as f:
            calib = json.load(f)

        # Extract per-taxel calibration params (from "each" section)
        slope = {
            "x": calib["each"]["x"]["slope"],
            "y": calib["each"]["y"]["slope"],
            "z": calib["each"]["z"]["slope"],
        }
        intercept = {
            "x": calib["each"]["x"]["intercept"],
            "y": calib["each"]["y"]["intercept"],
            "z": calib["each"]["z"]["intercept"],
        }

        rospy.loginfo(
            f"    slope_x: {slope['x']:.6e}, intercept_x: {intercept['x']:.4f}"
        )
        rospy.loginfo(
            f"    slope_y: {slope['y']:.6e}, intercept_y: {intercept['y']:.4f}"
        )
        rospy.loginfo(
            f"    slope_z: {slope['z']:.6e}, intercept_z: {intercept['z']:.4f}"
        )

        return {"slope": slope, "intercept": intercept}

    def handle_start_baseline(self, req):
        """Service handler to start baseline collection."""
        with self.baseline_lock:
            if self.baseline_mode:
                return TriggerResponse(
                    success=False, message="Baseline collection already in progress"
                )

            # Clear previous baseline data
            self.baseline_buffer = {}
            self.baseline_values = {}
            self.use_baseline = False
            self.baseline_mode = True

        rospy.loginfo(
            f"Starting baseline collection for {self.baseline_duration} seconds..."
        )
        rospy.loginfo("  Keep sensors unloaded (no contact with objects)")

        # Schedule baseline finalization
        rospy.Timer(
            rospy.Duration(self.baseline_duration),
            self._finalize_baseline,
            oneshot=True,
        )

        return TriggerResponse(
            success=True,
            message=f"Baseline collection started ({self.baseline_duration}s)",
        )

    def _finalize_baseline(self, event):
        """Finalize baseline after collection period."""
        with self.baseline_lock:
            self.baseline_mode = False

            if not self.baseline_buffer:
                rospy.logwarn("No baseline data collected!")
                return

            # Calculate mean baseline for each sensor
            for sensor_id, data in self.baseline_buffer.items():
                if len(data["x"]) == 0:
                    continue

                self.baseline_values[sensor_id] = {
                    "x": np.mean(data["x"]),
                    "y": np.mean(data["y"]),
                    "z": np.mean(data["z"]),
                }
                rospy.loginfo(
                    f"  Sensor {sensor_id} baseline: "
                    f"x={self.baseline_values[sensor_id]['x']:.2f}, "
                    f"y={self.baseline_values[sensor_id]['y']:.2f}, "
                    f"z={self.baseline_values[sensor_id]['z']:.2f} "
                    f"(from {len(data['x'])} samples)"
                )

            self.use_baseline = True
            self.baseline_buffer = {}  # Clear buffer

        rospy.loginfo("Baseline collection complete. Now using baseline correction.")

    def handle_clear_baseline(self, req):
        """Service handler to clear baseline and revert to intercept-based calibration."""
        with self.baseline_lock:
            self.baseline_mode = False
            self.baseline_buffer = {}
            self.baseline_values = {}
            self.use_baseline = False

        rospy.loginfo("Baseline cleared. Reverted to intercept-based calibration.")
        return TriggerResponse(success=True, message="Baseline cleared")

    def get_calibration(self, sensor_pos: int) -> dict:
        """Get calibration parameters for a specific sensor position."""
        if sensor_pos in self.sensor_calibrations:
            return self.sensor_calibrations[sensor_pos]
        return None

    def get_sensor_publishers(self, sensor_pos: int):
        """Get or create publishers for a specific sensor position."""
        if sensor_pos not in self.sensor_publishers:
            prefix = f"~sensor_{sensor_pos}"
            self.sensor_publishers[sensor_pos] = {
                "forces": rospy.Publisher(
                    f"{prefix}/forces", Float32MultiArray, queue_size=10
                ),
                "total_force": rospy.Publisher(
                    f"{prefix}/total_force", Vector3Stamped, queue_size=10
                ),
            }
            rospy.loginfo(f"Created publishers for sensor_{sensor_pos}")
        return self.sensor_publishers[sensor_pos]

    def callback(self, msg: SensStream):
        """Process incoming sensor data and publish converted forces for all sensors."""

        for sensor in msg.sensors:
            sensor_pos = sensor.sensor_pos
            num_taxels = len(sensor.taxels)

            if num_taxels == 0:
                continue

            # Collect baseline data if in baseline mode
            with self.baseline_lock:
                if self.baseline_mode:
                    if sensor_pos not in self.baseline_buffer:
                        self.baseline_buffer[sensor_pos] = {"x": [], "y": [], "z": []}

                    # Sum raw values across all taxels for this sensor
                    raw_sum_x = sum(taxel.x for taxel in sensor.taxels)
                    raw_sum_y = sum(taxel.y for taxel in sensor.taxels)
                    raw_sum_z = sum(taxel.z for taxel in sensor.taxels)

                    self.baseline_buffer[sensor_pos]["x"].append(raw_sum_x)
                    self.baseline_buffer[sensor_pos]["y"].append(raw_sum_y)
                    self.baseline_buffer[sensor_pos]["z"].append(raw_sum_z)

            # Get publishers for this sensor
            pubs = self.get_sensor_publishers(sensor_pos)

            # Get calibration for this sensor
            calib = self.get_calibration(sensor_pos)

            if calib is None:
                rospy.logwarn_throttle(
                    10, f"No calibration found for sensor {sensor_pos}, skipping..."
                )
                continue

            slope = calib["slope"]
            intercept = calib["intercept"]

            # Check if we should use baseline correction
            use_baseline_for_sensor = (
                self.use_baseline and sensor_pos in self.baseline_values
            )

            # Prepare array for forces
            forces_data = np.zeros((num_taxels, 3), dtype=np.float32)

            if use_baseline_for_sensor:
                # Baseline mode: force = slope * (raw - baseline)
                # Note: baseline is for total (sum), so we distribute proportionally
                # For simplicity, apply to total force after summing raw values
                baseline = self.baseline_values[sensor_pos]

                # Sum raw values
                raw_sum_x = sum(taxel.x for taxel in sensor.taxels)
                raw_sum_y = sum(taxel.y for taxel in sensor.taxels)
                raw_sum_z = sum(taxel.z for taxel in sensor.taxels)

                # Apply baseline correction and slope
                total_force = np.array(
                    [
                        slope["x"] * (raw_sum_x - baseline["x"]),
                        slope["y"] * (raw_sum_y - baseline["y"]),
                        slope["z"] * (raw_sum_z - baseline["z"]),
                    ],
                    dtype=np.float32,
                )

                # For per-taxel forces, use standard calibration (baseline is total-level)
                for i, taxel in enumerate(sensor.taxels):
                    forces_data[i, 0] = slope["x"] * taxel.x + intercept["x"]
                    forces_data[i, 1] = slope["y"] * taxel.y + intercept["y"]
                    forces_data[i, 2] = slope["z"] * taxel.z + intercept["z"]
            else:
                # Standard mode: force = slope * raw + intercept
                for i, taxel in enumerate(sensor.taxels):
                    forces_data[i, 0] = slope["x"] * taxel.x + intercept["x"]
                    forces_data[i, 1] = slope["y"] * taxel.y + intercept["y"]
                    forces_data[i, 2] = slope["z"] * taxel.z + intercept["z"]

                # Calculate total force (sum over all taxels)
                total_force = forces_data.sum(axis=0)

            # Create and publish forces message
            forces_msg = self._create_multiarray(forces_data, "forces")
            pubs["forces"].publish(forces_msg)

            # Create and publish total_force message
            total_msg = Vector3Stamped()
            total_msg.header = Header()
            total_msg.header.stamp = rospy.Time.now()
            total_msg.header.frame_id = f"xela_sensor_{sensor_pos}"
            total_msg.vector.x = float(total_force[0])
            total_msg.vector.y = float(total_force[1])
            total_msg.vector.z = float(total_force[2])
            pubs["total_force"].publish(total_msg)

    def _create_multiarray(self, data: np.ndarray, label: str) -> Float32MultiArray:
        """Create a Float32MultiArray message from numpy array."""
        msg = Float32MultiArray()

        # Set dimensions
        msg.layout.dim = [
            MultiArrayDimension(
                label="taxel", size=data.shape[0], stride=data.shape[0] * data.shape[1]
            ),
            MultiArrayDimension(label="axis", size=data.shape[1], stride=data.shape[1]),
        ]
        msg.layout.data_offset = 0

        # Flatten and set data
        msg.data = data.flatten().tolist()

        return msg

    def run(self):
        """Run the node."""
        rospy.spin()


def main():
    try:
        node = XelaForceConverter()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
