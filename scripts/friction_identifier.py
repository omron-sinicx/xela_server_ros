#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Friction Identifier Node

Records tangential and normal forces from XELA sensors, displays interactive plots,
and calculates friction coefficient using Orthogonal Distance Regression (ODR/TLS).

Usage:
    roslaunch xela_server_ros friction_identifier.launch
"""

import os
import sys
import threading
import datetime
import csv
import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

# For keyboard input detection
import select
import termios
import tty


class FrictionIdentifier:
    """Node to identify friction coefficient from force data."""

    def __init__(self):
        rospy.init_node("friction_identifier", anonymous=False)

        # Parameters
        self.sensor_ids = rospy.get_param("~sensor_ids", [1, 2])
        if isinstance(self.sensor_ids, str):
            # Parse string like "[1, 2]" to list
            self.sensor_ids = eval(self.sensor_ids)

        # ODR intercept parameter: None means free intercept, 0 means forced through origin
        intercept_param = rospy.get_param("~odr_intercept", None)
        if intercept_param == "None" or intercept_param == "":
            self.odr_intercept = None
        else:
            self.odr_intercept = float(intercept_param)

        # Results directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_dir = os.path.dirname(script_dir)
        self.results_base_dir = os.path.join(
            package_dir, "results", "friction_identifier"
        )
        os.makedirs(self.results_base_dir, exist_ok=True)

        # Data storage per sensor
        self.sensor_data = {
            sid: {"x": [], "y": [], "z": [], "timestamps": []}
            for sid in self.sensor_ids
        }

        # Subscribers
        self.subscribers = {}
        for sid in self.sensor_ids:
            topic = f"/xela_force_converter/sensor_{sid}/total_force"
            self.subscribers[sid] = rospy.Subscriber(
                topic,
                Vector3Stamped,
                self.force_callback,
                callback_args=sid,
                queue_size=100,
            )

        # Publishers for friction coefficient
        self.friction_publishers = {}
        for sid in self.sensor_ids:
            topic = f"~friction_coefficient/sensor_{sid}"
            self.friction_publishers[sid] = rospy.Publisher(
                topic, Float32, queue_size=1, latch=True
            )

        # Publisher for aggregated friction coefficients (all sensors)
        self.friction_array_pub = rospy.Publisher(
            "/friction_identifier/friction_coefficients",
            Float32MultiArray,
            queue_size=1,
            latch=True,
        )

        # Recording state
        self.recording = False
        self.lock = threading.Lock()

        rospy.loginfo("Friction Identifier Node initialized")
        rospy.loginfo(f"  Sensor IDs: {self.sensor_ids}")
        rospy.loginfo(f"  ODR Intercept: {self.odr_intercept}")
        rospy.loginfo(f"  Results directory: {self.results_base_dir}")

    def force_callback(self, msg: Vector3Stamped, sensor_id: int):
        """Store force data when recording."""
        if not self.recording:
            return

        with self.lock:
            self.sensor_data[sensor_id]["x"].append(msg.vector.x)
            self.sensor_data[sensor_id]["y"].append(msg.vector.y)
            self.sensor_data[sensor_id]["z"].append(msg.vector.z)
            self.sensor_data[sensor_id]["timestamps"].append(msg.header.stamp.to_sec())

    def clear_data(self):
        """Clear all recorded data."""
        with self.lock:
            for sid in self.sensor_ids:
                self.sensor_data[sid] = {"x": [], "y": [], "z": [], "timestamps": []}

    def wait_for_key(self):
        """Wait for Enter or Space key press."""
        print("\n" + "=" * 60)
        print("Recording data... Press ENTER or SPACE to stop.")
        print("=" * 60 + "\n")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == "\n" or ch == " " or ch == "\r":
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        print("\nRecording stopped.")

    def compute_tangential_force(self, x_data, y_data):
        """Compute tangential force magnitude: sqrt(x^2 + y^2)."""
        x = np.array(x_data)
        y = np.array(y_data)
        return np.sqrt(x**2 + y**2)

    def create_trial_directory(self):
        """Create a new trial subdirectory with timestamp."""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        trial_dir = os.path.join(self.results_base_dir, f"trial_{timestamp}")
        os.makedirs(trial_dir, exist_ok=True)
        return trial_dir

    def save_raw_data(self, trial_dir, sensor_id, x_data, y_data, z_data, timestamps):
        """Save raw force data to CSV."""
        filename = os.path.join(trial_dir, f"sensor_{sensor_id}_raw_forces.csv")
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "tax_x", "tax_y", "tax_z"])
            for t, x, y, z in zip(timestamps, x_data, y_data, z_data):
                writer.writerow([t, x, y, z])
        rospy.loginfo(f"  Saved raw data: {filename}")
        return filename

    def save_processed_data(self, trial_dir, sensor_id, tangential, normal):
        """Save processed tangential and normal force data to CSV."""
        filename = os.path.join(trial_dir, f"sensor_{sensor_id}_processed_forces.csv")
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["normal_force", "tangential_force"])
            for n, t in zip(normal, tangential):
                writer.writerow([n, t])
        rospy.loginfo(f"  Saved processed data: {filename}")
        return filename

    def save_regression_result(
        self,
        trial_dir,
        sensor_id,
        friction_coef,
        intercept,
        lower_pct,
        upper_pct,
        n_points,
    ):
        """Save regression result to CSV with header row + data row."""
        filename = os.path.join(trial_dir, f"sensor_{sensor_id}_regression_result.csv")
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            # Header row
            writer.writerow(
                [
                    "friction_coefficient",
                    "intercept",
                    "lower_bound_percent",
                    "upper_bound_percent",
                    "n_data_points",
                    "odr_intercept_setting",
                ]
            )
            # Data row
            intercept_val = intercept if intercept is not None else "fixed_at_origin"
            writer.writerow(
                [
                    friction_coef,
                    intercept_val,
                    lower_pct,
                    upper_pct,
                    n_points,
                    self.odr_intercept if self.odr_intercept is not None else "None",
                ]
            )
        rospy.loginfo(f"  Saved regression result: {filename}")
        return filename

    def plot_force_data(self, normal, tangential, sensor_id, ax=None):
        """Display interactive plot with alternating background stripes.

        Plots tangential and normal forces as separate series over sample index.
        If ax is provided, plot on that axis; otherwise create a new figure.
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 6))
            standalone = True
        else:
            fig = ax.get_figure()
            standalone = False

        n_points = len(normal)
        indices = np.arange(n_points)

        # Data range for y-axis
        all_forces = np.concatenate([normal, tangential])
        y_min, y_max = all_forces.min(), all_forces.max()
        y_margin = (y_max - y_min) * 0.05 if y_max != y_min else 0.1

        # Alternating background stripes (5 sets = 10 stripes along x-axis)
        stripe_width = n_points / 10
        for i in range(10):
            if i % 2 == 1:  # Colored stripes
                ax.axvspan(
                    i * stripe_width,
                    (i + 1) * stripe_width,
                    alpha=0.15,
                    color="lightblue",
                    zorder=0,
                )

        # Plot tangential and normal forces as separate series
        ax.scatter(indices, normal, c="blue", s=8, alpha=0.7, label="Normal Force (z)")
        ax.scatter(
            indices, tangential, c="red", s=8, alpha=0.7, label="Tangential Force"
        )

        # Labels
        ax.set_xlabel("Sample Index", fontsize=10)
        ax.set_ylabel("Force (N)", fontsize=10)
        ax.set_title(f"Sensor {sensor_id}", fontsize=12)

        # Set limits tight to data (no extra margins on x)
        ax.set_xlim(0, n_points)
        ax.set_ylim(y_min - y_margin, y_max + y_margin)

        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        if standalone:
            plt.tight_layout()
            plt.show(block=False)
            plt.pause(0.5)

        return fig

    def plot_all_sensors(self, processed_data, trial_dir):
        """Plot all sensors in a single figure with side-by-side axes.

        Args:
            processed_data: dict of {sensor_id: {'normal': array, 'tangential': array}}
            trial_dir: directory to save the plot

        Returns:
            fig: matplotlib figure
        """
        n_sensors = len(processed_data)

        if n_sensors == 1:
            fig, axes = plt.subplots(1, 1, figsize=(12, 6))
            axes = [axes]
        else:
            fig, axes = plt.subplots(1, n_sensors, figsize=(6 * n_sensors, 6))
            if not isinstance(axes, np.ndarray):
                axes = [axes]

        for ax, (sensor_id, data) in zip(axes, processed_data.items()):
            self.plot_force_data(data["normal"], data["tangential"], sensor_id, ax=ax)

        plt.tight_layout()

        # Save plot
        plot_path = os.path.join(trial_dir, "force_data_plot.png")
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")
        rospy.loginfo(f"  Saved plot: {plot_path}")

        plt.show(block=False)
        plt.pause(0.5)

        return fig

    def get_range_input(self):
        """Get lower and upper bound percentages from user."""
        while True:
            try:
                print("\n" + "=" * 60)
                user_input = input(
                    "Enter lower and upper bounds (format: lower upper) > "
                ).strip()
                print("=" * 60)

                parts = user_input.split()
                if len(parts) != 2:
                    print(
                        "Invalid format. Please enter two numbers separated by space."
                    )
                    continue

                lower = float(parts[0])
                upper = float(parts[1])

                if not (0 <= lower < upper <= 100):
                    print("Invalid range. Lower must be < upper, both in [0, 100].")
                    continue

                return lower, upper

            except ValueError:
                print("Invalid input. Please enter numeric values.")

    def compute_friction_coefficient_odr(self, normal, tangential):
        """Compute friction coefficient using Orthogonal Distance Regression (TLS)."""
        from scipy import odr

        # Define linear model: tangential = slope * normal + intercept
        if self.odr_intercept is not None:
            # Fixed intercept (e.g., 0)
            def linear_func(B, x):
                return B[0] * x + self.odr_intercept

            linear_model = odr.Model(linear_func)
            data = odr.Data(normal, tangential)
            odr_obj = odr.ODR(data, linear_model, beta0=[0.5])
            output = odr_obj.run()
            slope = output.beta[0]
            intercept = self.odr_intercept
        else:
            # Free intercept
            def linear_func(B, x):
                return B[0] * x + B[1]

            linear_model = odr.Model(linear_func)
            data = odr.Data(normal, tangential)
            odr_obj = odr.ODR(data, linear_model, beta0=[0.5, 0.0])
            output = odr_obj.run()
            slope = output.beta[0]
            intercept = output.beta[1]

        return slope, intercept

    def confirm_result(self, sensor_id, friction_coef, intercept):
        """Ask user to confirm the result."""
        print("\n" + "=" * 60)
        print(f"Sensor {sensor_id} Results:")
        print(f"  Friction Coefficient (slope): {friction_coef:.6f}")
        if self.odr_intercept is not None:
            print(f"  Intercept: {intercept:.6f} (fixed)")
        else:
            print(f"  Intercept: {intercept:.6f} (fitted)")
        print("=" * 60)

        while True:
            response = input("Is this value acceptable? (y/n) > ").strip().lower()
            if response in ["y", "yes"]:
                return True
            elif response in ["n", "no"]:
                return False
            else:
                print("Please enter 'y' or 'n'.")

    def process_sensor_data(self, sensor_id, trial_dir):
        """Process and save data for a single sensor (without plotting).

        Returns:
            dict with 'normal', 'tangential' arrays, or None if no data
        """
        data = self.sensor_data[sensor_id]

        if len(data["x"]) == 0:
            rospy.logwarn(f"No data recorded for sensor {sensor_id}")
            return None

        x_data = data["x"]
        y_data = data["y"]
        z_data = data["z"]
        timestamps = data["timestamps"]

        rospy.loginfo(f"\nProcessing sensor {sensor_id}: {len(x_data)} data points")

        # Save raw data
        self.save_raw_data(trial_dir, sensor_id, x_data, y_data, z_data, timestamps)

        # Compute tangential force
        tangential = self.compute_tangential_force(x_data, y_data)
        normal = np.array(z_data)

        # Save processed data
        self.save_processed_data(
            trial_dir, sensor_id, tangential.tolist(), normal.tolist()
        )

        return {"normal": normal, "tangential": tangential}

    def process_sensor_regression(self, sensor_id, normal, tangential, trial_dir):
        """Perform regression for a single sensor after range selection.

        Returns:
            (friction_coef, intercept) tuple, or None if failed
        """
        # Get range from user
        print(f"\n--- Sensor {sensor_id} ---")
        lower_pct, upper_pct = self.get_range_input()

        # Extract data within range
        n_points = len(normal)
        lower_idx = int(n_points * lower_pct / 100)
        upper_idx = int(n_points * upper_pct / 100)

        normal_subset = normal[lower_idx:upper_idx]
        tangential_subset = tangential[lower_idx:upper_idx]

        if len(normal_subset) < 2:
            rospy.logerr("Not enough data points in selected range.")
            return None

        # Compute friction coefficient
        friction_coef, intercept = self.compute_friction_coefficient_odr(
            normal_subset, tangential_subset
        )

        # Save regression result
        self.save_regression_result(
            trial_dir,
            sensor_id,
            friction_coef,
            intercept,
            lower_pct,
            upper_pct,
            len(normal_subset),
        )

        return (friction_coef, intercept)

    def run(self):
        """Main run loop."""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Friction Identifier - Starting identification process")
        rospy.loginfo("=" * 60)

        while not rospy.is_shutdown():
            # Clear previous data
            self.clear_data()

            # Start recording
            self.recording = True
            self.wait_for_key()
            self.recording = False

            # Create trial directory
            trial_dir = self.create_trial_directory()
            rospy.loginfo(f"\nTrial directory: {trial_dir}")

            # Process and save data for all sensors
            processed_data = {}
            for sensor_id in self.sensor_ids:
                result = self.process_sensor_data(sensor_id, trial_dir)
                if result is not None:
                    processed_data[sensor_id] = result

            if not processed_data:
                rospy.logwarn("No data recorded for any sensor. Restarting...")
                continue

            # Plot all sensors together
            fig = self.plot_all_sensors(processed_data, trial_dir)

            # Get range ONCE for all sensors
            print("\n" + "=" * 60)
            print(
                "Specify the data range to use for regression (applies to ALL sensors)"
            )
            lower_pct, upper_pct = self.get_range_input()

            # Compute regression for all sensors using the same range
            regression_results = {}
            all_regression_ok = True
            for sensor_id, data in processed_data.items():
                normal = data["normal"]
                tangential = data["tangential"]
                n_points = len(normal)
                lower_idx = int(n_points * lower_pct / 100)
                upper_idx = int(n_points * upper_pct / 100)

                normal_subset = normal[lower_idx:upper_idx]
                tangential_subset = tangential[lower_idx:upper_idx]

                if len(normal_subset) < 2:
                    rospy.logerr(
                        f"Sensor {sensor_id}: Not enough data points in selected range."
                    )
                    all_regression_ok = False
                    break

                # Compute friction coefficient
                friction_coef, intercept = self.compute_friction_coefficient_odr(
                    normal_subset, tangential_subset
                )

                # Save regression result
                self.save_regression_result(
                    trial_dir,
                    sensor_id,
                    friction_coef,
                    intercept,
                    lower_pct,
                    upper_pct,
                    len(normal_subset),
                )

                regression_results[sensor_id] = (friction_coef, intercept)

            plt.close(fig)

            if not all_regression_ok:
                rospy.logwarn("Regression failed. Restarting...")
                continue

            # Show ALL results together
            print("\n" + "=" * 60)
            print("REGRESSION RESULTS")
            print("=" * 60)
            for sensor_id, (friction_coef, intercept) in regression_results.items():
                print(f"\n  Sensor {sensor_id}:")
                print(f"    Friction Coefficient (slope): {friction_coef:.6f}")
                if self.odr_intercept is not None:
                    print(f"    Intercept: {intercept:.6f} (fixed)")
                else:
                    print(f"    Intercept: {intercept:.6f} (fitted)")
            print("\n" + "=" * 60)
            print(f"Topics to publish:")
            for sensor_id in regression_results.keys():
                print(f"  /friction_identifier/friction_coefficient/sensor_{sensor_id}")
            print("=" * 60)

            # Single confirmation for all results
            while True:
                response = (
                    input("Accept these results and publish? (y/n) > ").strip().lower()
                )
                if response in ["y", "yes"]:
                    # Publish all friction coefficients
                    friction_values = []
                    # Ensure order based on sensor_ids
                    for sensor_id in self.sensor_ids:
                        if sensor_id in regression_results:
                            friction_values.append(regression_results[sensor_id][0])
                        else:
                            friction_values.append(0.0)  # Should not happen given logic above

                    # Publish individual topics
                    for sensor_id, (friction_coef, _) in regression_results.items():
                        self.friction_publishers[sensor_id].publish(
                            Float32(friction_coef)
                        )
                        rospy.loginfo(
                            f"Published friction coefficient for sensor {sensor_id}: {friction_coef:.6f}"
                        )

                    # Publish aggregated topic
                    array_msg = Float32MultiArray(data=friction_values)
                    self.friction_array_pub.publish(array_msg)
                    rospy.loginfo(f"Published aggregated friction coefficients: {friction_values}")

                    rospy.loginfo("\nAll sensor friction coefficients published.")
                    print()
                    print("-" * 60)
                    print("Press ENTER to restart identification, or Ctrl+C to exit...")
                    print("-" * 60)
                    try:
                        input()
                        rospy.loginfo("Restarting identification process...")
                        break  # Break inner loop to restart outer loop
                    except EOFError:
                        rospy.spin()
                        return
                elif response in ["n", "no"]:
                    rospy.logwarn("Results not accepted. Restarting...")
                    break
                else:
                    print("Please enter 'y' or 'n'.")


def main():
    try:
        node = FrictionIdentifier()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
