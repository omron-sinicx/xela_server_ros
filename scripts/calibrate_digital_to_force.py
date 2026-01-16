#!/usr/bin/env python3
"""
Calibration script for XELA sensor digital-to-force conversion.
Uses Orthogonal Distance Regression (ODR) to fit linear models for X, Y, Z axes.

Usage:
    rosrun xela_server_ros calibrate_digital_to_force.py --data-dir <path>
    python3 calibrate_digital_to_force.py --data-dir data/xr1944-2110-45-ctrlr-4
"""

import os
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.odr import ODR, Model, RealData


def linear_func(params, x):
    """Linear model: y = slope * x + intercept"""
    slope, intercept = params
    return slope * x + intercept


def perform_odr(digital: np.ndarray, force: np.ndarray) -> tuple:
    """
    Perform Orthogonal Distance Regression on the given data.

    Args:
        digital: Digital values (input)
        force: Force values (output)

    Returns:
        Tuple of (slope, intercept, slope_std, intercept_std)
    """
    # Initial guess using OLS
    slope_init = np.polyfit(digital, force, 1)[0]
    intercept_init = np.mean(force) - slope_init * np.mean(digital)

    # Set up ODR
    linear_model = Model(linear_func)
    data = RealData(digital, force)
    odr = ODR(data, linear_model, beta0=[slope_init, intercept_init])

    # Run ODR
    output = odr.run()

    slope, intercept = output.beta
    slope_std, intercept_std = output.sd_beta

    return slope, intercept, slope_std, intercept_std


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Calibrate XELA sensor digital-to-force conversion using ODR"
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default=None,
        help="Directory containing tax_x.csv, tax_y.csv, tax_z.csv. "
        "Output files (calibration_params.json, calibration_plot.png) "
        "will be saved in the same directory.",
    )
    args = parser.parse_args()

    # Determine data directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)

    if args.data_dir:
        # Use specified directory (absolute or relative to current working directory)
        if os.path.isabs(args.data_dir):
            data_dir = args.data_dir
        else:
            data_dir = os.path.join(os.getcwd(), args.data_dir)
    else:
        # Default to legacy path
        data_dir = os.path.join(package_dir, "data", "digital_to_force")

    if not os.path.isdir(data_dir):
        print(f"[ERROR] Data directory does not exist: {data_dir}")
        return

    # Output paths
    config_output_path = os.path.join(data_dir, "calibration_params.json")
    plot_output_path = os.path.join(data_dir, "calibration_plot.png")

    # CSV files for each axis
    axes = ["x", "y", "z"]
    csv_files = {
        "x": os.path.join(data_dir, "tax_x.csv"),
        "y": os.path.join(data_dir, "tax_y.csv"),
        "z": os.path.join(data_dir, "tax_z.csv"),
    }

    # Store calibration parameters
    calibration_params = {}
    data_dict = {}

    print("=" * 60)
    print("XELA Sensor Digital-to-Force Calibration")
    print("Using Orthogonal Distance Regression (ODR)")
    print("=" * 60)

    # Perform ODR for each axis
    for axis in axes:
        csv_path = csv_files[axis]
        print(f"\n[{axis.upper()} axis] Loading: {csv_path}")

        # Load data
        df = pd.read_csv(csv_path)
        digital = df["digital"].values.astype(float)
        force = df["measured_frc"].values.astype(float)

        # Store data for plotting
        data_dict[axis] = {"digital": digital, "force": force}

        # Perform ODR
        slope, intercept, slope_std, intercept_std = perform_odr(digital, force)

        # Store parameters
        calibration_params[axis] = {
            "slope": float(slope),
            "intercept": float(intercept),
            "slope_std": float(slope_std),
            "intercept_std": float(intercept_std),
            "unit": "N",
            "model": "force = slope * digital + intercept",
        }

        print(f"  Slope:     {slope:.10e} ± {slope_std:.10e}")
        print(f"  Intercept: {intercept:.6f} ± {intercept_std:.6f} N")

        # Calculate R-squared
        force_pred = linear_func([slope, intercept], digital)
        ss_res = np.sum((force - force_pred) ** 2)
        ss_tot = np.sum((force - np.mean(force)) ** 2)
        r_squared = 1 - ss_res / ss_tot
        calibration_params[axis]["r_squared"] = float(r_squared)
        print(f"  R²:        {r_squared:.6f}")

    # Prepare JSON structure: total/each -> x/y/z
    # "total": センサー全体（16タクセル合計）のパラメータ
    # "each": 各タクセル1個あたりのパラメータ（total / 16）
    json_output = {
        "total": {},
        "each": {},
    }
    for axis, params in calibration_params.items():
        # total parameters (already computed)
        json_output["total"][axis] = params.copy()
        # each-taxel parameters:
        # - slope stays the same (applied to individual taxel digital values)
        # - intercept is divided by 16 (total offset distributed across 16 taxels)
        json_output["each"][axis] = {
            "slope": params["slope"],  # NOT divided by 16
            "intercept": params["intercept"] / 16.0,  # divided by 16
            "slope_std": params.get("slope_std", 0.0),
            "intercept_std": params.get("intercept_std", 0.0) / 16.0,
            "unit": params.get("unit", "N"),
            "model": params.get("model", "force = slope * digital + intercept"),
            "r_squared": params.get("r_squared", None),
        }
    # Save calibration parameters to JSON
    with open(config_output_path, "w") as f:
        json.dump(json_output, f, indent=2)
    print(f"\n[INFO] Calibration parameters saved to: {config_output_path}")

    # Create plot with 3 subplots
    fig, axes_plot = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(
        "XELA Sensor Digital-to-Force Calibration (ODR)", fontsize=14, fontweight="bold"
    )

    axis_labels = {"x": "X-axis (Shear)", "y": "Y-axis (Shear)", "z": "Z-axis (Normal)"}
    colors = {"x": "#E74C3C", "y": "#27AE60", "z": "#3498DB"}

    for idx, axis in enumerate(["x", "y", "z"]):
        ax = axes_plot[idx]
        digital = data_dict[axis]["digital"]
        force = data_dict[axis]["force"]
        params = calibration_params[axis]

        # Plot data points
        ax.scatter(
            digital,
            force,
            c=colors[axis],
            alpha=0.7,
            s=50,
            edgecolors="white",
            linewidth=0.5,
            label="Measured data",
        )

        # Plot regression line
        digital_range = np.linspace(digital.min(), digital.max(), 100)
        force_pred = linear_func([params["slope"], params["intercept"]], digital_range)
        ax.plot(digital_range, force_pred, "k-", linewidth=2, label="ODR fit")

        # Labels and formatting
        ax.set_xlabel("Digital Value", fontsize=11)
        ax.set_ylabel("Force (N)", fontsize=11)
        ax.set_title(
            f"{axis_labels[axis]}\nR² = {params['r_squared']:.4f}", fontsize=12
        )
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)

        # Add equation text
        eq_text = f"F = {params['slope']:.2e} × D + {params['intercept']:.2f}"
        ax.text(
            0.05,
            0.95,
            eq_text,
            transform=ax.transAxes,
            fontsize=9,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    plt.tight_layout()
    plt.savefig(plot_output_path, dpi=150, bbox_inches="tight")
    print(f"[INFO] Calibration plot saved to: {plot_output_path}")

    plt.show()

    print("\n" + "=" * 60)
    print("Calibration complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
