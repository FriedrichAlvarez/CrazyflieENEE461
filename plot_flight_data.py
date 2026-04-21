"""
plot_flight_data.py
─────────────────────────────────────────────────────────────
ENEE461 Controls Lab — Flight Data Plotter

Reads CSV files from builtin_pid_hover.py or custom_pid_hover.py
and generates plots for your lab report.

Plots generated:
  1. Position over time (x, y, z) — shows disturbance + recovery
  2. Attitude over time (roll, pitch, yaw)
  3. XY trajectory (bird's eye view of position)
  4. Z altitude tracking vs setpoint
  5. (Custom PID only) Thrust and PID components

Usage:
    python plot_flight_data.py builtin_pid_20260421_143000.csv
    python plot_flight_data.py custom_pid_20260421_143500.csv
    python plot_flight_data.py builtin.csv custom.csv   # comparison

─────────────────────────────────────────────────────────────
"""

import sys
import csv
import os

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend for saving
except ImportError:
    print("matplotlib not installed. Run: pip install matplotlib")
    sys.exit(1)


def load_csv(filename):
    """Load flight data CSV into a dict of lists."""
    data = {}
    with open(filename, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, val in row.items():
                if key not in data:
                    data[key] = []
                try:
                    data[key].append(float(val))
                except (ValueError, TypeError):
                    data[key].append(val)
    return data


def plot_single(data, label, output_prefix):
    """Generate plots for a single flight CSV."""

    t = data.get("time", [])
    if not t:
        print("No time column found.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"Flight Data — {label}", fontsize=14, fontweight="bold")

    # ── Plot 1: Position over time ──
    ax = axes[0][0]
    ax.plot(t, data.get("x", []), label="X (fwd)", linewidth=1)
    ax.plot(t, data.get("y", []), label="Y (left)", linewidth=1)
    ax.plot(t, data.get("z", []), label="Z (alt)", linewidth=1.5, color="green")
    ax.axhline(y=1.0, color="green", linestyle="--", alpha=0.5, label="Z target")
    ax.axhline(y=0.0, color="gray", linestyle="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Position vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Plot 2: Attitude over time ──
    ax = axes[0][1]
    ax.plot(t, data.get("roll", []), label="Roll", linewidth=0.8)
    ax.plot(t, data.get("pitch", []), label="Pitch", linewidth=0.8)
    ax.plot(t, data.get("yaw", []), label="Yaw", linewidth=0.8)
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (degrees)")
    ax.set_title("Attitude vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Plot 3: XY trajectory (top-down) ──
    ax = axes[1][0]
    x_vals = data.get("x", [])
    y_vals = data.get("y", [])
    if x_vals and y_vals:
        scatter = ax.scatter(y_vals, x_vals, c=t, cmap="viridis",
                            s=2, alpha=0.7)
        ax.plot(y_vals[0], x_vals[0], "go", markersize=8, label="Start")
        ax.plot(0, 0, "r+", markersize=12, label="Origin")
        plt.colorbar(scatter, ax=ax, label="Time (s)")
    ax.set_xlabel("Y position (m)")
    ax.set_ylabel("X position (m)")
    ax.set_title("XY Trajectory (top-down)")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Plot 4: Altitude tracking ──
    ax = axes[1][1]
    z_vals = data.get("z", [])
    ax.plot(t, z_vals, label="Measured Z", linewidth=1.5, color="blue")
    ax.axhline(y=1.0, color="red", linestyle="--", linewidth=1,
               label="Setpoint (1.0m)")
    if z_vals:
        ax.fill_between(t, z_vals, 1.0, alpha=0.15, color="blue")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude Tracking")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fname = f"{output_prefix}_plots.png"
    plt.savefig(fname, dpi=150, bbox_inches="tight")
    print(f"  Saved: {fname}")
    plt.close()

    # ── Extra plot for custom PID: thrust + PID components ──
    if "thrust" in data and "pid_z_p" in data:
        fig2, axes2 = plt.subplots(2, 1, figsize=(12, 8))
        fig2.suptitle(f"PID Internals — {label}", fontsize=14,
                      fontweight="bold")

        ax = axes2[0]
        ax.plot(t, data["thrust"], label="Thrust cmd", linewidth=1)
        ax.axhline(y=38000, color="gray", linestyle="--", alpha=0.5,
                   label="Base thrust")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Thrust (0-65535)")
        ax.set_title("Thrust Command")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes2[1]
        ax.plot(t, data.get("pid_z_p", []), label="P term", linewidth=0.8)
        ax.plot(t, data.get("pid_z_i", []), label="I term", linewidth=0.8)
        ax.plot(t, data.get("pid_z_d", []), label="D term", linewidth=0.8)
        ax.axhline(y=0, color="gray", linestyle="--", alpha=0.3)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("PID output")
        ax.set_title("Z-axis PID Components (P, I, D)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        fname2 = f"{output_prefix}_pid_internals.png"
        plt.savefig(fname2, dpi=150, bbox_inches="tight")
        print(f"  Saved: {fname2}")
        plt.close()


def plot_comparison(data1, label1, data2, label2):
    """Compare two flights side by side."""

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"Comparison: {label1} vs {label2}", fontsize=14,
                 fontweight="bold")

    t1 = data1.get("time", [])
    t2 = data2.get("time", [])

    # ── Z altitude comparison ──
    ax = axes[0][0]
    ax.plot(t1, data1.get("z", []), label=label1, linewidth=1)
    ax.plot(t2, data2.get("z", []), label=label2, linewidth=1)
    ax.axhline(y=1.0, color="red", linestyle="--", alpha=0.5,
               label="Setpoint")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude Tracking Comparison")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── X position comparison ──
    ax = axes[0][1]
    ax.plot(t1, data1.get("x", []), label=label1, linewidth=1)
    ax.plot(t2, data2.get("x", []), label=label2, linewidth=1)
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X position (m)")
    ax.set_title("X Position Comparison")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Roll comparison ──
    ax = axes[1][0]
    ax.plot(t1, data1.get("roll", []), label=label1, linewidth=0.8)
    ax.plot(t2, data2.get("roll", []), label=label2, linewidth=0.8)
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll (degrees)")
    ax.set_title("Roll Comparison")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── XY trajectory comparison ──
    ax = axes[1][1]
    ax.scatter(data1.get("y", []), data1.get("x", []),
               s=2, alpha=0.5, label=label1)
    ax.scatter(data2.get("y", []), data2.get("x", []),
               s=2, alpha=0.5, label=label2)
    ax.plot(0, 0, "r+", markersize=12, label="Origin")
    ax.set_xlabel("Y (m)")
    ax.set_ylabel("X (m)")
    ax.set_title("XY Trajectory Comparison")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("comparison_plots.png", dpi=150, bbox_inches="tight")
    print("  Saved: comparison_plots.png")
    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python plot_flight_data.py <file.csv>")
        print("  python plot_flight_data.py <builtin.csv> <custom.csv>")
        sys.exit(1)

    if len(sys.argv) == 2:
        # Single file
        fname = sys.argv[1]
        print(f"Loading {fname}...")
        data = load_csv(fname)
        label = os.path.splitext(os.path.basename(fname))[0]
        plot_single(data, label, label)

    elif len(sys.argv) >= 3:
        # Comparison mode
        f1, f2 = sys.argv[1], sys.argv[2]
        print(f"Loading {f1} and {f2} for comparison...")
        d1 = load_csv(f1)
        d2 = load_csv(f2)
        l1 = os.path.splitext(os.path.basename(f1))[0]
        l2 = os.path.splitext(os.path.basename(f2))[0]

        plot_single(d1, l1, l1)
        plot_single(d2, l2, l2)
        plot_comparison(d1, l1, d2, l2)

    print("\nDone! Open the PNG files to view your plots.")


if __name__ == "__main__":
    main()
