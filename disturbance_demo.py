"""
disturbance_demo.py
─────────────────────────────────────────────────────────────
ENEE461 Controls Lab — Disturbance Rejection Data Logger

Same as hover_position_hold.py, but logs ALL telemetry data
to a CSV file for analysis and plotting in your final report.

After running, you'll have a CSV with timestamped columns for:
  - roll, pitch, yaw (attitude)
  - x, y, z (estimated position)
  - vx, vy (velocity)
  - range.front/back/left/right/up (multi-ranger distances)
  - pm.vbat (battery voltage)

You can then plot the disturbance response in Python/MATLAB
to show the PID controller bringing the drone back to origin.

Usage:
    python disturbance_demo.py

Output:
    flight_data_YYYYMMDD_HHMMSS.csv
─────────────────────────────────────────────────────────────
"""

import csv
import sys
import time
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ── Configuration ──────────────────────────────────────────
URI = "radio://0/100/2M"    # ← Change to match your radio URI
HOVER_HEIGHT = 1.0          # meters
HOVER_TIME = 30             # seconds (increase for longer demos)
# ───────────────────────────────────────────────────────────

# Data storage
flight_data = []
start_time = None


def check_decks(scf):
    """Verify decks before flight."""
    cf = scf.cf
    time.sleep(1)

    flow = int(cf.param.get_value("deck.bcFlow2") or 0)
    ranger = int(cf.param.get_value("deck.bcMultiranger") or 0)

    print(f"  Flow deck v2:  {'OK' if flow else 'NOT DETECTED'}")
    print(f"  Multi-ranger:  {'OK' if ranger else 'NOT DETECTED'}")

    if not flow:
        print("\n  ERROR: Flow deck required. Aborting.")
        sys.exit(1)

    return flow, ranger


def setup_data_logging(scf, has_ranger):
    """Set up comprehensive data logging for the CSV."""
    global start_time
    start_time = time.time()

    # Current data row (filled in by callbacks)
    current = {
        "roll": 0, "pitch": 0, "yaw": 0,
        "x": 0, "y": 0, "z": 0,
        "vx": 0, "vy": 0,
        "front": 0, "back": 0, "left": 0, "right": 0, "up": 0,
        "vbat": 0,
    }

    # --- Attitude + altitude (10 Hz) ---
    log_stab = LogConfig(name="Stab", period_in_ms=100)
    log_stab.add_variable("stabilizer.roll", "float")
    log_stab.add_variable("stabilizer.pitch", "float")
    log_stab.add_variable("stabilizer.yaw", "float")
    log_stab.add_variable("stateEstimate.z", "float")

    def stab_cb(timestamp, data, logconf):
        current["roll"] = data["stabilizer.roll"]
        current["pitch"] = data["stabilizer.pitch"]
        current["yaw"] = data["stabilizer.yaw"]
        current["z"] = data["stateEstimate.z"]

    scf.cf.log.add_config(log_stab)
    log_stab.data_received_cb.add_callback(stab_cb)

    # --- Position + velocity (10 Hz) ---
    log_pos = LogConfig(name="Pos", period_in_ms=100)
    log_pos.add_variable("stateEstimate.x", "float")
    log_pos.add_variable("stateEstimate.y", "float")
    log_pos.add_variable("stateEstimate.vx", "float")
    log_pos.add_variable("stateEstimate.vy", "float")

    def pos_cb(timestamp, data, logconf):
        current["x"] = data["stateEstimate.x"]
        current["y"] = data["stateEstimate.y"]
        current["vx"] = data["stateEstimate.vx"]
        current["vy"] = data["stateEstimate.vy"]

        # Record a row every position update (10 Hz)
        t = time.time() - start_time
        row = {"time": round(t, 3)}
        row.update(current)
        flight_data.append(row)

        # Print live
        print(f"\r  t={t:5.1f}s  z={current['z']:.2f}m  "
              f"pos=({current['x']:+.2f},{current['y']:+.2f})  "
              f"roll={current['roll']:+5.1f} pitch={current['pitch']:+5.1f}",
              end="", flush=True)

    scf.cf.log.add_config(log_pos)
    log_pos.data_received_cb.add_callback(pos_cb)

    # --- Multi-ranger distances (5 Hz) ---
    log_range = None
    if has_ranger:
        log_range = LogConfig(name="Range", period_in_ms=200)
        log_range.add_variable("range.front", "uint16_t")
        log_range.add_variable("range.back", "uint16_t")
        log_range.add_variable("range.left", "uint16_t")
        log_range.add_variable("range.right", "uint16_t")
        log_range.add_variable("range.up", "uint16_t")

        def range_cb(timestamp, data, logconf):
            current["front"] = data["range.front"]
            current["back"] = data["range.back"]
            current["left"] = data["range.left"]
            current["right"] = data["range.right"]
            current["up"] = data["range.up"]

        scf.cf.log.add_config(log_range)
        log_range.data_received_cb.add_callback(range_cb)

    # --- Battery (0.5 Hz) ---
    log_bat = LogConfig(name="Bat", period_in_ms=2000)
    log_bat.add_variable("pm.vbat", "float")

    def bat_cb(timestamp, data, logconf):
        current["vbat"] = data["pm.vbat"]
        if data["pm.vbat"] < 3.2:
            print(f"\n  *** BATTERY LOW: {data['pm.vbat']:.2f}V ***")

    scf.cf.log.add_config(log_bat)
    log_bat.data_received_cb.add_callback(bat_cb)

    logs = [log_stab, log_pos, log_bat]
    if log_range:
        logs.append(log_range)

    return logs


def save_csv(filename):
    """Save flight data to CSV."""
    if not flight_data:
        print("No data to save.")
        return

    fieldnames = [
        "time", "x", "y", "z", "vx", "vy",
        "roll", "pitch", "yaw",
        "front", "back", "left", "right", "up",
        "vbat"
    ]

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(flight_data)

    print(f"  Saved {len(flight_data)} data points to {filename}")


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"flight_data_{timestamp_str}.csv"

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")

            # Check hardware
            flow_ok, ranger_ok = check_decks(scf)

            # Reset estimator
            scf.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            scf.cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1)
            print("Kalman estimator reset.\n")

            # Start logging
            logs = setup_data_logging(scf, ranger_ok)
            for log in logs:
                log.start()

            print(f"{'='*60}")
            print(f"  DISTURBANCE REJECTION DEMO")
            print(f"  Altitude: {HOVER_HEIGHT}m | Duration: {HOVER_TIME}s")
            print(f"  Data logging to: {csv_filename}")
            print(f"")
            print(f"  INSTRUCTIONS:")
            print(f"  1. Let it stabilize for 5 seconds")
            print(f"  2. Gently push the drone with a stick/finger")
            print(f"  3. Watch it return to origin")
            print(f"  4. Repeat a few times for good data")
            print(f"  Press Ctrl+C to land")
            print(f"{'='*60}\n")

            time.sleep(3)  # step back

            try:
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    print(f"Airborne — position hold ACTIVE. Recording data...\n")

                    start = time.time()
                    while time.time() - start < HOVER_TIME:
                        time.sleep(0.1)

                    print("\n\nHover complete — landing...")

            except KeyboardInterrupt:
                print("\n\nCtrl+C — landing!")

            # Stop logging
            for log in logs:
                log.stop()

            # Save data
            print(f"\nSaving flight data...")
            save_csv(csv_filename)
            print("Done!")

    except KeyboardInterrupt:
        print("\nAborted before takeoff.")
        if flight_data:
            save_csv(csv_filename)
    except Exception as e:
        print(f"\nError: {e}")
        if flight_data:
            save_csv(csv_filename)


if __name__ == "__main__":
    main()
