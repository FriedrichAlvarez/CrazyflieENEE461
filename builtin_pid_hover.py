"""
builtin_pid_hover.py
─────────────────────────────────────────────────────────────
ENEE461 Controls Lab — Built-in PID Position Hold

This script uses the Crazyflie's ONBOARD PID controller (no custom
control logic). The MotionCommander API handles everything:
  - Altitude hold via Flow deck ToF sensor
  - Position hold via optical flow + Kalman filter
  - Attitude stabilization via onboard rate/attitude PID

This is the "baseline" — it shows what the factory PID can do
out of the box. Compare this against your custom PID script
to demonstrate what tuning and understanding the system achieves.

Architecture (all onboard):
  ┌─────────────────────────────────────────────────┐
  │  Position setpoint (x=0, y=0, z=1.0)           │
  │       ↓                                         │
  │  Position PID → desired velocity                │
  │       ↓                                         │
  │  Velocity PID → desired attitude (roll/pitch)   │
  │       ↓                                         │
  │  Attitude PID → desired body rates              │
  │       ↓                                         │
  │  Rate PID → motor PWM commands                  │
  └─────────────────────────────────────────────────┘

Usage:
    python builtin_pid_hover.py

Press Ctrl+C to land immediately.
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
URI = "radio://0/100/2M"    # ← Change to your radio URI
HOVER_HEIGHT = 1.0          # meters
HOVER_TIME = 30             # seconds
LOG_TO_CSV = True           # save flight data for comparison
# ───────────────────────────────────────────────────────────

flight_data = []
t0 = None


def check_decks(scf):
    cf = scf.cf
    time.sleep(1)
    flow = int(cf.param.get_value("deck.bcFlow2") or 0)
    ranger = int(cf.param.get_value("deck.bcMultiranger") or 0)
    print(f"  Flow deck v2:  {'OK' if flow else 'MISSING'}")
    print(f"  Multi-ranger:  {'OK' if ranger else 'MISSING'}")
    if not flow:
        print("  ABORT: Flow deck required for altitude + position hold.")
        sys.exit(1)
    return ranger


def setup_logging(scf, has_ranger):
    global t0
    t0 = time.time()

    # Attitude + altitude at 10 Hz
    log_stab = LogConfig(name="Stab", period_in_ms=100)
    log_stab.add_variable("stabilizer.roll", "float")
    log_stab.add_variable("stabilizer.pitch", "float")
    log_stab.add_variable("stabilizer.yaw", "float")
    log_stab.add_variable("stateEstimate.z", "float")

    # Position + velocity at 10 Hz
    log_pos = LogConfig(name="Pos", period_in_ms=100)
    log_pos.add_variable("stateEstimate.x", "float")
    log_pos.add_variable("stateEstimate.y", "float")
    log_pos.add_variable("stateEstimate.vx", "float")
    log_pos.add_variable("stateEstimate.vy", "float")

    # Ranger at 5 Hz (optional)
    log_range = None
    if has_ranger:
        log_range = LogConfig(name="Range", period_in_ms=200)
        log_range.add_variable("range.front", "uint16_t")
        log_range.add_variable("range.back", "uint16_t")
        log_range.add_variable("range.left", "uint16_t")
        log_range.add_variable("range.right", "uint16_t")
        log_range.add_variable("range.up", "uint16_t")

    current = {
        "roll": 0, "pitch": 0, "yaw": 0,
        "x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0,
        "front": 0, "back": 0, "left": 0, "right": 0, "up": 0,
    }

    def stab_cb(ts, data, lc):
        current["roll"] = data["stabilizer.roll"]
        current["pitch"] = data["stabilizer.pitch"]
        current["yaw"] = data["stabilizer.yaw"]
        current["z"] = data["stateEstimate.z"]

    def pos_cb(ts, data, lc):
        current["x"] = data["stateEstimate.x"]
        current["y"] = data["stateEstimate.y"]
        current["vx"] = data["stateEstimate.vx"]
        current["vy"] = data["stateEstimate.vy"]

        t = time.time() - t0
        row = {"time": round(t, 3), "controller": "builtin"}
        row.update(current)
        flight_data.append(row)

        print(f"\r  t={t:5.1f}s  z={current['z']:.2f}m  "
              f"pos=({current['x']:+.3f},{current['y']:+.3f})  "
              f"r={current['roll']:+5.1f} p={current['pitch']:+5.1f}",
              end="", flush=True)

    def range_cb(ts, data, lc):
        current["front"] = data["range.front"]
        current["back"] = data["range.back"]
        current["left"] = data["range.left"]
        current["right"] = data["range.right"]
        current["up"] = data["range.up"]

    scf.cf.log.add_config(log_stab)
    log_stab.data_received_cb.add_callback(stab_cb)
    scf.cf.log.add_config(log_pos)
    log_pos.data_received_cb.add_callback(pos_cb)

    logs = [log_stab, log_pos]
    if log_range:
        scf.cf.log.add_config(log_range)
        log_range.data_received_cb.add_callback(range_cb)
        logs.append(log_range)

    return logs


def save_csv(filename):
    if not flight_data:
        return
    fields = ["time", "controller", "x", "y", "z", "vx", "vy",
              "roll", "pitch", "yaw", "front", "back", "left", "right", "up"]
    with open(filename, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(flight_data)
    print(f"  Saved {len(flight_data)} rows → {filename}")


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = f"builtin_pid_{ts}.csv"

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")
            has_ranger = check_decks(scf)

            # Reset Kalman estimator for clean state
            scf.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            scf.cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1)
            print("  Kalman estimator reset.\n")

            logs = setup_logging(scf, has_ranger)
            for log in logs:
                log.start()

            print(f"  BUILT-IN PID — hover at {HOVER_HEIGHT}m for {HOVER_TIME}s")
            print(f"  Push the drone to test disturbance rejection.\n")
            time.sleep(2)

            try:
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    print(f"  Airborne — position hold ACTIVE\n")
                    start = time.time()
                    while time.time() - start < HOVER_TIME:
                        time.sleep(0.1)
                    print("\n\n  Landing...")

            except KeyboardInterrupt:
                print("\n\n  Ctrl+C — landing!")

            for log in logs:
                log.stop()

            if LOG_TO_CSV:
                save_csv(csv_file)

            print("  Done.")

    except Exception as e:
        print(f"\nError: {e}")
        if LOG_TO_CSV and flight_data:
            save_csv(csv_file)


if __name__ == "__main__":
    main()
