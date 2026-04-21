"""
hover_position_hold.py
─────────────────────────────────────────────────────────────
ENEE461 Controls Lab — Crazyflie 2.1 Position Hold Demo

This script makes the Crazyflie:
  1. Take off to 1.0m altitude
  2. Hold its position using the Flow deck v2 (optical flow + ToF)
  3. Actively resist disturbances — if pushed, it returns to origin
  4. Print live telemetry: height, roll, pitch, yaw, x, y position
  5. Land after the hover period

Hardware required:
  - Crazyflie 2.1 with Flow deck v2 (altitude + optical flow)
  - Multi-ranger deck (optional, for wall-distance feedback)
  - Crazyradio PA USB dongle
  - Charged battery (>3.3V)

How it works:
  The Crazyflie's onboard Kalman filter fuses the Flow deck's ToF
  (height) and optical flow (horizontal velocity) to estimate the
  drone's 3D position. The built-in PID controller then drives the
  drone back to the commanded setpoint whenever it's displaced.

  This is a cascaded PID architecture:
    Outer loop: position → desired velocity
    Inner loop: velocity → desired attitude (roll/pitch)
    Innermost: attitude → motor commands

Usage:
    python hover_position_hold.py

Press Ctrl+C at any time to trigger emergency landing.
─────────────────────────────────────────────────────────────
"""

import sys
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ── Configuration ──────────────────────────────────────────
URI = "radio://0/80/2M/E7E7E7E13"    # ← Change to match your radio URI
HOVER_HEIGHT = 1.0          # meters above ground
HOVER_TIME = 30             # seconds to hold position
# ───────────────────────────────────────────────────────────


def check_decks(scf):
    """Verify that the Flow deck v2 is attached."""
    print("Checking decks...")
    cf = scf.cf
    time.sleep(1)  # let params load

    flow = int(cf.param.get_value("deck.bcFlow2") or 0)
    ranger = int(cf.param.get_value("deck.bcMultiranger") or 0)

    print(f"  Flow deck v2:   {'OK' if flow else 'NOT DETECTED'}")
    print(f"  Multi-ranger:   {'OK' if ranger else 'NOT DETECTED'}")

    if not flow:
        print("\n  ERROR: Flow deck is required for position hold!")
        print("  Cannot fly safely without it. Aborting.")
        sys.exit(1)

    return flow, ranger


def setup_logging(scf):
    """Set up real-time telemetry logging."""

    # Log group 1: attitude + height
    log_stab = LogConfig(name="Stabilizer", period_in_ms=100)
    log_stab.add_variable("stabilizer.roll", "float")
    log_stab.add_variable("stabilizer.pitch", "float")
    log_stab.add_variable("stabilizer.yaw", "float")
    log_stab.add_variable("stateEstimate.z", "float")

    # Log group 2: estimated x, y position (from optical flow)
    log_pos = LogConfig(name="Position", period_in_ms=100)
    log_pos.add_variable("stateEstimate.x", "float")
    log_pos.add_variable("stateEstimate.y", "float")
    log_pos.add_variable("stateEstimate.vx", "float")
    log_pos.add_variable("stateEstimate.vy", "float")

    # Log group 3: battery
    log_bat = LogConfig(name="Battery", period_in_ms=2000)
    log_bat.add_variable("pm.vbat", "float")

    def stab_cb(timestamp, data, logconf):
        r = data["stabilizer.roll"]
        p = data["stabilizer.pitch"]
        y = data["stabilizer.yaw"]
        z = data["stateEstimate.z"]
        print(f"  ALT={z:.2f}m  roll={r:+6.1f}  pitch={p:+6.1f}  yaw={y:+6.1f}", end="")

    def pos_cb(timestamp, data, logconf):
        x = data["stateEstimate.x"]
        y = data["stateEstimate.y"]
        vx = data["stateEstimate.vx"]
        vy = data["stateEstimate.vy"]
        print(f"  pos=({x:+.2f},{y:+.2f})  vel=({vx:+.2f},{vy:+.2f})")

    def bat_cb(timestamp, data, logconf):
        v = data["pm.vbat"]
        if v < 3.2:
            print(f"\n  *** BATTERY LOW: {v:.2f}V — LAND NOW ***")

    scf.cf.log.add_config(log_stab)
    log_stab.data_received_cb.add_callback(stab_cb)

    scf.cf.log.add_config(log_pos)
    log_pos.data_received_cb.add_callback(pos_cb)

    scf.cf.log.add_config(log_bat)
    log_bat.data_received_cb.add_callback(bat_cb)

    return log_stab, log_pos, log_bat


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")

            # Safety check
            flow_ok, ranger_ok = check_decks(scf)

            # Enable Kalman estimator reset (clean state before flight)
            scf.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            scf.cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1)  # let estimator converge
            print("Kalman estimator reset.\n")

            # Start logging
            log_stab, log_pos, log_bat = setup_logging(scf)
            log_stab.start()
            log_pos.start()
            log_bat.start()

            print(f"{'='*60}")
            print(f"  TAKING OFF to {HOVER_HEIGHT}m")
            print(f"  Hovering for {HOVER_TIME}s with position hold")
            print(f"  Push the drone gently — it will return to origin")
            print(f"  Press Ctrl+C to land immediately")
            print(f"{'='*60}\n")

            time.sleep(2)  # step back

            try:
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    print(f"Airborne at {HOVER_HEIGHT}m — position hold ACTIVE\n")

                    # The MotionCommander keeps the drone at the
                    # takeoff position. The Flow deck + Kalman filter
                    # + PID controller handle all disturbance rejection.
                    # We just need to keep the connection alive.

                    start = time.time()
                    while time.time() - start < HOVER_TIME:
                        time.sleep(0.1)

                    print("\n\nHover complete — landing...")
                # Auto-lands when exiting MotionCommander block

            except KeyboardInterrupt:
                print("\n\nCtrl+C — landing immediately!")

            # Clean up
            log_stab.stop()
            log_pos.stop()
            log_bat.stop()
            print("Done. Drone has landed safely.")

    except KeyboardInterrupt:
        print("\nAborted before takeoff.")
    except Exception as e:
        print(f"\nError: {e}")
        print("Check that Crazyflie is on and radio dongle is plugged in.")


if __name__ == "__main__":
    main()
