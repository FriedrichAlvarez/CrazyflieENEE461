"""
first_flight_05m.py
─────────────────────────────────────────────────────────────
ENEE461 — First Flight Test: 0.5m Hover

A safe, short first flight to verify everything works.
The drone will:
  1. Take off to 0.5m (low and safe)
  2. Hover for 5 seconds
  3. Land automatically

SAFETY:
  - Press Ctrl+C in terminal → immediate landing
  - If anything goes wrong, UNPLUG THE BATTERY
  - Fly over a flat surface with some texture/pattern
  - Clear the area around the drone (1m radius minimum)
  - Keep your hands AWAY during flight

HOW THIS CONTROLS THE DRONE:
  Your computer sends commands over radio at ~100Hz:

  ┌─────────────┐    radio     ┌─────────────────┐
  │  Your Mac   │ ──────────→  │  Crazyflie 2.1  │
  │  (Python)   │  2.4 GHz     │  (onboard PID)  │
  │             │ ←──────────  │                  │
  │  commands   │  sensor data │  motor control   │
  └─────────────┘              └─────────────────┘

  MotionCommander sends high-level commands:
    "go to height 0.5m" → onboard PID figures out thrust
    "hold position"     → onboard PID adjusts roll/pitch

  The onboard controller does the hard work at 500Hz.
  Your code just tells it WHAT to do, not HOW.

Usage:
    1. Close cfclient GUI first!
    2. python first_flight_05m.py

─────────────────────────────────────────────────────────────
"""

import sys
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ══════════════════════════════════════════════════════════
#  YOUR DRONE'S RADIO URI — this matches what cfclient shows
# ══════════════════════════════════════════════════════════
URI = "radio://0/80/2M/E7E7E7E13"

# ══════════════════════════════════════════════════════════
#  FLIGHT PARAMETERS — start conservative!
# ══════════════════════════════════════════════════════════
HOVER_HEIGHT = 0.5   # meters (start low for safety)
HOVER_TIME = 5       # seconds (short first test)


def main():
    # ── Step 1: Initialize radio drivers ──
    # This sets up the USB dongle so Python can talk to it
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    try:
        # ── Step 2: Connect to the Crazyflie ──
        # SyncCrazyflie is a "context manager" — it connects when
        # entering the 'with' block and disconnects when leaving
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")
            cf = scf.cf

            # ── Step 3: Check that Flow deck is present ──
            # Without it, the drone can't measure height and WILL crash
            time.sleep(1)
            flow = int(cf.param.get_value("deck.bcFlow2") or 0)
            if not flow:
                print("ERROR: Flow deck not detected! Cannot fly safely.")
                sys.exit(1)
            print("Flow deck v2: OK")

            # ── Step 4: Reset the Kalman estimator ──
            # The Kalman filter estimates position from sensor data.
            # Resetting it clears any old state so it starts fresh.
            # The drone MUST be still on a flat surface during this.
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1.5)  # wait for estimator to converge
            print("Kalman estimator: READY\n")

            # ── Step 5: Set up live telemetry ──
            # We ask the drone to send us data every 200ms
            log = LogConfig(name="Flight", period_in_ms=200)
            log.add_variable("stateEstimate.z", "float")    # height
            log.add_variable("stabilizer.roll", "float")     # tilt L/R
            log.add_variable("stabilizer.pitch", "float")    # tilt F/B
            log.add_variable("pm.vbat", "float")             # battery

            def print_data(timestamp, data, logconf):
                z = data["stateEstimate.z"]
                r = data["stabilizer.roll"]
                p = data["stabilizer.pitch"]
                v = data["pm.vbat"]
                print(f"  height={z:.2f}m  roll={r:+5.1f}°  "
                      f"pitch={p:+5.1f}°  battery={v:.2f}V")

            cf.log.add_config(log)
            log.data_received_cb.add_callback(print_data)
            log.start()

            # ── Step 6: FLIGHT ──
            print("=" * 50)
            print(f"  TAKING OFF to {HOVER_HEIGHT}m")
            print(f"  Will hover {HOVER_TIME}s then auto-land")
            print(f"  >>> Press Ctrl+C to LAND NOW <<<")
            print("=" * 50)
            print()

            time.sleep(2)  # give yourself 2 seconds to step back!

            try:
                # MotionCommander handles takeoff, hover, and landing.
                # When you enter this block: drone takes off to HOVER_HEIGHT
                # When you exit this block: drone lands automatically
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    print(f"AIRBORNE at {HOVER_HEIGHT}m!\n")

                    # Just wait — the onboard PID holds position
                    time.sleep(HOVER_TIME)

                    print("\nHover complete — landing...")
                # ← drone auto-lands here when exiting the 'with' block

            except KeyboardInterrupt:
                # Ctrl+C pressed — MotionCommander still lands safely
                print("\n\nCtrl+C — LANDING NOW!")

            log.stop()
            print("\nFlight complete. Motors off.")

    except KeyboardInterrupt:
        print("\nAborted before takeoff.")
    except Exception as e:
        print(f"\nError: {e}")
        print("Make sure cfclient GUI is CLOSED before running this.")


if __name__ == "__main__":
    main()
