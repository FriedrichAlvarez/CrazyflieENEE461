"""
hover_1m.py
Hover the Crazyflie at 1 meter using the Flow deck v2 for height estimation.
Uses the high-level commander for position hold.

The drone will:
  1. Take off to 1.0m
  2. Hold position for 10 seconds (push it and it returns to position)
  3. Land

Requirements:
  - Flow deck v2 attached (provides optical flow + ToF height)
  - Charged battery (>3.3V)
  - Clear area, flat floor

Usage:
    python hover_1m.py

Press Ctrl+C at any time to trigger emergency landing.
"""

import time
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

# ── Configuration ──────────────────────────────────────────────
URI = "radio://0/80/2M/E7E7E7E13"
HOVER_HEIGHT = 1.0      # meters
HOVER_TIME = 10          # seconds to hold position
# ───────────────────────────────────────────────────────────────


def check_decks(scf):
    """Verify the Flow deck is detected before attempting flight."""
    print("Checking for Flow deck v2...")

    is_flow_attached = False

    try:
        # The parameter 'deck.bcFlow2' is 1 if Flow deck v2 is attached
        is_flow_attached = scf.cf.param.get_value("deck.bcFlow2")
        is_flow_attached = int(is_flow_attached) == 1
    except Exception as e:
        print(f"  Could not check deck parameter: {e}")

    if is_flow_attached:
        print("  Flow deck v2: DETECTED")
    else:
        print("  Flow deck v2: NOT DETECTED")
        print("  Cannot fly safely without height sensing. Aborting.")
        sys.exit(1)

    return True


def log_stabilizer(scf):
    """Set up logging for roll, pitch, yaw and height during flight."""
    log_stab = LogConfig(name="Stabilizer", period_in_ms=200)
    log_stab.add_variable("stabilizer.roll", "float")
    log_stab.add_variable("stabilizer.pitch", "float")
    log_stab.add_variable("stabilizer.yaw", "float")
    log_stab.add_variable("stateEstimate.z", "float")

    def stab_callback(timestamp, data, logconf):
        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]
        yaw = data["stabilizer.yaw"]
        z = data["stateEstimate.z"]
        print(f"  height={z:.2f}m  roll={roll:.1f}  pitch={pitch:.1f}  yaw={yaw:.1f}")

    scf.cf.log.add_config(log_stab)
    log_stab.data_received_cb.add_callback(stab_callback)
    log_stab.start()
    return log_stab


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")

            # Safety check: is the Flow deck present?
            check_decks(scf)

            # Start logging attitude + height
            log_stab = log_stabilizer(scf)

            print(f"\n{'='*50}")
            print(f"  TAKING OFF to {HOVER_HEIGHT}m")
            print(f"  Will hover for {HOVER_TIME}s then land")
            print(f"  Press Ctrl+C for emergency landing")
            print(f"{'='*50}\n")

            time.sleep(2)  # Give yourself a moment to step back

            try:
                # MotionCommander handles takeoff, hover, and landing
                # It uses the Flow deck's height estimate for altitude hold
                # The internal PID controller maintains position — if you
                # push the drone, it will return to its hover point
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    print(f"Airborne! Hovering at {HOVER_HEIGHT}m...")
                    print("Try gently pushing the drone — it should return.\n")

                    # Hold position for HOVER_TIME seconds
                    time.sleep(HOVER_TIME)

                    print("\nLanding...")
                # MotionCommander lands automatically when exiting the block

            except KeyboardInterrupt:
                print("\n\nCtrl+C detected — landing immediately!")

            log_stab.stop()
            print("\nDone! Drone has landed.")

    except KeyboardInterrupt:
        print("\nAborted before takeoff.")
    except Exception as e:
        print(f"\nError: {e}")
        print("Make sure the Crazyflie is on and the dongle is plugged in.")


if __name__ == "__main__":
    main()
