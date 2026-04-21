"""
knife_orbit.py
─────────────────────────────────────────────────────────────
ENEE461 — Knife-Edge Orbit Demo

The drone performs a steep banked circular orbit, demonstrating
aggressive attitude control while maintaining altitude.

FLIGHT SEQUENCE:
  1. SPACEBAR → Take off to 1.0m, hover stable
  2. K        → Check sensors, enter knife-edge orbit
  3. K again  → Exit orbit, return to stable 1.0m hover
  4. L        → Slow controlled landing
  5. ESC      → Emergency stop (any time)

WHAT IS A KNIFE-EDGE ORBIT?
  The drone flies in a tight circle while rolled at a steep angle.
  In a normal turn, the drone tilts slightly. In a knife-edge turn,
  the roll angle is much steeper (30-45°), meaning a large portion
  of the thrust is directed sideways to maintain the circular path.

  Physics:
    - Centripetal force = m * v² / r (keeps drone in circle)
    - This force comes from the horizontal component of thrust
    - Horizontal thrust = Total_thrust * sin(roll_angle)
    - Vertical thrust = Total_thrust * cos(roll_angle)
    - At 45° roll, only ~71% of thrust fights gravity
    - So the drone needs MORE total thrust to maintain altitude

  The onboard PID handles the altitude compensation automatically.
  We just command the circular velocity and let the controller
  figure out the rest.

SAFETY:
  - Multi-ranger sensors check for obstacles before orbit starts
  - Orbit radius is small (~0.5m) to stay in a confined area
  - ESC kills motors instantly
  - If link is lost, drone cuts motors (same as before)

REQUIREMENTS:
  pip install pynput
  Battery must be >3.5V (fully charged preferred)

─────────────────────────────────────────────────────────────
"""

import sys
import time
import math
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Run: pip install pynput")
    sys.exit(1)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ══════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════
URI = "radio://0/80/2M/E7E7E7E13"

HOVER_HEIGHT = 1.0          # meters — takeoff and recovery height

# Orbit parameters (tune these!)
ORBIT_RADIUS = 0.5          # meters — radius of the circle
ORBIT_SPEED = 0.5           # m/s — how fast it flies around
ORBIT_DIRECTION = 1         # 1 = clockwise, -1 = counter-clockwise

# The yaw rate needed to fly in a circle of given radius at given speed:
#   yaw_rate = (speed / radius) * (180/pi) degrees per second
# For radius=0.5m, speed=0.5m/s: yaw_rate = 57.3 deg/s
ORBIT_YAW_RATE = (ORBIT_SPEED / ORBIT_RADIUS) * (180.0 / math.pi)

# Minimum safe distance from walls to enter orbit (mm)
MIN_WALL_DISTANCE = 800     # 800mm = 0.8m on each side

# Ramp time — how many seconds to transition into/out of orbit
ORBIT_RAMP_TIME = 2.0       # seconds to smoothly enter orbit


# ══════════════════════════════════════════════════════════
#  STATE
# ══════════════════════════════════════════════════════════

class FlightState:
    """Tracks the current flight phase and sensor data."""

    # Flight phases
    GROUNDED = "GROUNDED"
    HOVERING = "HOVERING"
    ENTERING_ORBIT = "ENTERING_ORBIT"
    IN_ORBIT = "IN_ORBIT"
    EXITING_ORBIT = "EXITING_ORBIT"
    LANDING = "LANDING"
    STOPPED = "STOPPED"

    def __init__(self):
        self.phase = self.GROUNDED
        self.emergency = False
        self.takeoff_requested = False
        self.orbit_toggle = False       # K pressed
        self.land_requested = False     # L pressed

        # Sensor data (updated by logging callbacks)
        self.z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vbat = 4.0

        # Multi-ranger distances (mm)
        self.range_front = 0
        self.range_back = 0
        self.range_left = 0
        self.range_right = 0
        self.range_up = 0

        self.lock = threading.Lock()


state = FlightState()


# ══════════════════════════════════════════════════════════
#  KEYBOARD HANDLER
# ══════════════════════════════════════════════════════════

def on_key_press(key):
    try:
        k = key.char.lower()
        if k == "k":
            state.orbit_toggle = True
        elif k == "l":
            state.land_requested = True
    except AttributeError:
        if key == keyboard.Key.space:
            state.takeoff_requested = True
        elif key == keyboard.Key.esc:
            state.emergency = True
            return False

def on_key_release(key):
    pass


# ══════════════════════════════════════════════════════════
#  OBSTACLE CHECK
# ══════════════════════════════════════════════════════════

def check_clearance():
    """Check multi-ranger sensors for sufficient clearance."""
    distances = {
        "front": state.range_front,
        "back": state.range_back,
        "left": state.range_left,
        "right": state.range_right,
    }

    all_clear = True
    print("\n  Checking clearance for orbit:")
    for direction, dist in distances.items():
        status = "OK" if dist > MIN_WALL_DISTANCE else "TOO CLOSE"
        if dist <= MIN_WALL_DISTANCE:
            all_clear = False
        # Sensor returns 0 or very high values if nothing detected
        # (max range ~4000mm). Treat >3500 as "clear"
        if dist == 0 or dist > 3500:
            dist_str = ">3.5m"
            status = "OK"
            all_clear = True  # no wall detected = clear
        else:
            dist_str = f"{dist/1000:.2f}m"
        print(f"    {direction:>5}: {dist_str} [{status}]")

    return all_clear


# ══════════════════════════════════════════════════════════
#  MAIN FLIGHT LOGIC
# ══════════════════════════════════════════════════════════

def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")
            cf = scf.cf

            # ── Hardware check ──
            time.sleep(1)
            flow = int(cf.param.get_value("deck.bcFlow2") or 0)
            ranger = int(cf.param.get_value("deck.bcMultiranger") or 0)
            print(f"  Flow deck v2:  {'OK' if flow else 'MISSING'}")
            print(f"  Multi-ranger:  {'OK' if ranger else 'MISSING'}")
            if not flow:
                print("  ABORT: Flow deck required.")
                sys.exit(1)
            if not ranger:
                print("  WARNING: Multi-ranger not detected.")
                print("  Obstacle checking will be skipped.\n")

            # ── Reset Kalman ──
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1.5)
            print("  Kalman estimator: READY\n")

            # ── Telemetry logging ──
            log_stab = LogConfig(name="Stab", period_in_ms=100)
            log_stab.add_variable("stateEstimate.z", "float")
            log_stab.add_variable("stateEstimate.x", "float")
            log_stab.add_variable("stateEstimate.y", "float")
            log_stab.add_variable("stabilizer.roll", "float")
            log_stab.add_variable("stabilizer.pitch", "float")
            log_stab.add_variable("stabilizer.yaw", "float")

            log_bat = LogConfig(name="Bat", period_in_ms=1000)
            log_bat.add_variable("pm.vbat", "float")

            log_range = None
            if ranger:
                log_range = LogConfig(name="Range", period_in_ms=200)
                log_range.add_variable("range.front", "uint16_t")
                log_range.add_variable("range.back", "uint16_t")
                log_range.add_variable("range.left", "uint16_t")
                log_range.add_variable("range.right", "uint16_t")
                log_range.add_variable("range.up", "uint16_t")

            def stab_cb(ts, data, lc):
                state.z = data["stateEstimate.z"]
                state.x = data["stateEstimate.x"]
                state.y = data["stateEstimate.y"]
                state.roll = data["stabilizer.roll"]
                state.pitch = data["stabilizer.pitch"]
                state.yaw = data["stabilizer.yaw"]

            def bat_cb(ts, data, lc):
                state.vbat = data["pm.vbat"]

            def range_cb(ts, data, lc):
                state.range_front = data["range.front"]
                state.range_back = data["range.back"]
                state.range_left = data["range.left"]
                state.range_right = data["range.right"]
                state.range_up = data["range.up"]

            cf.log.add_config(log_stab)
            log_stab.data_received_cb.add_callback(stab_cb)
            cf.log.add_config(log_bat)
            log_bat.data_received_cb.add_callback(bat_cb)
            if log_range:
                cf.log.add_config(log_range)
                log_range.data_received_cb.add_callback(range_cb)

            log_stab.start()
            log_bat.start()
            if log_range:
                log_range.start()

            # ── Print instructions ──
            print("=" * 55)
            print("  KNIFE-EDGE ORBIT DEMO")
            print("=" * 55)
            print(f"  Orbit radius: {ORBIT_RADIUS}m")
            print(f"  Orbit speed:  {ORBIT_SPEED} m/s")
            print(f"  Yaw rate:     {ORBIT_YAW_RATE:.1f} deg/s")
            direction_str = "clockwise" if ORBIT_DIRECTION == 1 else "counter-clockwise"
            print(f"  Direction:    {direction_str}")
            print()
            print("  SPACEBAR = Take off to 1m")
            print("  K        = Toggle knife orbit ON/OFF")
            print("  L        = Land")
            print("  ESC      = Emergency stop")
            print("=" * 55)
            print(f"\n  Battery: {state.vbat:.2f}V", end="")
            if state.vbat < 3.5:
                print(" — LOW! Charge before flying!")
            else:
                print(" — OK")
            print("\n  Press SPACEBAR when ready...\n")

            # ── Start keyboard listener ──
            listener = keyboard.Listener(
                on_press=on_key_press,
                on_release=on_key_release
            )
            listener.start()

            # ── Wait for takeoff ──
            while not state.takeoff_requested and not state.emergency:
                time.sleep(0.1)

            if state.emergency:
                listener.stop()
                log_stab.stop()
                log_bat.stop()
                if log_range:
                    log_range.stop()
                return

            # ── FLIGHT ──
            try:
                with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
                    state.phase = FlightState.HOVERING
                    print(f"\n  AIRBORNE at {HOVER_HEIGHT}m — HOVERING")
                    print("  Press K to enter knife orbit\n")

                    while not state.emergency and not state.land_requested:
                        # ── Print status ──
                        print(f"\r  [{state.phase}] "
                              f"z={state.z:.2f}m "
                              f"roll={state.roll:+5.1f}° "
                              f"pos=({state.x:+.2f},{state.y:+.2f}) "
                              f"bat={state.vbat:.2f}V  ",
                              end="", flush=True)

                        # ── Battery check ──
                        if state.vbat < 3.2:
                            print("\n\n  BATTERY CRITICAL — AUTO LANDING!")
                            break

                        # ── K pressed: toggle orbit ──
                        if state.orbit_toggle:
                            state.orbit_toggle = False

                            if state.phase == FlightState.HOVERING:
                                # ── ENTER ORBIT ──

                                # Check clearance first
                                if ranger:
                                    clear = check_clearance()
                                    if not clear:
                                        print(f"\n  BLOCKED: Too close to wall!"
                                              f" Need >{MIN_WALL_DISTANCE}mm clearance.")
                                        print("  Move drone away from walls first.\n")
                                        continue
                                    print("  Clearance: ALL CLEAR\n")

                                state.phase = FlightState.ENTERING_ORBIT
                                print(f"\n  ENTERING KNIFE ORBIT "
                                      f"(ramping over {ORBIT_RAMP_TIME}s)...")

                                # Ramp into orbit smoothly
                                ramp_start = time.time()
                                while time.time() - ramp_start < ORBIT_RAMP_TIME:
                                    if state.emergency or state.land_requested:
                                        break
                                    progress = (time.time() - ramp_start) / ORBIT_RAMP_TIME
                                    # Ease in: smoothly increase speed
                                    current_speed = ORBIT_SPEED * progress
                                    current_yaw = ORBIT_YAW_RATE * progress * ORBIT_DIRECTION

                                    # Forward velocity + yaw = circular path
                                    mc.start_linear_motion(
                                        current_speed,  # forward
                                        0.0,            # sideways
                                        0.0,            # vertical
                                        current_yaw     # yaw rate
                                    )
                                    time.sleep(0.05)

                                state.phase = FlightState.IN_ORBIT
                                print(f"\n  IN ORBIT — speed={ORBIT_SPEED}m/s "
                                      f"yaw={ORBIT_YAW_RATE:.0f}°/s")
                                print("  Press K to exit orbit\n")

                            elif state.phase == FlightState.IN_ORBIT:
                                # ── EXIT ORBIT ──
                                state.phase = FlightState.EXITING_ORBIT
                                print(f"\n  EXITING ORBIT "
                                      f"(ramping down over {ORBIT_RAMP_TIME}s)...")

                                # Ramp out smoothly
                                ramp_start = time.time()
                                while time.time() - ramp_start < ORBIT_RAMP_TIME:
                                    if state.emergency:
                                        break
                                    progress = 1.0 - (time.time() - ramp_start) / ORBIT_RAMP_TIME
                                    current_speed = ORBIT_SPEED * progress
                                    current_yaw = ORBIT_YAW_RATE * progress * ORBIT_DIRECTION

                                    mc.start_linear_motion(
                                        current_speed,
                                        0.0,
                                        0.0,
                                        current_yaw
                                    )
                                    time.sleep(0.05)

                                # Full stop — hover
                                mc.start_linear_motion(0, 0, 0, 0)
                                state.phase = FlightState.HOVERING
                                print(f"\n  HOVERING — orbit complete!")
                                print("  Press K for another orbit, L to land\n")

                        # ── Sustain orbit if in orbit phase ──
                        if state.phase == FlightState.IN_ORBIT:
                            mc.start_linear_motion(
                                ORBIT_SPEED,                        # forward
                                0.0,                                # sideways
                                0.0,                                # vertical
                                ORBIT_YAW_RATE * ORBIT_DIRECTION    # yaw
                            )

                        time.sleep(0.05)

                    # ── LANDING ──
                    if state.emergency:
                        print("\n\n  EMERGENCY STOP!")
                        cf.commander.send_stop_setpoint()
                    else:
                        # Stop any motion first
                        mc.start_linear_motion(0, 0, 0, 0)
                        time.sleep(0.5)
                        print("\n\n  LANDING...")
                    # MotionCommander auto-lands on block exit

            except KeyboardInterrupt:
                print("\n  Ctrl+C — landing!")

            # ── Cleanup ──
            listener.stop()
            log_stab.stop()
            log_bat.stop()
            if log_range:
                log_range.stop()
            print("\n  Flight complete. Motors off.")

    except Exception as e:
        print(f"\nError: {e}")
        print("Make sure cfclient GUI is CLOSED and battery is charged.")


if __name__ == "__main__":
    main()
