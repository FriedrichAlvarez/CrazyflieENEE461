"""
keyboard_fly.py
ENEE461 — Keyboard-Controlled Flight

Control the Crazyflie with keyboard

CONTROLS:
  ┌─────────────────────────────────────────────────┐
  │  SPACEBAR    = Take off / Land (toggle)         │
  │  ESC         = EMERGENCY STOP (kills motors)    │
  │                                                 │
  │  Movement (while flying):                       │
  │       W                    ↑                    │
  │     A   D  = move        ← → = yaw (rotate)    │
  │       S                    ↓                    │
  │                                                 │
  │  W = forward    S = backward                    │
  │  A = left       D = right                       │
  │  ↑ = go higher  ↓ = go lower                   │
  │  ← = rotate left  → = rotate right             │
  │                                                 │
  │  Release all keys = hover in place              │
  └─────────────────────────────────────────────────┘

HOW IT WORKS:
  This script uses MotionCommander which sends velocity commands.
  When you press W, it tells the drone "move forward at 0.3 m/s".
  When you release, velocity goes to 0 and the drone holds position.
  The onboard PID does the actual stabilization.

REQUIREMENTS:
  pip install pynput

Usage:
  1. Close cfclient GUI
  2. python keyboard_fly.py
  3. Press SPACEBAR to take off
  4. Use WASD + arrows to fly
  5. Press SPACEBAR to land, or ESC to emergency stop


"""

import sys
import time
import threading

try:
    from pynput import keyboard
except ImportError:
    print("pynput not installed. Run:")
    print("  pip install pynput")
    print("\nThen on macOS, you may need to grant Terminal/VS Code")
    print("accessibility permissions in System Settings → Privacy & Security → Accessibility")
    sys.exit(1)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander


#  CONFIGURATION
URI = "radio://0/80/2M/E7E7E7E13"

DEFAULT_HEIGHT = 0.5    # takeoff height in meters
VELOCITY = 0.3          # movement speed in m/s (start slow!)
YAW_RATE = 60           # rotation speed in degrees/s
HEIGHT_STEP = 0.1       # how much ↑/↓ changes height per press cycle



#  STATE TRACKING

class FlightState:
    def __init__(self):
        self.is_flying = False
        self.emergency = False
        self.forward = 0.0      # m/s (+forward, -backward)
        self.right = 0.0        # m/s (+right, -left)
        self.up = 0.0           # m/s (+up, -down)
        self.yaw = 0.0          # deg/s (+CW, -CCW)
        self.height = DEFAULT_HEIGHT
        self.keys_pressed = set()
        self.lock = threading.Lock()

    def update_velocities(self):
        """Convert currently pressed keys into velocity commands."""
        with self.lock:
            self.forward = 0.0
            self.right = 0.0
            self.up = 0.0
            self.yaw = 0.0

            # WASD — movement
            if "w" in self.keys_pressed:
                self.forward = VELOCITY
            if "s" in self.keys_pressed:
                self.forward = -VELOCITY
            if "a" in self.keys_pressed:
                self.right = -VELOCITY   # left
            if "d" in self.keys_pressed:
                self.right = VELOCITY    # right

            # Arrow keys — altitude and yaw
            if "up" in self.keys_pressed:
                self.up = 0.2  # rise
            if "down" in self.keys_pressed:
                self.up = -0.2  # descend
            if "left" in self.keys_pressed:
                self.yaw = -YAW_RATE  # rotate left
            if "right" in self.keys_pressed:
                self.yaw = YAW_RATE   # rotate right


state = FlightState()





#  KEYBOARD HANDLERS

def on_key_press(key):
    """Called when a key is pressed."""
    try:
        # Regular keys (WASD)
        k = key.char.lower()
        if k in ("w", "a", "s", "d"):
            state.keys_pressed.add(k)
            state.update_velocities()
    except AttributeError:
        # Special keys (arrows, space, esc)
        if key == keyboard.Key.up:
            state.keys_pressed.add("up")
        elif key == keyboard.Key.down:
            state.keys_pressed.add("down")
        elif key == keyboard.Key.left:
            state.keys_pressed.add("left")
        elif key == keyboard.Key.right:
            state.keys_pressed.add("right")
        elif key == keyboard.Key.space:
            state.is_flying = not state.is_flying
            if state.is_flying:
                print("\n  >>> SPACEBAR: TAKING OFF <<<")
            else:
                print("\n  >>> SPACEBAR: LANDING <<<")
        elif key == keyboard.Key.esc:
            state.emergency = True
            print("\n  >>> ESC: EMERGENCY STOP <<<")
            return False  # stop listener

        state.update_velocities()


def on_key_release(key):
    """Called when a key is released."""
    try:
        k = key.char.lower()
        state.keys_pressed.discard(k)
    except AttributeError:
        if key == keyboard.Key.up:
            state.keys_pressed.discard("up")
        elif key == keyboard.Key.down:
            state.keys_pressed.discard("down")
        elif key == keyboard.Key.left:
            state.keys_pressed.discard("left")
        elif key == keyboard.Key.right:
            state.keys_pressed.discard("right")

    state.update_velocities()





#  MAIN

def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")
            cf = scf.cf

            # Check Flow deck
            time.sleep(1)
            flow = int(cf.param.get_value("deck.bcFlow2") or 0)
            if not flow:
                print("ERROR: Flow deck not detected!")
                sys.exit(1)
            print("Flow deck: OK")

            # Reset Kalman
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1.5)
            print("Kalman estimator: READY\n")

            # Set up telemetry
            log = LogConfig(name="Flight", period_in_ms=500)
            log.add_variable("stateEstimate.z", "float")
            log.add_variable("stateEstimate.x", "float")
            log.add_variable("stateEstimate.y", "float")
            log.add_variable("pm.vbat", "float")

            def telemetry_cb(timestamp, data, logconf):
                z = data["stateEstimate.z"]
                x = data["stateEstimate.x"]
                y = data["stateEstimate.y"]
                v = data["pm.vbat"]
                fly_str = "FLYING" if state.is_flying else "GROUNDED"
                print(f"\r  [{fly_str}] height={z:.2f}m  "
                      f"pos=({x:+.2f},{y:+.2f})  "
                      f"bat={v:.2f}V  "
                      f"vel=({state.forward:+.1f},{state.right:+.1f})  ",
                      end="", flush=True)

            cf.log.add_config(log)
            log.data_received_cb.add_callback(telemetry_cb)
            log.start()

            # Print controls
            print("=" * 55)
            print("  KEYBOARD FLIGHT CONTROLLER")
            print("=" * 55)
            print("  SPACEBAR  = Take off / Land")
            print("  ESC       = Emergency stop")
            print("  W/A/S/D   = Forward/Left/Back/Right")
            print("  ↑/↓       = Higher/Lower")
            print("  ←/→       = Rotate left/right")
            print("  Release all = Hover in place")
            print("=" * 55)
            print("\n  Press SPACEBAR when ready to take off...\n")

            # Start keyboard listener in background
            listener = keyboard.Listener(
                on_press=on_key_press,
                on_release=on_key_release
            )
            listener.start()

            # Wait for spacebar to start flying
            while not state.is_flying and not state.emergency:
                time.sleep(0.1)

            if state.emergency:
                print("Aborted.")
                listener.stop()
                log.stop()
                return

            # ── FLIGHT LOOP ──
            try:
                with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
                    print(f"\n  AIRBORNE at {DEFAULT_HEIGHT}m!")
                    print("  Use WASD + arrows. SPACE to land. ESC to stop.\n")

                    while state.is_flying and not state.emergency:
                        # Send velocity commands based on pressed keys
                        mc.start_linear_motion(
                            state.forward,   # forward/backward m/s
                            -state.right,    # left/right m/s (inverted)
                            state.up,        # up/down m/s
                            state.yaw        # yaw rate deg/s
                        )
                        time.sleep(0.05)  # 20 Hz command rate

                    if state.emergency:
                        print("\n  EMERGENCY STOP — cutting motors!")
                        cf.commander.send_stop_setpoint()
                    else:
                        print("\n  Landing...")
                    # MotionCommander auto-lands on exit

            except KeyboardInterrupt:
                print("\n  Ctrl+C — landing!")

            listener.stop()
            log.stop()
            print("\n  Flight complete. Motors off.")

    except Exception as e:
        print(f"\nError: {e}")
        print("Make sure cfclient GUI is CLOSED.")


if __name__ == "__main__":
    main()
