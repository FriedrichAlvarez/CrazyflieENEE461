"""
custom_pid_hover.py
─────────────────────────────────────────────────────────────
ENEE461 Controls Lab — Custom PID Position Hold Controller

THIS IS THE SCRIPT YOU BUILD AND TUNE YOURSELF.

Instead of using MotionCommander (which hides the control logic),
this script reads raw sensor data and sends low-level commands
directly. You implement the PID loops yourself.

Architecture (YOUR code, running on your Mac, sent over radio):

  ┌──────────────────────────────────────────────────────────┐
  │                YOUR CONTROL LOOP (Python)                │
  │                                                          │
  │  Sensor readings (from Kalman estimator on drone):       │
  │    x, y, z        — estimated position (meters)          │
  │    vx, vy, vz     — estimated velocity (m/s)             │
  │    roll, pitch, yaw — current attitude (degrees)         │
  │                                                          │
  │  Your PID controllers compute:                           │
  │    1. Z PID:  z_error → thrust adjustment                │
  │    2. X PID:  x_error → desired pitch                    │
  │    3. Y PID:  y_error → desired roll                     │
  │    4. Yaw PID: yaw_error → desired yaw_rate              │
  │                                                          │
  │  Commands sent to drone:                                 │
  │    commander.send_setpoint(roll, pitch, yaw_rate, thrust)│
  │                                                          │
  │  The ONBOARD attitude controller still handles:          │
  │    desired roll/pitch/yaw → motor PWM                    │
  │    (you're replacing the outer position loop only)       │
  └──────────────────────────────────────────────────────────┘

  Note on coordinate frame:
    - X axis: forward (positive = front of drone)
    - Y axis: left (positive = left side of drone)
    - Z axis: up (positive = higher altitude)
    - Roll:  positive = tilt RIGHT → drone moves RIGHT (-Y)
    - Pitch: positive = tilt BACKWARD → drone moves BACKWARD (-X)
    So: to move +X (forward), command NEGATIVE pitch
        to move +Y (left), command NEGATIVE roll

WHAT TO TUNE:
  The PID gains below are starting points. Your job is to tune
  them for best performance on YOUR drone. The tuning process:

  1. Start with Z (altitude) PID — get stable hover height
  2. Then tune X/Y (position) PID — get stable position hold
  3. Finally tune Yaw PID — keep heading stable
  4. Test disturbance rejection — push drone, measure recovery

  For each axis, the tuning approach:
  - Start with P only (I=0, D=0), increase until it oscillates
  - Back off P by ~30%, then add D to dampen oscillations
  - Add small I to eliminate steady-state error
  - If it overshoots: increase D or decrease P
  - If it's sluggish: increase P
  - If it drifts: increase I (but too much I causes windup)

Usage:
    python custom_pid_hover.py

Press Ctrl+C to trigger emergency stop.
─────────────────────────────────────────────────────────────
"""

import csv
import sys
import time
import threading
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# ── Radio URI ──────────────────────────────────────────────
URI = "radio://0/100/2M"    # ← Change to your radio URI
# ───────────────────────────────────────────────────────────

# ── Flight parameters ──────────────────────────────────────
TARGET_HEIGHT = 1.0         # meters — desired hover altitude
TARGET_X = 0.0              # meters — desired X position (origin)
TARGET_Y = 0.0              # meters — desired Y position (origin)
TARGET_YAW = 0.0            # degrees — desired heading

HOVER_TIME = 30             # seconds to hover
CONTROL_RATE = 0.02         # seconds between control updates (50 Hz)

LOG_TO_CSV = True
# ───────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════
#  PID CONTROLLER CLASS
#  This is the core of your controls project. Study this
#  carefully — it implements the textbook PID algorithm.
# ══════════════════════════════════════════════════════════

class PIDController:
    """
    Standard PID controller with:
      - Proportional: reacts to current error
      - Integral: accumulates past error (eliminates steady-state offset)
      - Derivative: reacts to rate of change (dampens oscillation)
      - Anti-windup: clamps the integral term to prevent saturation
      - Output clamping: limits the output to safe range

    Transfer function (continuous):
        u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt

    Discrete implementation (what we use):
        u[k] = Kp * e[k] + Ki * Σ(e[i]*dt) + Kd * (e[k] - e[k-1]) / dt
    """

    def __init__(self, kp, ki, kd, output_min, output_max,
                 integral_max=None, name="PID"):
        # ── Gains (TUNE THESE) ──
        self.kp = kp    # Proportional gain
        self.ki = ki    # Integral gain
        self.kd = kd    # Derivative gain

        # ── Output limits ──
        self.output_min = output_min
        self.output_max = output_max

        # ── Anti-windup: limit on integral accumulator ──
        # Without this, the integral term can grow huge when the
        # drone is stuck (e.g., on the ground) and cause a massive
        # overshoot when it finally moves. This is called "windup."
        self.integral_max = integral_max or abs(output_max) * 0.5

        # ── Internal state ──
        self.integral = 0.0     # accumulated error × time
        self.prev_error = 0.0   # error from last update
        self.prev_time = None   # timestamp of last update
        self.name = name

        # ── For logging/debugging ──
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0

    def update(self, error, current_time=None):
        """
        Compute PID output given the current error.

        Parameters:
            error: setpoint - measured_value
                   (positive error = we are BELOW target)
            current_time: timestamp in seconds (uses time.time() if None)

        Returns:
            Control output (clamped to [output_min, output_max])
        """
        now = current_time or time.time()

        if self.prev_time is None:
            # First call — can't compute derivative yet
            self.prev_time = now
            self.prev_error = error
            # Return proportional-only on first step
            output = self.kp * error
            self.last_p = self.kp * error
            self.last_output = max(self.output_min,
                                   min(self.output_max, output))
            return self.last_output

        # ── Time step ──
        dt = now - self.prev_time
        if dt <= 0:
            return self.last_output
        self.prev_time = now

        # ── Proportional term ──
        # Reacts to the CURRENT error magnitude
        # Larger Kp = stronger reaction, but can cause oscillation
        p_term = self.kp * error

        # ── Integral term ──
        # Accumulates error over time to eliminate steady-state offset
        # Example: if drone consistently hovers 5cm too low, the
        # integral builds up and adds more thrust to correct it
        self.integral += error * dt

        # Anti-windup clamp: prevent integral from growing unbounded
        self.integral = max(-self.integral_max,
                           min(self.integral_max, self.integral))

        i_term = self.ki * self.integral

        # ── Derivative term ──
        # Reacts to how FAST the error is changing
        # Acts as a "brake" — if error is decreasing quickly,
        # the D term reduces the output to prevent overshoot
        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        # ── Sum and clamp output ──
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        # Save for debugging
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_output = output

        return output

    def reset(self):
        """Reset integral and derivative state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


# ══════════════════════════════════════════════════════════
#  PID GAIN TUNING SECTION
#
#  *** THIS IS WHERE YOU DO YOUR CONTROLS WORK ***
#
#  Start with the defaults below, then tune iteratively.
#  Keep notes on what you changed and why — good for your
#  lab report.
#
#  Tuning order:
#    1. Z (altitude) — most critical, tune first
#    2. X/Y (position) — tune together, they're symmetric
#    3. Yaw — usually needs minimal tuning
#
#  The Crazyflie 2.1 weighs ~27g. Thrust is 0-65535 (uint16).
#  Hover thrust is roughly 35000-42000 depending on battery.
# ══════════════════════════════════════════════════════════

# ── Altitude (Z) PID ──────────────────────────────────────
# Input:  height error in meters (positive = too low)
# Output: thrust adjustment added to base hover thrust
#
# TUNE THESE:
Z_KP = 11000    # How aggressively to correct altitude error
                 # Start here. If it bobs up/down, reduce.
                 # If it's slow to reach 1m, increase.

Z_KI = 3500     # Eliminates steady-state altitude offset
                 # If it hovers 5cm too low consistently, increase.
                 # If it oscillates slowly around target, decrease.

Z_KD = 9000     # Dampens altitude oscillations
                 # If it overshoots target height, increase.
                 # If altitude response is too sluggish, decrease.

BASE_THRUST = 38000  # Approximate hover thrust (battery dependent)
                      # This is the "feedforward" term — roughly the
                      # thrust needed to hover with no correction.
                      # If drone doesn't take off: increase.
                      # If drone rockets up: decrease.
                      # IMPORTANT: adjust this as battery drains.

# ── Position X (forward/back) PID ─────────────────────────
# Input:  x position error in meters
# Output: desired pitch angle in degrees
#
# Remember: NEGATIVE pitch = forward movement = +X direction
# So we negate the output before sending to the drone.
#
# TUNE THESE:
X_KP = 25.0     # Degrees of pitch per meter of error
                 # 25 means: if 1m off in X, pitch 25 degrees
                 # If it's slow to return after push, increase.
                 # If it oscillates back and forth, decrease.

X_KI = 1.0      # Eliminates steady-state position drift
                 # If it consistently drifts in X, increase.
                 # Keep this small to avoid integral windup.

X_KD = 15.0     # Dampens position oscillations
                 # If it overshoots when returning, increase.
                 # If position response feels sluggish, decrease.

# ── Position Y (left/right) PID ───────────────────────────
# Input:  y position error in meters
# Output: desired roll angle in degrees
#
# Remember: NEGATIVE roll = left tilt = +Y direction
# So we negate the output before sending to the drone.
#
# TUNE THESE (usually same as X gains for symmetric response):
Y_KP = 25.0
Y_KI = 1.0
Y_KD = 15.0

# ── Yaw PID ──────────────────────────────────────────────
# Input:  yaw error in degrees
# Output: yaw rate in degrees/second
#
# TUNE THESE:
YAW_KP = 6.0
YAW_KI = 1.0
YAW_KD = 0.35

# ── Safety limits ─────────────────────────────────────────
MAX_THRUST = 55000      # Never exceed this (prevents flyaway)
MIN_THRUST = 20000      # Below this the drone will fall
MAX_ROLL = 20.0         # Max degrees of roll (prevents flips)
MAX_PITCH = 20.0        # Max degrees of pitch
MAX_YAW_RATE = 120.0    # Max degrees/second yaw rate


# ══════════════════════════════════════════════════════════
#  SENSOR STATE — updated by logging callbacks
# ══════════════════════════════════════════════════════════

class DroneState:
    """Holds the latest sensor readings from the drone."""
    def __init__(self):
        self.x = 0.0       # meters, from Kalman estimator
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0      # m/s
        self.vy = 0.0
        self.roll = 0.0    # degrees
        self.pitch = 0.0
        self.yaw = 0.0
        self.vbat = 4.0
        self.lock = threading.Lock()

    def update_stab(self, roll, pitch, yaw, z):
        with self.lock:
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.z = z

    def update_pos(self, x, y, vx, vy):
        with self.lock:
            self.x = x
            self.y = y
            self.vx = vx
            self.vy = vy

    def get(self):
        with self.lock:
            return {
                "x": self.x, "y": self.y, "z": self.z,
                "vx": self.vx, "vy": self.vy,
                "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
                "vbat": self.vbat,
            }


# ══════════════════════════════════════════════════════════
#  MAIN CONTROL LOOP
# ══════════════════════════════════════════════════════════

def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = f"custom_pid_{ts}.csv"
    flight_data = []

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!\n")
            cf = scf.cf

            # ── Check hardware ──
            time.sleep(1)
            flow = int(cf.param.get_value("deck.bcFlow2") or 0)
            ranger = int(cf.param.get_value("deck.bcMultiranger") or 0)
            print(f"  Flow deck v2:  {'OK' if flow else 'MISSING'}")
            print(f"  Multi-ranger:  {'OK' if ranger else 'MISSING'}")
            if not flow:
                print("  ABORT: Flow deck required.")
                sys.exit(1)

            # ── Reset Kalman estimator ──
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(2)  # let estimator converge
            print("  Kalman estimator reset and converged.\n")

            # ── Set up sensor logging ──
            state = DroneState()

            log_stab = LogConfig(name="Stab", period_in_ms=20)  # 50 Hz
            log_stab.add_variable("stabilizer.roll", "float")
            log_stab.add_variable("stabilizer.pitch", "float")
            log_stab.add_variable("stabilizer.yaw", "float")
            log_stab.add_variable("stateEstimate.z", "float")

            log_pos = LogConfig(name="Pos", period_in_ms=20)    # 50 Hz
            log_pos.add_variable("stateEstimate.x", "float")
            log_pos.add_variable("stateEstimate.y", "float")
            log_pos.add_variable("stateEstimate.vx", "float")
            log_pos.add_variable("stateEstimate.vy", "float")

            def stab_cb(ts, data, lc):
                state.update_stab(
                    data["stabilizer.roll"],
                    data["stabilizer.pitch"],
                    data["stabilizer.yaw"],
                    data["stateEstimate.z"])

            def pos_cb(ts, data, lc):
                state.update_pos(
                    data["stateEstimate.x"],
                    data["stateEstimate.y"],
                    data["stateEstimate.vx"],
                    data["stateEstimate.vy"])

            cf.log.add_config(log_stab)
            log_stab.data_received_cb.add_callback(stab_cb)
            cf.log.add_config(log_pos)
            log_pos.data_received_cb.add_callback(pos_cb)

            log_stab.start()
            log_pos.start()

            # ── Create PID controllers ──
            pid_z = PIDController(
                kp=Z_KP, ki=Z_KI, kd=Z_KD,
                output_min=MIN_THRUST - BASE_THRUST,
                output_max=MAX_THRUST - BASE_THRUST,
                name="Z"
            )
            pid_x = PIDController(
                kp=X_KP, ki=X_KI, kd=X_KD,
                output_min=-MAX_PITCH, output_max=MAX_PITCH,
                name="X"
            )
            pid_y = PIDController(
                kp=Y_KP, ki=Y_KI, kd=Y_KD,
                output_min=-MAX_ROLL, output_max=MAX_ROLL,
                name="Y"
            )
            pid_yaw = PIDController(
                kp=YAW_KP, ki=YAW_KI, kd=YAW_KD,
                output_min=-MAX_YAW_RATE, output_max=MAX_YAW_RATE,
                name="Yaw"
            )

            print(f"{'='*60}")
            print(f"  CUSTOM PID CONTROLLER")
            print(f"  Target: x={TARGET_X}, y={TARGET_Y}, "
                  f"z={TARGET_HEIGHT}m, yaw={TARGET_YAW}°")
            print(f"  Duration: {HOVER_TIME}s  |  Rate: {1/CONTROL_RATE:.0f} Hz")
            print(f"  Base thrust: {BASE_THRUST}")
            print(f"")
            print(f"  PID Gains:")
            print(f"    Z:   Kp={Z_KP}  Ki={Z_KI}  Kd={Z_KD}")
            print(f"    X:   Kp={X_KP}  Ki={X_KI}  Kd={X_KD}")
            print(f"    Y:   Kp={Y_KP}  Ki={Y_KI}  Kd={Y_KD}")
            print(f"    Yaw: Kp={YAW_KP}  Ki={YAW_KI}  Kd={YAW_KD}")
            print(f"")
            print(f"  Press Ctrl+C to EMERGENCY STOP")
            print(f"{'='*60}\n")

            time.sleep(3)  # step back from drone

            # ══════════════════════════════════════════════
            #  THE CONTROL LOOP
            #  This runs at ~50 Hz on your Mac and sends
            #  commands to the drone over radio.
            # ══════════════════════════════════════════════

            t0 = time.time()
            running = True

            try:
                # ── Takeoff ramp ──
                # Gradually increase thrust to avoid sudden jump
                print("  Ramping up thrust for takeoff...")
                ramp_time = 1.5  # seconds to ramp from 0 to hover thrust
                ramp_start = time.time()
                while time.time() - ramp_start < ramp_time:
                    progress = (time.time() - ramp_start) / ramp_time
                    ramp_thrust = int(BASE_THRUST * progress)
                    cf.commander.send_setpoint(0, 0, 0, ramp_thrust)
                    time.sleep(CONTROL_RATE)

                print("  Ramp complete — PID control ACTIVE\n")

                # ── Main control loop ──
                while time.time() - t0 < HOVER_TIME + ramp_time:
                    loop_start = time.time()
                    now = time.time()

                    # Read current state
                    s = state.get()

                    # ── Compute errors ──
                    z_error = TARGET_HEIGHT - s["z"]    # positive = too low
                    x_error = TARGET_X - s["x"]         # positive = behind target
                    y_error = TARGET_Y - s["y"]         # positive = right of target
                    yaw_error = TARGET_YAW - s["yaw"]   # positive = rotated CW

                    # Wrap yaw error to [-180, 180]
                    while yaw_error > 180:
                        yaw_error -= 360
                    while yaw_error < -180:
                        yaw_error += 360

                    # ── Run PID controllers ──
                    thrust_adj = pid_z.update(z_error, now)
                    pitch_cmd = pid_x.update(x_error, now)
                    roll_cmd = pid_y.update(y_error, now)
                    yaw_rate_cmd = pid_yaw.update(yaw_error, now)

                    # ── Apply coordinate frame corrections ──
                    # Negative pitch = forward (+X), so negate
                    # Negative roll = left (+Y), so negate
                    pitch_cmd = -pitch_cmd
                    roll_cmd = -roll_cmd

                    # ── Compute total thrust ──
                    thrust = int(BASE_THRUST + thrust_adj)
                    thrust = max(MIN_THRUST, min(MAX_THRUST, thrust))

                    # ── Send command to drone ──
                    # send_setpoint(roll_deg, pitch_deg, yaw_rate_deg/s, thrust_uint16)
                    cf.commander.send_setpoint(
                        roll_cmd,       # roll in degrees
                        pitch_cmd,      # pitch in degrees
                        yaw_rate_cmd,   # yaw rate in deg/s
                        thrust          # thrust 0-65535
                    )

                    # ── Log data ──
                    t = now - t0
                    row = {
                        "time": round(t, 3),
                        "controller": "custom",
                        "x": round(s["x"], 4),
                        "y": round(s["y"], 4),
                        "z": round(s["z"], 4),
                        "vx": round(s["vx"], 4),
                        "vy": round(s["vy"], 4),
                        "roll": round(s["roll"], 2),
                        "pitch": round(s["pitch"], 2),
                        "yaw": round(s["yaw"], 2),
                        "z_err": round(z_error, 4),
                        "x_err": round(x_error, 4),
                        "y_err": round(y_error, 4),
                        "thrust": thrust,
                        "pitch_cmd": round(pitch_cmd, 2),
                        "roll_cmd": round(roll_cmd, 2),
                        "pid_z_p": round(pid_z.last_p, 1),
                        "pid_z_i": round(pid_z.last_i, 1),
                        "pid_z_d": round(pid_z.last_d, 1),
                    }
                    flight_data.append(row)

                    # ── Print status ──
                    if len(flight_data) % 10 == 0:  # every 10th frame
                        print(f"\r  t={t:5.1f}s  "
                              f"z={s['z']:.2f}m(e={z_error:+.2f})  "
                              f"pos=({s['x']:+.2f},{s['y']:+.2f})  "
                              f"thrust={thrust}  "
                              f"r={roll_cmd:+5.1f} p={pitch_cmd:+5.1f}",
                              end="", flush=True)

                    # ── Maintain loop rate ──
                    elapsed = time.time() - loop_start
                    sleep_time = CONTROL_RATE - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                # ── Landing: ramp down thrust ──
                print("\n\n  Landing — ramping down thrust...")
                land_start = time.time()
                land_time = 2.0
                current_thrust = thrust
                while time.time() - land_start < land_time:
                    progress = (time.time() - land_start) / land_time
                    land_thrust = int(current_thrust * (1 - progress))
                    cf.commander.send_setpoint(0, 0, 0, max(0, land_thrust))
                    time.sleep(CONTROL_RATE)

            except KeyboardInterrupt:
                print("\n\n  EMERGENCY STOP!")

            # ── Stop motors ──
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            cf.commander.send_stop_setpoint()

            # ── Cleanup ──
            log_stab.stop()
            log_pos.stop()

            # ── Save CSV ──
            if LOG_TO_CSV and flight_data:
                fields = list(flight_data[0].keys())
                with open(csv_file, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=fields)
                    w.writeheader()
                    w.writerows(flight_data)
                print(f"  Saved {len(flight_data)} rows → {csv_file}")

            print("  Done.")

    except Exception as e:
        print(f"\nError: {e}")
        if LOG_TO_CSV and flight_data:
            fields = list(flight_data[0].keys())
            with open(csv_file, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                w.writerows(flight_data)
            print(f"  Saved {len(flight_data)} rows → {csv_file}")


if __name__ == "__main__":
    main()
