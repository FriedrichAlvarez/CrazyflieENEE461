"""
Position-Hold Obstacle Avoidance Controller for Crazyflie 2.1
ENEE461 Controls Lab - UMD

Combines obstacle avoidance with return-to-home localization:
- Takes off and records starting position as home (0, 0, 0.5m)
- Uses Kalman filter estimates for X, Y position tracking
- Monitors 4 horizontal distances (front, back, left, right)
- When obstacle detected, moves away (obstacle avoidance)
- When clear of obstacles, autonomously returns to home position
- Maintains altitude at 0.5m above ground
- SPACEBAR or ESC triggers slow, gentle landing

Controls:
- SPACEBAR: Begin slow descent and landing
- ESC: Same as SPACEBAR (slow landing)
"""

import logging
import time
import math
from typing import Optional, Dict, Tuple

try:
    from pynput import keyboard
except ImportError:
    print("pynput not installed. Run:\n  pip install pynput")
    raise

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

OBSTACLE_DISTANCE = 0.5
TARGET_ALTITUDE = 0.5
AVOIDANCE_SPEED = 0.3
RETURN_SPEED = 0.2
RETURN_KP = 1.0
RETURN_DEADBAND = 0.15
CONTROL_FREQUENCY = 10
CONTROL_DT = 1.0 / CONTROL_FREQUENCY
FLIGHT_DURATION = 60
URI = "radio://0/80/2M/E7E7E7E13"


class PositionHoldAvoidanceController:
    def __init__(self):
        self.cf: Optional[Crazyflie] = None
        self.is_flying = False
        self.landing_requested = False
        self.listener = None
        self.landing_rate = 0.02
        self.landing_altitude_threshold = 0.15

        self.ranges = {
            'front': float('inf'),
            'back': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'zrange': 0.0,
        }

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_altitude = 0.0
        self.target_altitude = TARGET_ALTITUDE
        self.battery_voltage = 0.0
        self.log_data = []

    def _transform_coords(self, x: float, y: float) -> Tuple[float, float]:
        """Transform Kalman coords to drone top-down view.
        Kalman: negative X = front, positive X = back, Y = left/right
        Top-down view: positive Y = front, positive X = right
        """
        plot_x = -y
        plot_y = x
        return plot_x, plot_y

    def setup_logging(self, cf: Crazyflie):
        """Setup logging with fully connected drone"""
        self.cf = cf
        log_config = LogConfig(name='flight', period_in_ms=100)

        try:
            log_config.add_variable('range.front', 'uint16_t')
            log_config.add_variable('range.back', 'uint16_t')
            log_config.add_variable('range.left', 'uint16_t')
            log_config.add_variable('range.right', 'uint16_t')
            log_config.add_variable('range.zrange', 'uint16_t')
            log_config.add_variable('stateEstimate.x', 'float')
            log_config.add_variable('stateEstimate.y', 'float')
            log_config.add_variable('pm.vbat', 'float')

            self.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_received)
            log_config.start()
            logger.info("Logging setup complete")
        except KeyError as e:
            logger.warning(f"Some log variables not available: {e}")

    def _log_data_received(self, timestamp, data, logconf):
        try:
            if 'range.front' in data:
                self.ranges['front'] = data['range.front'] / 1000.0
            if 'range.back' in data:
                self.ranges['back'] = data['range.back'] / 1000.0
            if 'range.left' in data:
                self.ranges['left'] = data['range.left'] / 1000.0
            if 'range.right' in data:
                self.ranges['right'] = data['range.right'] / 1000.0

            if 'range.zrange' in data:
                zrange_m = data['range.zrange'] / 1000.0
                if 0.0 < zrange_m < 5.0:
                    self.ranges['zrange'] = zrange_m
                    self.current_altitude = zrange_m

            if 'stateEstimate.x' in data:
                self.current_x = data['stateEstimate.x']
            if 'stateEstimate.y' in data:
                self.current_y = data['stateEstimate.y']

            if 'pm.vbat' in data:
                self.battery_voltage = data['pm.vbat']

        except Exception as e:
            logger.error(f"Error processing data: {e}")

    def _on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.landing_requested = True
                logger.info("SPACE pressed: starting slow landing")
            elif key == keyboard.Key.esc:
                self.landing_requested = True
                logger.warning("ESC pressed: landing requested")
        except AttributeError:
            pass

    def _start_keyboard_listener(self):
        if self.listener is None:
            self.listener = keyboard.Listener(on_press=self._on_key_press)
            self.listener.daemon = True
            self.listener.start()

    def detect_obstacles(self) -> Dict[str, bool]:
        obstacles = {
            'front': self.ranges['front'] < OBSTACLE_DISTANCE and self.ranges['front'] > 0.1,
            'back': self.ranges['back'] < OBSTACLE_DISTANCE and self.ranges['back'] > 0.1,
            'left': self.ranges['left'] < OBSTACLE_DISTANCE and self.ranges['left'] > 0.1,
            'right': self.ranges['right'] < OBSTACLE_DISTANCE and self.ranges['right'] > 0.1,
        }
        return obstacles

    def compute_avoidance_velocity(self, obstacles: Dict[str, bool]) -> Tuple[float, float]:
        vx = 0.0
        vy = 0.0

        if obstacles['left']:
            vy = -AVOIDANCE_SPEED
        elif obstacles['right']:
            vy = AVOIDANCE_SPEED

        if obstacles['front']:
            vx = -AVOIDANCE_SPEED
        elif obstacles['back']:
            vx = AVOIDANCE_SPEED

        if obstacles['left'] and obstacles['front']:
            vy = -AVOIDANCE_SPEED
            vx = 0.0
        elif obstacles['right'] and obstacles['front']:
            vy = AVOIDANCE_SPEED
            vx = 0.0

        return vx, vy

    def compute_return_velocity(self) -> Tuple[float, float]:
        dx = -self.current_x
        dy = -self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > RETURN_DEADBAND:
            vx = RETURN_KP * dx
            vy = RETURN_KP * dy
            vx = max(-RETURN_SPEED, min(RETURN_SPEED, vx))
            vy = max(-RETURN_SPEED, min(RETURN_SPEED, vy))
            return vx, vy
        else:
            return 0.0, 0.0

    def takeoff(self) -> bool:
        if not self.cf:
            logger.error("Not connected")
            return False

        logger.info("Taking off...")
        self.is_flying = True

        try:
            for thrust in range(20000, 40000, 500):
                if self.current_altitude > 0.15:
                    break
                self.cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.05)

            logger.info(f"Takeoff complete at {self.current_altitude:.2f}m")
            return True

        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            self.is_flying = False
            return False

    def fly(self, duration: float = FLIGHT_DURATION):
        if not self.is_flying:
            logger.error("Drone not flying")
            return

        logger.info(f"Starting position-hold avoidance for {duration} seconds...")
        logger.info(f"Target altitude: {self.target_altitude}m, Home: (0.0, 0.0)")

        self.start_live_plot()

        start_time = time.time()
        loop_count = 0

        try:
            while time.time() - start_time < duration:
                if not self.is_flying:
                    break

                obstacles = self.detect_obstacles()
                obstacle_detected = any(obstacles.values())

                if self.landing_requested:
                    self.target_altitude = max(0.0, self.target_altitude - self.landing_rate)
                    if self.target_altitude <= self.landing_altitude_threshold:
                        logger.info("Landing altitude reached")
                        break
                    vx, vy = 0.0, 0.0
                elif obstacle_detected:
                    vx, vy = self.compute_avoidance_velocity(obstacles)
                else:
                    vx, vy = self.compute_return_velocity()

                try:
                    self.cf.commander.send_hover_setpoint(vx, vy, 0, self.target_altitude)
                except Exception as e:
                    logger.warning(f"send_hover_setpoint failed: {e}")
                    self.cf.commander.send_stop_setpoint()

                self.log_data.append({
                    'time': time.time() - start_time,
                    'x': self.current_x,
                    'y': self.current_y,
                    'altitude': self.current_altitude,
                    'front': self.ranges['front'],
                    'back': self.ranges['back'],
                    'left': self.ranges['left'],
                    'right': self.ranges['right'],
                    'vx': vx,
                    'vy': vy,
                    'obstacles': obstacle_detected,
                })

                if loop_count % 5 == 0:
                    self.update_live_plot()

                if loop_count % (CONTROL_FREQUENCY * 2) == 0:
                    distance_to_home = math.sqrt(self.current_x**2 + self.current_y**2)
                    obstacle_str = "YES" if obstacle_detected else "NO"
                    logger.info(
                        f"Alt: {self.current_altitude:.2f}m | "
                        f"Pos: ({self.current_x:+.2f}, {self.current_y:+.2f})m | "
                        f"Home dist: {distance_to_home:.2f}m | "
                        f"Obstacle: {obstacle_str} | "
                        f"Cmd: vx={vx:.2f} vy={vy:.2f}"
                    )

                loop_count += 1
                time.sleep(CONTROL_DT)

        except KeyboardInterrupt:
            logger.info("Flight interrupted by user")
        except Exception as e:
            logger.error(f"Error during flight: {e}")
            self.is_flying = False
        finally:
            logger.info("Flight ended")

    def land(self):
        if not self.cf or not self.is_flying:
            return

        logger.info("Landing - slow descent...")
        try:
            land_start = time.time()
            max_land_time = 10.0

            while self.current_altitude > self.landing_altitude_threshold and (time.time() - land_start) < max_land_time:
                self.target_altitude = max(0.0, self.target_altitude - self.landing_rate)
                self.cf.commander.send_hover_setpoint(0, 0, 0, self.target_altitude)
                time.sleep(CONTROL_DT)

            self.cf.commander.send_stop_setpoint()
            self.is_flying = False
            logger.info("Landing complete")
        except Exception as e:
            logger.error(f"Landing error: {e}")

    def disconnect(self):
        if self.cf:
            self.cf.close_link()

    def start_live_plot(self):
        """Initialize live position plot"""
        try:
            import matplotlib.pyplot as plt
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.ax.set_xlabel('Left ← X (m) → Right')
            self.ax.set_ylabel('Back ← Y (m) → Front')
            self.ax.set_title('Drone Position (Top-Down View) - Return to Origin')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_xlim(-1.5, 1.5)
            self.ax.set_ylim(-1.5, 1.5)
            self.ax.axhline(y=0, color='k', linewidth=0.5)
            self.ax.axvline(x=0, color='k', linewidth=0.5)
            self.ax.plot(0, 0, 'r*', markersize=15, label='Home')
            self.ax.legend()
            self.line, = self.ax.plot([], [], 'g-', linewidth=1, label='Flight path')
            plt.show()
            return True
        except ImportError:
            logger.warning("Matplotlib not installed - live plot unavailable")
            return False

    def update_live_plot(self):
        """Update live position plot"""
        try:
            import matplotlib.pyplot as plt
            x_pos = []
            y_pos = []
            for d in self.log_data:
                px, py = self._transform_coords(d['x'], d['y'])
                x_pos.append(px)
                y_pos.append(py)
            self.line.set_data(x_pos, y_pos)
            plot_x, plot_y = self._transform_coords(self.current_x, self.current_y)
            self.ax.plot(plot_x, plot_y, 'bo', markersize=5)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            logger.debug(f"Live plot update error: {e}")

    def plot_results(self):
        try:
            import matplotlib.pyplot as plt
            if not self.log_data:
                logger.warning("No logged data")
                return

            times = [d['time'] for d in self.log_data]
            front = [d['front'] for d in self.log_data]
            back = [d['back'] for d in self.log_data]
            left = [d['left'] for d in self.log_data]
            right = [d['right'] for d in self.log_data]
            altitude = [d['altitude'] for d in self.log_data]

            x_pos = []
            y_pos = []
            for d in self.log_data:
                px, py = self._transform_coords(d['x'], d['y'])
                x_pos.append(px)
                y_pos.append(py)

            fig, axes = plt.subplots(3, 1, figsize=(12, 10))

            axes[0].plot(times, front, label='Front', linewidth=1)
            axes[0].plot(times, back, label='Back', linewidth=1)
            axes[0].plot(times, left, label='Left', linewidth=1)
            axes[0].plot(times, right, label='Right', linewidth=1)
            axes[0].axhline(y=OBSTACLE_DISTANCE, color='r', linestyle='--',
                           label=f'Threshold ({OBSTACLE_DISTANCE}m)', linewidth=2)
            axes[0].set_ylabel('Distance (m)')
            axes[0].set_title('Multiranger Sensor Data')
            axes[0].legend(loc='upper right')
            axes[0].grid(True, alpha=0.3)
            axes[0].set_ylim(0, 2.5)

            axes[1].plot(times, altitude, 'b-', label='Altitude', linewidth=1)
            axes[1].axhline(y=TARGET_ALTITUDE, color='r', linestyle='--',
                           label=f'Target ({TARGET_ALTITUDE}m)', linewidth=2)
            axes[1].set_ylabel('Altitude (m)')
            axes[1].set_title('Altitude Control')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(x_pos, y_pos, 'g-', linewidth=1, label='Flight path')
            axes[2].plot(0, 0, 'r*', markersize=15, label='Home (0, 0)')
            axes[2].set_xlabel('Left ← X (m) → Right')
            axes[2].set_ylabel('Back ← Y (m) → Front')
            axes[2].set_title('Position Track (Top-Down View)')
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)
            axes[2].axis('equal')

            plt.tight_layout()
            plt.savefig('position_hold_avoidance_results.png', dpi=150)
            logger.info("Results saved to position_hold_avoidance_results.png")
            plt.show()

        except ImportError:
            logger.warning("Matplotlib not installed - skipping plots")


def main():
    cflib.crtp.init_drivers()
    controller = PositionHoldAvoidanceController()

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            logger.info("Connected!")
            cf = scf.cf

            time.sleep(0.5)

            logger.info("Resetting Kalman estimator...")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1.5)
            logger.info("Kalman ready")

            controller.setup_logging(cf)
            time.sleep(1)
            controller._start_keyboard_listener()
            time.sleep(0.5)

            if not controller.takeoff():
                logger.error("Takeoff failed")
                return

            controller.fly(duration=FLIGHT_DURATION)
            controller.land()
            controller.plot_results()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")


if __name__ == '__main__':
    main()
