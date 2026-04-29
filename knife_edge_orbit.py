"""
Knife-Edge Orbit Controller for Crazyflie 2.1
ENEE461 Controls Lab - UMD

The drone flies a circular orbit with natural banking (knife-edge appearance).
How it works:
  - The drone commands constant forward velocity + yaw rate
  - As it yaws while moving forward, it traces a circular path around takeoff point
  - The drone automatically tilts toward center due to circular motion physics
  - Bank angle ≈ arctan(v²/(R·g))

Controls:
  - SPACEBAR: Stop orbit and land
  - ESC: Emergency stop
"""

import logging
import time
import math
from typing import Optional, Tuple

try:
    from pynput import keyboard
except ImportError:
    print("pynput not installed. Run:\n  pip install pynput")
    raise

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

URI = "radio://0/80/2M/E7E7E7E13"
ORBIT_RADIUS = 1.0
ORBIT_SPEED = 1.0
ORBIT_HEIGHT = 0.5
ORBIT_DURATION = 30
CONTROL_HZ = 20
CONTROL_DT = 1.0 / CONTROL_HZ


class KnifeEdgeOrbitController:
    def __init__(self):
        self.cf: Optional[Crazyflie] = None
        self.landing_requested = False
        self.listener = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.battery_voltage = 0.0
        self.log_data = []

    def _transform_coords(self, x: float, y: float) -> Tuple[float, float]:
        """Transform Kalman coords to drone top-down view."""
        plot_x = y
        plot_y = -x
        return plot_x, plot_y

    def setup_logging(self, cf: Crazyflie):
        """Setup logging with fully connected drone"""
        self.cf = cf
        log_config = LogConfig(name='orbit', period_in_ms=100)

        try:
            log_config.add_variable('stateEstimate.x', 'float')
            log_config.add_variable('stateEstimate.y', 'float')
            log_config.add_variable('stateEstimate.z', 'float')
            log_config.add_variable('pm.vbat', 'float')

            self.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_received)
            log_config.start()
            logger.info("Logging setup complete")
        except KeyError as e:
            logger.warning(f"Some log variables not available: {e}")

    def _log_data_received(self, timestamp, data, logconf):
        try:
            if 'stateEstimate.x' in data:
                self.current_x = data['stateEstimate.x']
            if 'stateEstimate.y' in data:
                self.current_y = data['stateEstimate.y']
            if 'stateEstimate.z' in data:
                self.current_z = data['stateEstimate.z']
            if 'pm.vbat' in data:
                self.battery_voltage = data['pm.vbat']
        except Exception as e:
            logger.error(f"Error processing data: {e}")

    def _on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.landing_requested = True
                logger.info("SPACE pressed: stopping orbit")
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

    def start_live_plot(self):
        """Initialize live position plot"""
        try:
            import matplotlib.pyplot as plt
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.ax.set_xlabel('Left ← X (m) → Right')
            self.ax.set_ylabel('Back ← Y (m) → Front')
            self.ax.set_title(f'Knife-Edge Orbit (v={ORBIT_SPEED} m/s, R={ORBIT_RADIUS} m)')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-2, 2)
            self.ax.axhline(y=0, color='k', linewidth=0.5)
            self.ax.axvline(x=0, color='k', linewidth=0.5)
            self.ax.plot(0, 0, 'r*', markersize=15, label='Home')
            circle = plt.Circle((0, 0), ORBIT_RADIUS, color='b', fill=False, linestyle='--', label=f'Orbit R={ORBIT_RADIUS}m')
            self.ax.add_patch(circle)
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

    def orbit(self, mc: MotionCommander):
        """Execute knife-edge orbit"""
        omega_deg = ORBIT_SPEED / ORBIT_RADIUS * (180.0 / math.pi)
        logger.info(f"Starting orbit: speed={ORBIT_SPEED} m/s, radius={ORBIT_RADIUS} m, yaw_rate={omega_deg:.1f} deg/s")
        logger.info(f"Expected orbit period: {2*math.pi*ORBIT_RADIUS/ORBIT_SPEED:.1f} seconds")

        self.start_live_plot()

        start_time = time.time()
        loop_count = 0

        try:
            while time.time() - start_time < ORBIT_DURATION and not self.landing_requested:
                mc.start_linear_motion(ORBIT_SPEED, 0, 0, omega_deg)

                self.log_data.append({
                    'time': time.time() - start_time,
                    'x': self.current_x,
                    'y': self.current_y,
                    'z': self.current_z,
                    'vbat': self.battery_voltage,
                })

                if loop_count % 5 == 0:
                    self.update_live_plot()

                if loop_count % (CONTROL_HZ * 2) == 0:
                    distance_from_origin = math.sqrt(self.current_x**2 + self.current_y**2)
                    bank_deg = math.degrees(math.atan(ORBIT_SPEED**2 / (ORBIT_RADIUS * 9.81)))
                    logger.info(
                        f"Time: {time.time() - start_time:.1f}s | "
                        f"Pos: ({self.current_x:+.2f}, {self.current_y:+.2f})m | "
                        f"Radius: {distance_from_origin:.2f}m | "
                        f"Alt: {self.current_z:.2f}m | "
                        f"Est. bank: {bank_deg:.1f}° | "
                        f"Batt: {self.battery_voltage:.2f}V"
                    )

                loop_count += 1
                time.sleep(CONTROL_DT)

        except KeyboardInterrupt:
            logger.info("Orbit interrupted by user")
        except Exception as e:
            logger.error(f"Error during orbit: {e}")
        finally:
            logger.info("Orbit ended")

    def plot_results(self):
        """Plot orbit results"""
        try:
            import matplotlib.pyplot as plt
            if not self.log_data:
                logger.warning("No logged data")
                return

            times = [d['time'] for d in self.log_data]
            z_pos = [d['z'] for d in self.log_data]

            x_pos = []
            y_pos = []
            for d in self.log_data:
                px, py = self._transform_coords(d['x'], d['y'])
                x_pos.append(px)
                y_pos.append(py)

            fig, axes = plt.subplots(2, 1, figsize=(12, 10))

            axes[0].plot(x_pos, y_pos, 'g-', linewidth=1, label='Flight path')
            axes[0].plot(0, 0, 'r*', markersize=15, label='Home')
            circle = plt.Circle((0, 0), ORBIT_RADIUS, color='b', fill=False, linestyle='--', label=f'Target R={ORBIT_RADIUS}m')
            axes[0].add_patch(circle)
            axes[0].set_xlabel('Left ← X (m) → Right')
            axes[0].set_ylabel('Back ← Y (m) → Front')
            axes[0].set_title('Knife-Edge Orbit Path (Top-Down View)')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            axes[0].axis('equal')

            axes[1].plot(times, z_pos, 'b-', linewidth=1, label='Altitude')
            axes[1].axhline(y=ORBIT_HEIGHT, color='r', linestyle='--', label=f'Target ({ORBIT_HEIGHT}m)', linewidth=2)
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Altitude (m)')
            axes[1].set_title('Altitude During Orbit')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig('knife_edge_orbit_results.png', dpi=150)
            logger.info("Results saved to knife_edge_orbit_results.png")
            plt.show()

        except ImportError:
            logger.warning("Matplotlib not installed - skipping plots")


def main():
    cflib.crtp.init_drivers()
    controller = KnifeEdgeOrbitController()

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

            with MotionCommander(scf, default_height=ORBIT_HEIGHT) as mc:
                logger.info(f"Airborne at {ORBIT_HEIGHT}m")
                controller.orbit(mc)

            controller.plot_results()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")


if __name__ == '__main__':
    main()
