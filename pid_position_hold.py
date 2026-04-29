"""
PID Position Hold Controller for Crazyflie 2.1
ENEE461 Controls Lab - UMD

Demonstrates 3-axis PID control for maintaining position using:
- Multiranger deck for Z-axis (altitude) feedback
- Motion capture or flow deck for X, Y feedback (if available)

This script shows how to:
1. Tune three independent PID loops (X, Y, Z)
2. Send velocity commands to maintain desired position
3. Monitor control performance in real-time
"""

import logging
import time
import math
from typing import Optional, Tuple

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from pid_controller import PIDGains, MultiAxisPIDController

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# TUNING PARAMETERS - Adjust these for your specific Crazyflie
# ============================================================================

# PID Gains for X-axis (lateral position - m/s)
X_GAINS = PIDGains(kp=0.8, ki=0.1, kd=0.3)

# PID Gains for Y-axis (lateral position - m/s)
Y_GAINS = PIDGains(kp=0.8, ki=0.1, kd=0.3)

# PID Gains for Z-axis (altitude - thrust 0-1)
# Z control is more critical - needs careful tuning
Z_GAINS = PIDGains(kp=0.6, ki=0.05, kd=0.2)

# Target hover altitude (meters) - multiranger range ~0.3m to 5m
TARGET_ALTITUDE = 0.5  # 50cm hover height

# Flight parameters
CONTROL_FREQUENCY = 10  # Hz (update rate)
CONTROL_DT = 1.0 / CONTROL_FREQUENCY  # Time step
MAX_VELOCITY = 0.5  # Max velocity command (m/s)
FLIGHT_DURATION = 30  # How long to maintain position (seconds)

# ============================================================================
# Position Feedback Options
# ============================================================================

class FlightController:
    """
    Main flight controller integrating PID loops with Crazyflie API
    Handles communication, sensor feedback, and command dispatch
    """

    def __init__(self, crazyflie_uri: str = 'radio://0/80/2M/E7E7E7E13'):
        """
        Initialize flight controller

        Args:
            crazyflie_uri: Crazyflie radio connection URI
        """
        self.uri = crazyflie_uri
        self.cf: Optional[Crazyflie] = None
        self.is_flying = False

        # Position state
        self.current_pos = [0.0, 0.0, 0.0]  # X, Y, Z
        self.target_pos = [0.0, 0.0, TARGET_ALTITUDE]

        # Sensor feedback
        self.multiranger_z = 0.0
        self.flow_x = 0.0
        self.flow_y = 0.0
        self.battery_voltage = 0.0

        # Initialize PID controller
        self.pid_controller = MultiAxisPIDController(X_GAINS, Y_GAINS, Z_GAINS, CONTROL_DT)

        # Logging
        self.log_data = []

    def connect(self):
        """Connect to Crazyflie via radio"""
        logger.info(f"Connecting to Crazyflie at {self.uri}...")
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._on_connected)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.connection_lost.add_callback(self._on_connection_lost)

        self.cf.open_link(self.uri)
        logger.info("Connection initiated...")

    def _on_connected(self, uri):
        """Callback when connection established"""
        logger.info(f"Connected to {uri}")
        self._setup_logging()

    def _on_disconnected(self, uri):
        """Callback when disconnected"""
        logger.info(f"Disconnected from {uri}")

    def _on_connection_failed(self, uri, msg):
        """Callback on connection failure"""
        logger.error(f"Connection failed to {uri}: {msg}")

    def _on_connection_lost(self, uri, msg):
        """Callback when connection lost"""
        logger.error(f"Connection lost to {uri}: {msg}")

    def _setup_logging(self):
        """Setup real-time data logging from drone"""
        if not self.cf:
            return

        # Multiranger altitude feedback
        log_config = LogConfig(name='position', period_in_ms=100)
        log_config.add_variable('range.zrange', 'float')  # Altitude from multiranger
        log_config.add_variable('pm.vbat', 'float')  # Battery voltage

        try:
            self.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_received)
            log_config.start()
            logger.info("Logging setup complete")
        except KeyError as e:
            logger.warning(f"Could not setup logging: {e}")

    def _log_data_received(self, timestamp, data, logconf):
        """Callback when sensor data received"""
        try:
            # Extract multiranger Z (altitude)
            if 'range.zrange' in data:
                self.multiranger_z = data['range.zrange'][0] / 1000.0  # Convert mm to m
                self.current_pos[2] = self.multiranger_z

            # Extract battery voltage
            if 'pm.vbat' in data:
                self.battery_voltage = data['pm.vbat'][0]
        except Exception as e:
            logger.error(f"Error processing log data: {e}")

    def takeoff(self):
        """Launch drone and reach hover altitude"""
        if not self.cf:
            logger.error("Not connected to Crazyflie")
            return False

        logger.info("Initiating takeoff sequence...")
        self.is_flying = True

        try:
            # Ramp thrust from 0 to hover thrust
            for thrust in range(20000, 35000, 500):
                if self.multiranger_z > 0.1:  # Detected ground
                    break
                self.cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.05)

            logger.info("Takeoff complete, entering position hold mode")
            return True

        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            self.is_flying = False
            return False

    def maintain_position(self, duration: float = FLIGHT_DURATION):
        """
        Main control loop - maintains position using PID

        This demonstrates the control law:
        1. Read sensor position (Z from multiranger, X/Y from flow or estimate)
        2. Compute error: e = target - measured
        3. Update PID controllers
        4. Send velocity commands to drone
        5. Log performance
        """
        if not self.is_flying:
            logger.error("Drone not flying")
            return

        logger.info(f"Starting position hold for {duration} seconds...")
        logger.info(f"Target position: X={self.target_pos[0]}, "
                   f"Y={self.target_pos[1]}, Z={self.target_pos[2]}")

        start_time = time.time()
        loop_count = 0

        try:
            while time.time() - start_time < duration:
                if not self.is_flying:
                    break

                # Update current position (normally from motion capture or flow)
                # For now, we estimate position assuming starts at origin
                # In practice, use: self.current_pos from flow deck or Vicon

                # Compute PID control outputs
                vx, vy, thrust = self.pid_controller.update(
                    tuple(self.target_pos),
                    tuple(self.current_pos)
                )

                # Clamp velocities to max
                vx = max(-MAX_VELOCITY, min(MAX_VELOCITY, vx))
                vy = max(-MAX_VELOCITY, min(MAX_VELOCITY, vy))

                # Send control to drone
                # Format: vx (m/s), vy (m/s), yaw_rate (deg/s), thrust (0-65535)
                thrust_cmd = int(32767 + thrust * 20000)  # Convert thrust to command
                self.cf.commander.send_setpoint(vx, vy, 0, thrust_cmd)

                # Log performance
                self.log_data.append({
                    'time': time.time() - start_time,
                    'target_z': self.target_pos[2],
                    'current_z': self.current_pos[2],
                    'error_z': self.target_pos[2] - self.current_pos[2],
                    'vx': vx,
                    'vy': vy,
                    'thrust': thrust,
                })

                # Print status
                if loop_count % (CONTROL_FREQUENCY * 2) == 0:  # Print every 2 seconds
                    error_z = self.target_pos[2] - self.current_pos[2]
                    logger.info(f"Alt: {self.current_pos[2]:.2f}m | "
                               f"Error: {error_z:+.3f}m | "
                               f"Vx: {vx:+.2f} Vy: {vy:+.2f} | "
                               f"Battery: {self.battery_voltage:.2f}V")

                loop_count += 1
                time.sleep(CONTROL_DT)

        except Exception as e:
            logger.error(f"Error during position hold: {e}")
            self.is_flying = False
        finally:
            logger.info("Position hold ended")

    def land(self):
        """Land drone safely"""
        if not self.cf or not self.is_flying:
            return

        logger.info("Landing...")
        try:
            # Reduce thrust gradually
            for thrust in range(35000, 0, -1000):
                self.cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.02)

            self.cf.commander.send_setpoint(0, 0, 0, 0)
            self.is_flying = False
            logger.info("Landing complete")
        except Exception as e:
            logger.error(f"Error during landing: {e}")

    def disconnect(self):
        """Close connection to Crazyflie"""
        if self.cf:
            self.cf.close_link()

    def plot_results(self):
        """Plot control performance (requires matplotlib)"""
        try:
            import matplotlib.pyplot as plt
            if not self.log_data:
                logger.warning("No logged data to plot")
                return

            times = [d['time'] for d in self.log_data]
            target_z = [d['target_z'] for d in self.log_data]
            current_z = [d['current_z'] for d in self.log_data]
            errors = [d['error_z'] for d in self.log_data]

            fig, axes = plt.subplots(2, 1, figsize=(12, 8))

            # Altitude tracking
            axes[0].plot(times, target_z, 'r--', label='Target Altitude', linewidth=2)
            axes[0].plot(times, current_z, 'b-', label='Measured Altitude', linewidth=1)
            axes[0].set_ylabel('Altitude (m)')
            axes[0].set_title('PID Position Hold - Altitude Tracking')
            axes[0].grid(True, alpha=0.3)
            axes[0].legend()

            # Tracking error
            axes[1].plot(times, errors, 'r-', label='Tracking Error')
            axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Error (m)')
            axes[1].set_title('Altitude Tracking Error')
            axes[1].grid(True, alpha=0.3)
            axes[1].legend()

            plt.tight_layout()
            plt.savefig('pid_position_hold_results.png', dpi=150)
            logger.info("Results saved to pid_position_hold_results.png")
            plt.show()

        except ImportError:
            logger.warning("Matplotlib not installed - skipping plots")


def main():
    """Main execution"""
    # Initialize radio communication
    cflib.crtp.init_drivers()

    controller = FlightController()

    try:
        # Connect to drone
        controller.connect()
        time.sleep(2)  # Wait for connection

        # Takeoff
        if controller.takeoff():
            # Hold position
            controller.maintain_position(duration=FLIGHT_DURATION)

        # Land
        controller.land()
        time.sleep(1)

        # Visualize results
        controller.plot_results()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
