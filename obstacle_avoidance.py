"""
Obstacle Avoidance Controller for Crazyflie 2.1
ENEE461 Controls Lab - UMD

Implements reactive obstacle avoidance using multiranger deck:
- Monitors 4 horizontal distances (front, back, left, right)
- When object detected within 0.5m on any side, moves away
- Maintains altitude while avoiding lateral obstacles
- Demonstrates sensor fusion and reactive control

Multiranger ranges:
- range.front: Forward distance
- range.back: Backward distance
- range.left: Left side distance
- range.right: Right side distance
- range.zrange: Altitude (downward)
"""

import logging
import time
from typing import Optional, Dict
from collections import defaultdict

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# TUNING PARAMETERS
# ============================================================================

# Obstacle detection threshold (meters)
OBSTACLE_DISTANCE = 0.5  # Trigger avoidance if object closer than 50cm

# Target hover altitude (meters)
TARGET_ALTITUDE = 1.0  # Higher altitude provides more safety margin

# Avoidance speeds (m/s)
AVOIDANCE_SPEED = 0.3  # Speed to move away from obstacle
HOVER_SPEED = 0.0  # Zero speed when maintaining position

# Control loop frequency
CONTROL_FREQUENCY = 10  # Hz
CONTROL_DT = 1.0 / CONTROL_FREQUENCY

# Safety parameters
MIN_ALTITUDE = 0.2  # Minimum safe altitude (don't go below 20cm)
MAX_VELOCITY = 0.5  # Maximum velocity command

# Flight duration
FLIGHT_DURATION = 30  # seconds

# ============================================================================
# Obstacle Avoidance Logic
# ============================================================================

class ObstacleAvoidanceController:
    """
    Reactive obstacle avoidance using multiranger sensor array

    Control Strategy:
    1. Continuously monitor 4 horizontal range sensors
    2. When obstacle detected within OBSTACLE_DISTANCE:
       - Identify which direction has obstacle
       - Command velocity away from that direction
       - Continue moving until obstacle clears
    3. When no obstacles:
       - Hold altitude and hover at current position
    4. Always monitor altitude and adjust thrust

    This is a simple but effective "bug algorithm" variant
    """

    def __init__(self, crazyflie_uri: str = 'radio://0/80/2M/E7E7E7E7E7'):
        """
        Initialize obstacle avoidance controller

        Args:
            crazyflie_uri: Crazyflie radio connection URI
        """
        self.uri = crazyflie_uri
        self.cf: Optional[Crazyflie] = None
        self.is_flying = False

        # Multiranger sensor readings (meters) - read as mm, convert to m
        self.ranges = {
            'front': float('inf'),
            'back': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'zrange': 0.0,
        }

        # Altitude control
        self.current_altitude = 0.0
        self.target_altitude = TARGET_ALTITUDE
        self.battery_voltage = 0.0

        # State tracking
        self.avoidance_state = 'hovering'  # hovering, avoiding_front, avoiding_back, etc.
        self.obstacle_detected = False
        self.active_obstacles = set()  # Set of currently active obstacles

        # Logging
        self.log_data = []

    def connect(self):
        """Connect to Crazyflie"""
        logger.info(f"Connecting to Crazyflie at {self.uri}...")
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._on_connected)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.connection_lost.add_callback(self._on_connection_lost)

        self.cf.open_link(self.uri)
        logger.info("Connection initiated...")

    def _on_connected(self, uri):
        """Connected callback"""
        logger.info(f"Connected to {uri}")
        self._setup_logging()

    def _on_disconnected(self, uri):
        """Disconnected callback"""
        logger.info(f"Disconnected from {uri}")

    def _on_connection_failed(self, uri, msg):
        """Connection failed callback"""
        logger.error(f"Connection failed: {msg}")

    def _on_connection_lost(self, uri, msg):
        """Connection lost callback"""
        logger.error(f"Connection lost: {msg}")

    def _setup_logging(self):
        """Setup multiranger logging"""
        if not self.cf:
            return

        # Log all range sensor data
        log_config = LogConfig(name='ranges', period_in_ms=100)

        # Add multiranger sensor variables
        try:
            log_config.add_variable('range.front', 'uint16_t')
            log_config.add_variable('range.back', 'uint16_t')
            log_config.add_variable('range.left', 'uint16_t')
            log_config.add_variable('range.right', 'uint16_t')
            log_config.add_variable('range.zrange', 'uint16_t')
            log_config.add_variable('pm.vbat', 'float')

            self.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_received)
            log_config.start()
            logger.info("Multiranger logging setup complete")
        except KeyError as e:
            logger.warning(f"Some range variables not available: {e}")

    def _log_data_received(self, timestamp, data, logconf):
        """Process incoming sensor data"""
        try:
            # Convert from mm to meters (divide by 1000)
            if 'range.front' in data:
                self.ranges['front'] = data['range.front'][0] / 1000.0

            if 'range.back' in data:
                self.ranges['back'] = data['range.back'][0] / 1000.0

            if 'range.left' in data:
                self.ranges['left'] = data['range.left'][0] / 1000.0

            if 'range.right' in data:
                self.ranges['right'] = data['range.right'][0] / 1000.0

            if 'range.zrange' in data:
                self.ranges['zrange'] = data['range.zrange'][0] / 1000.0
                self.current_altitude = self.ranges['zrange']

            if 'pm.vbat' in data:
                self.battery_voltage = data['pm.vbat'][0]

        except Exception as e:
            logger.error(f"Error processing range data: {e}")

    def detect_obstacles(self) -> Dict[str, bool]:
        """
        Check if obstacles exist in any direction

        Returns:
            Dictionary with True/False for each direction
        """
        obstacles = {
            'front': self.ranges['front'] < OBSTACLE_DISTANCE and self.ranges['front'] > 0.1,
            'back': self.ranges['back'] < OBSTACLE_DISTANCE and self.ranges['back'] > 0.1,
            'left': self.ranges['left'] < OBSTACLE_DISTANCE and self.ranges['left'] > 0.1,
            'right': self.ranges['right'] < OBSTACLE_DISTANCE and self.ranges['right'] > 0.1,
        }
        return obstacles

    def compute_avoidance_velocity(self, obstacles: Dict[str, bool]) -> tuple:
        """
        Compute velocity command based on obstacle positions

        Strategy:
        - If obstacle on left: move right
        - If obstacle on right: move left
        - If obstacle front: move backward
        - If obstacle back: move forward
        - If multiple obstacles: prioritize orthogonal movement

        Args:
            obstacles: Dictionary of obstacle detections

        Returns:
            (vx, vy) velocity commands in m/s
        """
        vx = 0.0  # Forward/backward velocity
        vy = 0.0  # Left/right velocity

        # Handle lateral obstacles (left/right) - these take priority
        if obstacles['left']:
            vy = AVOIDANCE_SPEED  # Move right (positive vy)
        elif obstacles['right']:
            vy = -AVOIDANCE_SPEED  # Move left (negative vy)

        # Handle front/back obstacles
        if obstacles['front']:
            vx = -AVOIDANCE_SPEED  # Move backward
        elif obstacles['back']:
            vx = AVOIDANCE_SPEED  # Move forward

        # If conflicting obstacles (e.g., left and front), prefer lateral movement
        if obstacles['left'] and obstacles['front']:
            vy = AVOIDANCE_SPEED
            vx = 0.0
        elif obstacles['right'] and obstacles['front']:
            vy = -AVOIDANCE_SPEED
            vx = 0.0

        return vx, vy

    def compute_altitude_thrust(self) -> int:
        """
        Compute altitude control using simple proportional control

        Returns:
            Thrust command (integer 0-65535)
        """
        # Simple P-control for altitude
        error_z = self.target_altitude - self.current_altitude
        thrust_adjustment = error_z * 1000  # Proportional gain

        # Base thrust (~32767 hovers at 1.0m for Crazyflie)
        # Adjust based on your specific drone and altitude
        base_thrust = 32767
        thrust_cmd = base_thrust + thrust_adjustment

        # Clamp to valid range
        thrust_cmd = max(0, min(65535, int(thrust_cmd)))

        return thrust_cmd

    def takeoff(self):
        """Launch drone to target altitude"""
        if not self.cf:
            logger.error("Not connected")
            return False

        logger.info("Taking off...")
        self.is_flying = True

        try:
            # Ramp thrust until altitude sensor detects we've lifted off
            for thrust in range(20000, 40000, 500):
                if self.current_altitude > 0.15:  # Detected liftoff
                    break
                self.cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.05)

            logger.info(f"Takeoff complete at {self.current_altitude:.2f}m")
            return True

        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            self.is_flying = False
            return False

    def fly_with_avoidance(self, duration: float = FLIGHT_DURATION):
        """
        Main control loop - reactive obstacle avoidance

        This implements the core control law:
        1. Read multiranger sensors
        2. Detect obstacles in 4 horizontal directions
        3. If any obstacle < 0.5m:
           - Compute avoidance velocity away from obstacle
           - Send velocity command
        4. Else:
           - Hover at current position
        5. Always control altitude to maintain target height
        """
        if not self.is_flying:
            logger.error("Drone not flying")
            return

        logger.info(f"Starting obstacle avoidance for {duration} seconds...")
        logger.info(f"Obstacle threshold: {OBSTACLE_DISTANCE}m")
        logger.info(f"Target altitude: {self.target_altitude}m")

        start_time = time.time()
        loop_count = 0
        max_range_observed = defaultdict(float)

        try:
            while time.time() - start_time < duration:
                if not self.is_flying:
                    break

                # Detect obstacles
                obstacles = self.detect_obstacles()
                self.obstacle_detected = any(obstacles.values())

                # Compute velocity command
                if self.obstacle_detected:
                    vx, vy = self.compute_avoidance_velocity(obstacles)
                    self.avoidance_state = 'avoiding'
                else:
                    vx, vy = 0.0, 0.0
                    self.avoidance_state = 'hovering'

                # Altitude control
                thrust_cmd = self.compute_altitude_thrust()

                # Send command to drone
                # Crazyflie commander: send_setpoint(roll, pitch, yaw_rate, thrust)
                # But we want velocity, so we use send_velocity_world()
                # Format: (vx, vy, vz, yaw_rate)
                try:
                    self.cf.commander.send_setpoint(vx, vy, 0, thrust_cmd)
                except:
                    # Fallback if velocity commands not available
                    self.cf.commander.send_setpoint(0, 0, 0, thrust_cmd)

                # Log data
                self.log_data.append({
                    'time': time.time() - start_time,
                    'state': self.avoidance_state,
                    'altitude': self.current_altitude,
                    'front': self.ranges['front'],
                    'back': self.ranges['back'],
                    'left': self.ranges['left'],
                    'right': self.ranges['right'],
                    'vx': vx,
                    'vy': vy,
                    'thrust': thrust_cmd,
                    'obstacles': obstacles,
                })

                # Update max ranges observed
                for direction in ['front', 'back', 'left', 'right']:
                    max_range_observed[direction] = max(
                        max_range_observed[direction],
                        self.ranges[direction]
                    )

                # Print status
                if loop_count % (CONTROL_FREQUENCY * 2) == 0:  # Every 2 seconds
                    obstacle_str = "None"
                    if obstacles['front']:
                        obstacle_str = "FRONT"
                    elif obstacles['back']:
                        obstacle_str = "BACK"
                    elif obstacles['left']:
                        obstacle_str = "LEFT"
                    elif obstacles['right']:
                        obstacle_str = "RIGHT"

                    logger.info(
                        f"Alt: {self.current_altitude:.2f}m | "
                        f"Ranges - F:{self.ranges['front']:.2f}m "
                        f"B:{self.ranges['back']:.2f}m "
                        f"L:{self.ranges['left']:.2f}m "
                        f"R:{self.ranges['right']:.2f}m | "
                        f"Obstacle: {obstacle_str} | "
                        f"Cmd: vx={vx:.2f} vy={vy:.2f} | "
                        f"Batt: {self.battery_voltage:.2f}V"
                    )

                loop_count += 1
                time.sleep(CONTROL_DT)

        except KeyboardInterrupt:
            logger.info("Avoidance flight interrupted")
        except Exception as e:
            logger.error(f"Error during avoidance flight: {e}")
            self.is_flying = False
        finally:
            logger.info("Avoidance flight ended")
            logger.info(f"Max ranges observed: {dict(max_range_observed)}")

    def land(self):
        """Land safely"""
        if not self.cf or not self.is_flying:
            return

        logger.info("Landing...")
        try:
            for thrust in range(35000, 0, -1000):
                self.cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.02)

            self.cf.commander.send_setpoint(0, 0, 0, 0)
            self.is_flying = False
            logger.info("Landing complete")
        except Exception as e:
            logger.error(f"Landing error: {e}")

    def disconnect(self):
        """Disconnect from drone"""
        if self.cf:
            self.cf.close_link()

    def plot_results(self):
        """Visualize obstacle avoidance performance"""
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

            fig, axes = plt.subplots(2, 1, figsize=(12, 8))

            # Range sensor data
            axes[0].plot(times, front, label='Front', linewidth=1)
            axes[0].plot(times, back, label='Back', linewidth=1)
            axes[0].plot(times, left, label='Left', linewidth=1)
            axes[0].plot(times, right, label='Right', linewidth=1)
            axes[0].axhline(y=OBSTACLE_DISTANCE, color='r', linestyle='--',
                           label=f'Threshold ({OBSTACLE_DISTANCE}m)', linewidth=2)
            axes[0].set_ylabel('Distance (m)')
            axes[0].set_title('Obstacle Avoidance - Multiranger Sensor Data')
            axes[0].legend(loc='upper right')
            axes[0].grid(True, alpha=0.3)
            axes[0].set_ylim(0, 2.5)

            # Altitude control
            axes[1].plot(times, altitude, 'b-', label='Measured Altitude', linewidth=1)
            axes[1].axhline(y=TARGET_ALTITUDE, color='r', linestyle='--',
                           label=f'Target ({TARGET_ALTITUDE}m)', linewidth=2)
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Altitude (m)')
            axes[1].set_title('Altitude Control During Avoidance')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig('obstacle_avoidance_results.png', dpi=150)
            logger.info("Results saved to obstacle_avoidance_results.png")
            plt.show()

        except ImportError:
            logger.warning("Matplotlib not installed - skipping plots")


def main():
    """Main execution"""
    cflib.crtp.init_drivers()

    controller = ObstacleAvoidanceController()

    try:
        # Connect
        controller.connect()
        time.sleep(2)

        # Takeoff
        if controller.takeoff():
            time.sleep(1)

            # Run avoidance
            controller.fly_with_avoidance(duration=FLIGHT_DURATION)

        # Land
        controller.land()
        time.sleep(1)

        # Plot results
        controller.plot_results()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
