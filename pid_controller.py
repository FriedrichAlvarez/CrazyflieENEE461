"""
PID Controller Module for Crazyflie Drone
ENEE461 Controls Lab - UMD
Implements standard PID control for 3-axis position hold
"""

import time
from dataclasses import dataclass
from typing import Tuple


@dataclass
class PIDGains:
    """Stores proportional, integral, derivative gains"""
    kp: float
    ki: float
    kd: float


class PIDController:
    """
    Single-axis PID controller for drone control

    Implements the standard PID equation:
    u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

    where:
    - e(t) is the error (setpoint - actual)
    - u(t) is the control output
    """

    def __init__(self, gains: PIDGains, dt: float = 0.01,
                 output_limits: Tuple[float, float] = (-1.0, 1.0),
                 integral_limits: Tuple[float, float] = (-0.5, 0.5)):
        """
        Initialize PID controller

        Args:
            gains: PIDGains object with Kp, Ki, Kd tuning parameters
            dt: Time step (seconds) - must match control loop frequency
            output_limits: Min/max clipping for control output
            integral_limits: Anti-windup clipping for integral term
        """
        self.gains = gains
        self.dt = dt
        self.output_limits = output_limits
        self.integral_limits = integral_limits

        # Error tracking
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = time.time()

    def update(self, setpoint: float, measurement: float) -> float:
        """
        Compute PID control output

        Args:
            setpoint: Desired position/velocity
            measurement: Current position/velocity from sensor

        Returns:
            Control signal clipped to output_limits
        """
        # Calculate error
        error = setpoint - measurement

        # Proportional term: P = Kp * e
        p_term = self.gains.kp * error

        # Integral term with anti-windup: I = Ki * ∫e*dt
        self.integral_error += error * self.dt
        self.integral_error = max(self.integral_limits[0],
                                  min(self.integral_limits[1], self.integral_error))
        i_term = self.gains.ki * self.integral_error

        # Derivative term: D = Kd * de/dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        d_term = self.gains.kd * derivative

        # Total control output
        output = p_term + i_term + d_term

        # Clip to limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Store for next iteration
        self.prev_error = error

        return output

    def reset(self):
        """Reset integral and derivative tracking"""
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = time.time()


class MultiAxisPIDController:
    """
    Manages three PID controllers for X, Y, Z position hold
    Coordinates all three axes for 3D position control
    """

    def __init__(self, x_gains: PIDGains, y_gains: PIDGains, z_gains: PIDGains,
                 dt: float = 0.01):
        """
        Initialize 3-axis PID controller

        Args:
            x_gains, y_gains, z_gains: PIDGains for each axis
            dt: Control loop timestep
        """
        self.pid_x = PIDController(x_gains, dt)
        self.pid_y = PIDController(y_gains, dt)
        self.pid_z = PIDController(z_gains, dt,
                                   output_limits=(0.0, 1.0),  # Thrust is 0-1
                                   integral_limits=(0.0, 0.3))

    def update(self,
               target_pos: Tuple[float, float, float],
               current_pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Update all three axes and return control outputs

        Args:
            target_pos: (x, y, z) desired position in meters
            current_pos: (x, y, z) measured position from sensors

        Returns:
            (vx, vy, thrust) control outputs
        """
        vx = self.pid_x.update(target_pos[0], current_pos[0])
        vy = self.pid_y.update(target_pos[1], current_pos[1])
        thrust = self.pid_z.update(target_pos[2], current_pos[2])

        return vx, vy, thrust

    def reset_all(self):
        """Reset all three controllers"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
