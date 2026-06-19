"""Reusable PID controller with anti-windup and output clamping."""

from typing import Optional


class PIDController:
    """Discrete PID controller suitable for periodic timer callbacks."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = float('-inf'),
        output_max: float = float('inf'),
        integral_max: float = float('inf'),
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self._integral: float = 0.0
        self._prev_error: Optional[float] = None

    def compute(self, error: float, dt: float) -> float:
        """Return the control output for the given error and timestep."""
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))

        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))

    def reset(self) -> None:
        """Clear accumulated integral and derivative state."""
        self._integral = 0.0
        self._prev_error = None
