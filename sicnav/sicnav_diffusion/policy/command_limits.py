"""Limit outgoing commands using measured velocity and simulation-time intervals."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CommandLimiter:
    max_forward: float
    max_reverse: float
    max_angular: float
    acceleration: float
    deceleration: float
    angular_acceleration: float = 3.0  # Matches the Gazebo DiffDrive system.

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in (
                self.max_forward, self.max_reverse, self.max_angular,
                self.acceleration, self.deceleration, self.angular_acceleration)):
            raise ValueError('Command limits must be positive and finite')

    def limit(self, v, omega, measured_v, measured_omega, dt):
        if any(not math.isfinite(value) for value in (v, omega, measured_v, measured_omega, dt)) or dt < 0:
            raise ValueError('Command, measured velocity and elapsed time must be finite; dt must be nonnegative')
        target_v = min(self.max_forward, max(-self.max_reverse, v))
        if measured_v * target_v < 0:
            # Brake to zero before accelerating in the opposite direction.
            limited_v = math.copysign(max(0.0, abs(measured_v) - self.deceleration * dt), measured_v)
        elif abs(target_v) > abs(measured_v):
            limited_v = math.copysign(min(abs(target_v), abs(measured_v) + self.acceleration * dt), target_v)
        else:
            magnitude = max(abs(target_v), abs(measured_v) - self.deceleration * dt)
            limited_v = math.copysign(magnitude, measured_v)
        limited_v = min(self.max_forward, max(-self.max_reverse, limited_v))
        target_omega = min(self.max_angular, max(-self.max_angular, omega))
        delta_omega = self.angular_acceleration * dt
        limited_omega = min(measured_omega + delta_omega,
                            max(measured_omega - delta_omega, target_omega))
        limited_omega = min(self.max_angular, max(-self.max_angular, limited_omega))
        changed = not (math.isclose(v, limited_v, abs_tol=1e-9) and
                       math.isclose(omega, limited_omega, abs_tol=1e-9))
        return limited_v, limited_omega, changed
