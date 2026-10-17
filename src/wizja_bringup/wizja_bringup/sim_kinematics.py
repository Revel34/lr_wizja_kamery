"""Kinematics of the simulated rover (ROS-free, unit-tested).

Rover 3.0 has 4 independently steered wheels, so it can drive forward, sideways and turn on the
spot. The simulation is purely kinematic: the commanded body velocity (vx, vy, wz) is rate-limited
and integrated into a 2D pose in odom; there is no physics (the rover stays at z = 0).
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

WHEEL_RADIUS = 0.13  # [m] rover30 wheel mesh radius
STEER_LIMIT = math.pi / 2  # rover30 steer joint limits are +/- pi/2


@dataclass
class Limits:
    max_linear: float = 1.0        # [m/s]   magnitude of (vx, vy)
    max_angular: float = 1.0       # [rad/s]
    max_linear_acc: float = 1.0    # [m/s^2]
    max_angular_acc: float = 2.0   # [rad/s^2]
    cmd_timeout: float = 0.5       # [s] stop when no command arrived for this long


def _approach(value: float, target: float, max_step: float) -> float:
    return value + max(-max_step, min(max_step, target - value))


@dataclass
class RoverPose:
    """Rate-limited integration of body velocity commands into an odom pose."""

    limits: Limits = field(default_factory=Limits)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0          # current (rate-limited) body velocity
    vy: float = 0.0
    wz: float = 0.0
    _target: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    _last_cmd: Optional[float] = None

    def command(self, vx: float, vy: float, wz: float, now: float) -> None:
        lim = self.limits
        speed = math.hypot(vx, vy)
        if speed > lim.max_linear:
            vx, vy = vx * lim.max_linear / speed, vy * lim.max_linear / speed
        wz = max(-lim.max_angular, min(lim.max_angular, wz))
        self._target = (vx, vy, wz)
        self._last_cmd = now

    def step(self, now: float, dt: float) -> None:
        lim = self.limits
        if dt <= 0.0:
            return
        if self._last_cmd is None or now - self._last_cmd > lim.cmd_timeout:
            self._target = (0.0, 0.0, 0.0)
        tvx, tvy, twz = self._target
        # Linear acceleration limited as a vector so the direction of travel is kept.
        dvx, dvy = tvx - self.vx, tvy - self.vy
        dv = math.hypot(dvx, dvy)
        max_dv = lim.max_linear_acc * dt
        if dv > max_dv:
            dvx, dvy = dvx * max_dv / dv, dvy * max_dv / dv
        self.vx += dvx
        self.vy += dvy
        self.wz = _approach(self.wz, twz, lim.max_angular_acc * dt)
        # Midpoint integration of the body velocity in the odom frame.
        mid_yaw = self.yaw + 0.5 * self.wz * dt
        c, s = math.cos(mid_yaw), math.sin(mid_yaw)
        self.x += (self.vx * c - self.vy * s) * dt
        self.y += (self.vx * s + self.vy * c) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))


def swerve_wheels(vx: float, vy: float, wz: float, wheel_positions: Dict[str, Tuple[float, float]],
                  previous_angles: Optional[Dict[str, float]] = None,
                  eps: float = 1e-3) -> Tuple[Dict[str, float], Dict[str, float]]:
    """Steer angle [rad] and ground speed [m/s] of each wheel for a body velocity.

    The wheel at (px, py) moves with v + w x r = (vx - wz*py, vy + wz*px). Its steer angle points along
    that velocity; angles beyond the +/- pi/2 joint limit are flipped by pi with the wheel speed
    reversed. A wheel that (almost) does not move keeps its previous angle.
    """
    previous_angles = previous_angles or {}
    angles, speeds = {}, {}
    for name, (px, py) in wheel_positions.items():
        wvx = vx - wz * py
        wvy = vy + wz * px
        speed = math.hypot(wvx, wvy)
        if speed < eps:
            angles[name] = previous_angles.get(name, 0.0)
            speeds[name] = 0.0
            continue
        angle = math.atan2(wvy, wvx)
        if angle > STEER_LIMIT + 1e-9:
            angle -= math.pi
            speed = -speed
        elif angle < -STEER_LIMIT - 1e-9:
            angle += math.pi
            speed = -speed
        angles[name] = angle
        speeds[name] = speed
    return angles, speeds
