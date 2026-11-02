"""Production-geometry conversion for the monitored parallel waist."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class WaistConfig:
    lax: float = 35.0
    lay: float = 0.0
    laz: float = 38.0
    l4: float = 102.5
    lc: float = 46.5
    ll: float = 104.07
    lb: float = 19.0
    elbow: int = -1
    theta1_sign: float = -1.0
    theta2_sign: float = 1.0


class ParallelWaistKinematics:
    """Python equivalent of plugins/torso ParallelWaistKinematics."""

    _SEEDS = (
        (0.0, 0.0),
        (0.0, math.radians(40.0)),
        (0.0, math.radians(80.0)),
        (math.radians(20.0), math.radians(80.0)),
        (math.radians(-20.0), math.radians(80.0)),
        (0.0, math.radians(120.0)),
    )

    def __init__(self, config: WaistConfig = WaistConfig()):
        self.config = config

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _geometry(self, roll: float, pitch: float) -> Tuple[float, ...]:
        c = self.config
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        return (
            c.l4 * cp - c.lax * sr * cp - c.lay * cr * cp + c.laz * sp,
            c.l4 * sp - c.lax * sr * sp - c.lay * cr * sp - c.laz * cp,
            -c.lax * cr + c.lay * sr + c.lb,
            c.l4 * cp + c.lax * sr * cp - c.lay * cr * cp + c.laz * sp,
            c.l4 * sp + c.lax * sr * sp - c.lay * cr * sp - c.laz * cp,
            c.lax * cr + c.lay * sr - c.lb,
        )

    def _residual(
        self, roll: float, pitch: float, theta1: float, theta2: float
    ) -> Tuple[float, float]:
        p1, q1, z1, p2, q2, z2 = self._geometry(roll, pitch)
        c = self.config
        u1 = p1 - c.lc * math.sin(theta1)
        v1 = q1 + c.lc * math.cos(theta1)
        u2 = p2 - c.lc * math.sin(theta2)
        v2 = q2 + c.lc * math.cos(theta2)
        return (
            u1 * u1 + v1 * v1 + z1 * z1 - c.ll * c.ll,
            u2 * u2 + v2 * v2 + z2 * z2 - c.ll * c.ll,
        )

    def _inverse(self, roll: float, pitch: float, elbow: int) -> Optional[Tuple[float, float]]:
        p1, q1, z1, p2, q2, z2 = self._geometry(roll, pitch)
        c = self.config
        a1, b1 = -2.0 * c.lc * p1, 2.0 * c.lc * q1
        a2, b2 = -2.0 * c.lc * p2, 2.0 * c.lc * q2
        c1 = c.ll * c.ll - p1 * p1 - q1 * q1 - z1 * z1 - c.lc * c.lc
        c2 = c.ll * c.ll - p2 * p2 - q2 * q2 - z2 * z2 - c.lc * c.lc
        r1, r2 = math.hypot(a1, b1), math.hypot(a2, b2)
        if abs(c1) > r1 or abs(c2) > r2:
            return None
        sign = 1.0 if elbow >= 0 else -1.0
        theta1 = self._wrap(
            math.atan2(c1, sign * math.sqrt(max(0.0, r1 * r1 - c1 * c1)))
            - math.atan2(b1, a1)
        )
        theta2 = self._wrap(
            math.atan2(c2, sign * math.sqrt(max(0.0, r2 * r2 - c2 * c2)))
            - math.atan2(b2, a2)
        )
        return theta1, theta2

    def joint_to_motor(self, pitch: float, roll: float) -> Optional[Tuple[float, float]]:
        theta = self._inverse(roll, pitch, self.config.elbow)
        if theta is None:
            return None
        return (
            self.config.theta1_sign * theta[0],
            self.config.theta2_sign * theta[1],
        )

    def _solve_from_seed(
        self, theta1: float, theta2: float, seed_roll: float, seed_pitch: float
    ) -> Optional[Tuple[float, float]]:
        roll, pitch = seed_roll, seed_pitch
        epsilon = 1.0e-6
        for _ in range(100):
            f1, f2 = self._residual(roll, pitch, theta1, theta2)
            if math.hypot(f1, f2) < 1.0e-9:
                return roll, pitch
            fr1, fr2 = self._residual(roll + epsilon, pitch, theta1, theta2)
            fp1, fp2 = self._residual(roll, pitch + epsilon, theta1, theta2)
            j11, j21 = (fr1 - f1) / epsilon, (fr2 - f2) / epsilon
            j12, j22 = (fp1 - f1) / epsilon, (fp2 - f2) / epsilon
            determinant = j11 * j22 - j12 * j21
            if not math.isfinite(determinant) or abs(determinant) < 1.0e-10:
                return None
            roll += (-f1 * j22 + j12 * f2) / determinant
            pitch += (j21 * f1 - j11 * f2) / determinant
            if not math.isfinite(roll) or not math.isfinite(pitch):
                return None
        return None

    def motor_to_joint(self, motor1: float, motor2: float) -> Optional[Tuple[float, float]]:
        """Return (pitch, roll), or None when no physical FK branch verifies."""

        theta1 = self.config.theta1_sign * motor1
        theta2 = self.config.theta2_sign * motor2
        best_score = math.inf
        best = None
        for seed_roll, seed_pitch in self._SEEDS:
            solution = self._solve_from_seed(
                theta1, theta2, seed_roll, seed_pitch
            )
            if solution is None:
                continue
            roll, pitch = solution
            if not (
                math.radians(-35.0) <= pitch <= math.radians(125.0)
                and math.radians(-35.0) <= roll <= math.radians(35.0)
            ):
                continue
            check = self._inverse(roll, pitch, self.config.elbow)
            if check is None:
                continue
            score = math.hypot(
                self._wrap(check[0] - theta1), self._wrap(check[1] - theta2)
            )
            if score < best_score:
                best_score = score
                best = (pitch, roll)
        if best is None or best_score > math.radians(3.0):
            return None
        return best
