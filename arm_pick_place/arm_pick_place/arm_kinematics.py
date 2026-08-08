#!/usr/bin/env python3
"""3-DOF robotic arm forward and inverse kinematics.

Arm layout (spatial, RRP-like):

    Joint 1 (waist):    rotation about Z axis             (theta1)
    Joint 2 (shoulder): pitch about the Y axis            (theta2)
    Joint 3 (elbow):    pitch about the Y axis            (theta3)

Link lengths:

    L1 = 0.20 m   base height (ground -> shoulder)
    L2 = 0.40 m   upper arm (shoulder -> elbow)
    L3 = 0.35 m   forearm (elbow -> wrist / end-effector)

Forward kinematics maps joint angles (theta1, theta2, theta3) to the
end-effector Cartesian position (x, y, z).  Inverse kinematics maps a
Cartesian target back to joint angles using atan2 and the law of
cosines (elbow-up / elbow-down solutions).
"""

import math


class ThreeDOFArm:
    """3-DOF spatial arm with forward and inverse kinematics."""

    L1 = 0.20
    L2 = 0.40
    L3 = 0.35

    # Joint limits in radians
    LIMITS = {
        'waist': (-math.pi, math.pi),
        'shoulder': (-math.pi / 2.0, math.pi / 2.0),
        'elbow': (-math.pi * 5.0 / 6.0, math.pi * 5.0 / 6.0),
    }

    def __init__(self, l1=None, l2=None, l3=None):
        if l1 is not None:
            self.L1 = float(l1)
        if l2 is not None:
            self.L2 = float(l2)
        if l3 is not None:
            self.L3 = float(l3)

    # ------------------------------------------------------------------
    def forward_kinematics(self, theta1, theta2, theta3):
        """Return (x, y, z) of the end effector for given joint angles (rad)."""
        theta2_3 = theta2 + theta3
        r = self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2_3)
        x = r * math.cos(theta1)
        y = r * math.sin(theta1)
        z = self.L1 + self.L2 * math.sin(theta2) + self.L3 * math.sin(theta2_3)
        return x, y, z

    # ------------------------------------------------------------------
    def inverse_kinematics(self, x, y, z, elbow_up=True):
        """Return (theta1, theta2, theta3) for target (x, y, z), or None.

        Returns None when the target is outside the reachable workspace.
        """
        reach = self.L2 + self.L3
        d = math.hypot(math.hypot(x, y), z - self.L1)
        if d > reach + 1e-9 or d < abs(self.L2 - self.L3) - 1e-9:
            return None

        theta1 = math.atan2(y, x)
        r = math.hypot(x, y)
        zp = z - self.L1
        d_sq = r * r + zp * zp

        cos_t3 = (d_sq - self.L2 * self.L2 - self.L3 * self.L3) / \
                 (2.0 * self.L2 * self.L3)
        cos_t3 = max(-1.0, min(1.0, cos_t3))

        sin_t3 = math.sqrt(1.0 - cos_t3 * cos_t3)
        if not elbow_up:
            sin_t3 = -sin_t3
        theta3 = math.atan2(sin_t3, cos_t3)

        beta = math.atan2(zp, r)
        gamma = math.atan2(self.L3 * sin_t3,
                           self.L2 + self.L3 * cos_t3)
        theta2 = beta - gamma

        return self.sanitize(theta1, theta2, theta3)

    # ------------------------------------------------------------------
    def sanitize(self, theta1, theta2, theta3):
        """Clamp joint angles to their limits."""
        return (
            self._clamp(theta1, self.LIMITS['waist']),
            self._clamp(theta2, self.LIMITS['shoulder']),
            self._clamp(theta3, self.LIMITS['elbow']),
        )

    @staticmethod
    def _clamp(value, limits):
        lo, hi = limits
        return max(lo, min(hi, value))

    # ------------------------------------------------------------------
    def is_reachable(self, x, y, z):
        """True if the Cartesian target is inside the reachable workspace."""
        d = math.hypot(math.hypot(x, y), z - self.L1)
        return abs(self.L2 - self.L3) - 1e-9 <= d <= self.L2 + self.L3 + 1e-9

    def self_test(self):
        """Round-trip tests: IK(FK(angles)) == angles and FK(IK(target)) == target."""
        print("ThreeDOFArm self-test (L1=%.2f L2=%.2f L3=%.2f)" %
              (self.L1, self.L2, self.L3))
        ok = True

        samples = [
            (0.0, 0.0, 0.0),
            (math.pi / 4, math.pi / 6, -math.pi / 4),
            (math.pi / 2, math.pi / 3, math.pi / 6),
            (-math.pi / 3, -math.pi / 4, math.pi / 4),
            (math.pi, math.pi / 6, math.pi / 3),
        ]

        for angles in samples:
            xyz = self.forward_kinematics(*angles)
            back = self.inverse_kinematics(*xyz)
            if back is None:
                print("  FAIL: FK(%s) -> %s -> IK -> None" %
                      (self._fmt(angles), self._fmt(xyz)))
                ok = False
                continue
            fk_back = self.forward_kinematics(*back)
            err = max(abs(a - b) for a, b in zip(xyz, fk_back))
            status = "PASS" if err < 1e-9 else "FAIL"
            if status == "FAIL":
                ok = False
            print("  %s FK(%-25s) = %-35s IK -> %s -> FK %s (err %.2e)"
                  % (status, self._fmt(angles), self._fmt(xyz),
                     self._fmt(back), self._fmt(fk_back), err))

        targets = [
            (0.75, 0.0, 0.20),
            (0.60, 0.35, 0.30),
            (-0.40, 0.40, 0.10),
            (0.50, -0.25, 0.45),
        ]
        for target in targets:
            joint = self.inverse_kinematics(*target)
            if joint is None:
                print("  FAIL: IK(%s) -> None (unreachable?)" % self._fmt(target))
                ok = False
                continue
            fk = self.forward_kinematics(*joint)
            err = max(abs(a - b) for a, b in zip(target, fk))
            status = "PASS" if err < 1e-9 else "FAIL"
            if status == "FAIL":
                ok = False
            print("  %s IK(%-35s) = %s -> FK -> %s (err %.2e)"
                  % (status, self._fmt(target), self._fmt(joint),
                     self._fmt(fk), err))

        print("Self-test result:", "PASS" if ok else "FAIL")
        return ok

    @staticmethod
    def _fmt(vals):
        return "(" + ", ".join("%.4f" % v for v in vals) + ")"


def main():
    import sys
    arm = ThreeDOFArm()
    ok = arm.self_test()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
