import math
import time
from typing import Optional, Tuple

from TeamControl.robot import constants as C
from TeamControl.robot.motion.accel import AccelLimiter
from TeamControl.robot.motion.hardware import (
    apply_hardware_gains,
    apply_min_angular_command,
    apply_min_linear_command,
    shorten_target_for_overshoot,
)
from TeamControl.robot.motion.pd import PDController
from TeamControl.robot.motion.settings import PDSettingsStore
from TeamControl.world.transform_cords import world2robot

_MOTION_CONTROLLERS = {}


def get_motion_controller(robot_id, is_yellow: bool = True):
    """Get the one motion brain for this robot."""
    key = (bool(is_yellow), int(robot_id))
    ctrl = _MOTION_CONTROLLERS.get(key)
    if ctrl is None:
        ctrl = RobotMotionController(robot_id=robot_id, is_yellow=is_yellow)
        _MOTION_CONTROLLERS[key] = ctrl
    return ctrl


def _wrap_angle(a: float) -> float:
    """Change any angle into the shortest matching angle."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _speed_from_deadline(dist_mm: float, deadline: float) -> float:
    """How fast to drive so we arrive on time."""
    time_remaining = max(deadline - time.monotonic(), 0.001)
    return min(dist_mm / 1000.0 / time_remaining, C.MAX_SPEED)


def _w_from_deadline(angle_rad: float, deadline: float) -> float:
    """How fast to turn so we arrive on time."""
    time_remaining = max(deadline - time.monotonic(), 0.001)
    return min(abs(angle_rad) / time_remaining, C.MAX_W)


def _run_pd_with_limit(pd: PDController, error, limit: float):
    """Run one PD update with a temporary speed limit."""
    old_limit = pd.out_limit
    pd.out_limit = limit
    try:
        return pd.update(error)
    finally:
        pd.out_limit = old_limit


class RobotMotionController:
    """
    Simple motion controller for one robot.

    Think of it as:
    - turn to face the target
    - drive to the target
    - optionally do both together
    """

    def __init__(
        self,
        robot_id,
        is_yellow: bool = True,
    ):
        self.robot_id = int(robot_id)
        self.is_yellow = bool(is_yellow)
        self.settings = PDSettingsStore()

        gains = self.settings.load_gains(self.robot_id, self.is_yellow)
        self.speed_scale = float(gains["speed_scale"])
        self.lateral_drift_per_m = float(gains["lateral_drift_per_m"])
        self.stop_overshoot_mm = float(gains["stop_overshoot_mm"])
        self.min_v = float(gains["min_v"])
        self.min_w = float(gains["min_w"])

        self.angular_pd = PDController(
            kp=gains["turn_kp"],
            kd=gains["turn_kd"],
            out_limit=C.MAX_W,
        )
        self.linear_pd = PDController(
            kp=gains["linear_kp"],
            kd=gains["linear_kd"],
            out_limit=C.MAX_SPEED,
        )

        self.linear_accel = AccelLimiter(C.LINEAR_AMAX)
        self.angular_accel = AccelLimiter(C.ANGULAR_AMAX)

    def reset(self) -> None:
        """Forget previous errors."""
        self.angular_pd.reset()
        self.linear_pd.reset()
        self.linear_accel.reset()
        self.angular_accel.reset()

    def get_gains(self) -> dict:
        """Show the gains being used right now."""
        return {
            "turn_kp": self.angular_pd.kp,
            "turn_kd": self.angular_pd.kd,
            "linear_kp": self.linear_pd.kp,
            "linear_kd": self.linear_pd.kd,
            "speed_scale": self.speed_scale,
            "lateral_drift_per_m": self.lateral_drift_per_m,
            "stop_overshoot_mm": self.stop_overshoot_mm,
            "min_v": self.min_v,
            "min_w": self.min_w,
        }

    def apply_gains(self, gains: dict) -> dict:
        """Use these gains now, but do not save them."""
        if "turn_kp" in gains:
            self.angular_pd.kp = float(gains["turn_kp"])
        if "turn_kd" in gains:
            self.angular_pd.kd = float(gains["turn_kd"])
        if "linear_kp" in gains:
            self.linear_pd.kp = float(gains["linear_kp"])
        if "linear_kd" in gains:
            self.linear_pd.kd = float(gains["linear_kd"])
        if "speed_scale" in gains:
            self.speed_scale = float(gains["speed_scale"])
        if "lateral_drift_per_m" in gains:
            self.lateral_drift_per_m = float(gains["lateral_drift_per_m"])
        if "stop_overshoot_mm" in gains:
            self.stop_overshoot_mm = float(gains["stop_overshoot_mm"])
        if "min_v" in gains:
            self.min_v = float(gains["min_v"])
        if "min_w" in gains:
            self.min_w = float(gains["min_w"])

        self.reset()
        return self.get_gains()

    def apply_default_gains(self) -> dict:
        """Use the default gains from constants.py."""
        return self.apply_gains(self.settings.load_default_gains())

    def has_tuned_gains(self) -> bool:
        """True if this robot has saved gains."""
        return self.settings.has_robot_gains(self.robot_id, self.is_yellow)

    def reload_saved_or_default_gains(self) -> tuple[dict, str]:
        """Use saved gains if they exist, otherwise use defaults."""
        gains, source = self.settings.load_gains_with_source(
            self.robot_id,
            self.is_yellow,
        )
        return self.apply_gains(gains), source

    def clear_tuned_gains(self) -> bool:
        """Delete saved gains for this robot, then use defaults."""
        removed = self.settings.delete_robot_gains(
            self.robot_id,
            self.is_yellow,
        )
        self.apply_default_gains()
        return removed

    def calibrate(self, gains: dict, score: Optional[float] = None) -> dict:
        """Use these gains now and save them for next time."""
        applied = self.apply_gains(gains)
        return self.settings.save_gains(
            self.robot_id,
            self.is_yellow,
            applied,
            score=score,
        )

    def is_close_to_target(
        self,
        current_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        threshold_mm: float = 100.0,
    ) -> bool:
        """True when the robot is close enough to stop driving."""
        dx = target_xy[0] - current_xy[0]
        dy = target_xy[1] - current_xy[1]
        close = math.hypot(dx, dy) < threshold_mm
        if close:
            self.linear_pd.reset()
        return close

    def is_facing_dir(
        self,
        current_theta: float,
        target_theta: float,
        threshold_rad: float = 0.1,
    ) -> bool:
        """True when the robot is facing close enough to stop turning."""
        angle_error = _wrap_angle(target_theta - current_theta)
        facing = abs(angle_error) < threshold_rad
        if facing:
            self.angular_pd.reset()
        return facing

    def rotational_motion(
        self,
        current_theta: float,
        target_theta: float,
        deadline: float,
        use_pd: bool = True,
        use_hardware: bool = True,
    ) -> float:
        """Turn only. Returns w in rad/s."""
        angle = _wrap_angle(target_theta - current_theta)
        if abs(angle) < C.ANGLE_EPSILON:
            self.angular_pd.reset()
            return 0.0

        w_limit = _w_from_deadline(angle, deadline)
        if use_pd:
            w = _run_pd_with_limit(self.angular_pd, angle, w_limit)
        else:
            self.angular_pd.reset()
            w = max(-w_limit, min(w_limit, self.angular_pd.kp * angle))

        if use_hardware:
            w = apply_min_angular_command(w, self.min_w)

        w = self.angular_accel.limit(w)
        return w

    def translational_motion(
        self,
        current_pos: Tuple[float, float, float],
        target_pos: Tuple[float, float],
        deadline: float,
        use_pd: bool = True,
        use_hardware: bool = True,
    ) -> Tuple[float, float]:
        """Drive only. Returns vx, vy in robot frame."""
        target_in_robot_frame = world2robot(current_pos, target_pos)

        if use_hardware:
            target_in_robot_frame = shorten_target_for_overshoot(
                target_in_robot_frame,
                self.stop_overshoot_mm,
            )

        dist = math.hypot(target_in_robot_frame[0], target_in_robot_frame[1])
        if dist < C.KICKER_ZONE:
            self.linear_pd.reset()
            return 0.0, 0.0

        speed = _speed_from_deadline(dist, deadline)
        if use_pd:
            vx, vy = _run_pd_with_limit(self.linear_pd, target_in_robot_frame, speed)
        else:
            self.linear_pd.reset()
            unit_x = target_in_robot_frame[0] / dist
            unit_y = target_in_robot_frame[1] / dist
            vx, vy = unit_x * speed, unit_y * speed

        if use_hardware:
            vx, vy = apply_hardware_gains(vx, vy, self.get_gains())
            vx, vy = apply_min_linear_command(vx, vy, self.min_v)

        vx, vy = self.linear_accel.limit((vx, vy))
        return vx, vy

    def general_motion(
        self,
        current_pos: Tuple[float, float, float],
        target_pos: Tuple[float, float],
        target_theta: float,
        deadline: float,
        use_pd: bool = True,
        use_hardware: bool = True,
    ) -> Tuple[float, float, float]:
        """Turn first if badly misaligned, otherwise drive and turn together."""
        current_theta = current_pos[2]
        angle_err = abs(_wrap_angle(target_theta - current_theta))

        # If we point the wrong way, fix that first.
        if angle_err > math.radians(60):
            w = self.rotational_motion(
                current_theta,
                target_theta,
                deadline,
                use_pd,
                use_hardware,
            )
            self.linear_pd.reset()
            return 0.0, 0.0, w

        vx, vy = self.translational_motion(
            current_pos,
            target_pos,
            deadline,
            use_pd,
            use_hardware,
        )
        w = self.rotational_motion(
            current_theta,
            target_theta,
            deadline,
            use_pd,
            use_hardware,
        )

        target_in_robot_frame = world2robot(current_pos, target_pos)
        dist = math.hypot(target_in_robot_frame[0], target_in_robot_frame[1])

        # Drive less while turning. Turn less while far away.
        linear_scale = 1.0 - max(0.0, min(0.8, angle_err / math.pi))
        angular_scale = 1.0 - max(0.0, min(0.6, dist / C.BLEND_DIST))

        return vx * linear_scale, vy * linear_scale, w * angular_scale

    def tuned_velocity(
        self,
        vx: float,
        vy: float,
        w: float,
        use_hardware: bool = True,
    ) -> Tuple[float, float, float]:
        """
        Apply this robot's saved motion calibration to an existing velocity.

        This is for behaviours that already made a tactical velocity decision
        such as kick/dribble logic, but still need the same per-robot hardware
        compensation used by the PD target controller.
        """
        vx = float(vx)
        vy = float(vy)
        w = float(w)

        speed = math.hypot(vx, vy)
        if speed > C.MAX_SPEED and speed > 0.0:
            scale = C.MAX_SPEED / speed
            vx *= scale
            vy *= scale

        w = max(-C.MAX_W, min(C.MAX_W, w))

        if use_hardware:
            vx, vy = apply_hardware_gains(vx, vy, self.get_gains())
            vx, vy = apply_min_linear_command(vx, vy, self.min_v)
            w = apply_min_angular_command(w, self.min_w)

            speed = math.hypot(vx, vy)
            if speed > C.MAX_SPEED and speed > 0.0:
                scale = C.MAX_SPEED / speed
                vx *= scale
                vy *= scale
            w = max(-C.MAX_W, min(C.MAX_W, w))

        vx, vy = self.linear_accel.limit((vx, vy))
        w = self.angular_accel.limit(w)
        return vx, vy, w
