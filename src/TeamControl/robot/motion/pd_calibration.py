"""
Small PD calibration tests for grSim or real robots.

This file does not talk directly to vision/grSim/robot UDP. Instead it uses:

- pose_source.get_robot_pose(robot_id, is_yellow) -> (x, y, theta) or None
- command_sink.send(RobotCommand)

That keeps the same tests usable for grSim and real robots.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

from TeamControl.network.robot_command import RobotCommand
from TeamControl.robot.motion.controller import RobotMotionController


def _wrap_angle(a: float) -> float:
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class CalibrationResult:
    test_name: str
    passed: bool
    score: float
    start_pose: tuple[float, float, float]
    final_pose: tuple[float, float, float]
    target_xy: Optional[tuple[float, float]]
    target_theta: Optional[float]
    elapsed_s: float
    final_position_error_mm: float
    final_heading_error_rad: float
    max_position_error_mm: float
    max_heading_error_rad: float
    samples: int


class PDCalibration:
    def __init__(
        self,
        motion: RobotMotionController,
        pose_source,
        dispatch_q,
        is_yellow: Optional[bool] = None,
        tick_s: float = 0.05,
        command_runtime: float = 0.15,
    ):
        self.motion = motion
        self.pose_source = pose_source
        self.dispatch_q = dispatch_q
        self.is_yellow = motion.is_yellow if is_yellow is None else bool(is_yellow)
        self.tick_s = tick_s
        self.command_runtime = command_runtime

    def _get_pose(self):
        return self.pose_source.get_robot_pose(self.motion.robot_id, self.is_yellow)

    def _send(self, vx: float, vy: float, w: float) -> None:
        cmd = RobotCommand(
            self.motion.robot_id,
            vx,
            vy,
            w,
            0,
            0,
            isYellow=self.is_yellow,
        )
        self.dispatch_q.put((cmd, self.command_runtime))

    def stop(self) -> None:
        self._send(0.0, 0.0, 0.0)
        self.motion.reset()

    def _wait_for_pose(self, timeout_s: float = 1.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            pose = self._get_pose()
            if pose is not None:
                return pose
            time.sleep(self.tick_s)
        raise RuntimeError("No robot pose available for calibration")

    def run_angular_turn_test(
        self,
        angle_rad: float = math.pi / 2,
        gains: Optional[dict] = None,
        use_pd: bool = True,
        use_hardware: bool = False,
        timeout_s: float = 3.0,
        settle_error_rad: float = 0.08,
        deadline_s: float = 0.5,
    ) -> CalibrationResult:
        """Rotate in place by angle_rad and score final heading error."""
        if gains is not None:
            self.motion.apply_gains(gains)

        self.motion.reset()
        start_pose = self._wait_for_pose()
        target_theta = _wrap_angle(start_pose[2] + angle_rad)

        max_heading_error = 0.0
        samples = 0
        t0 = time.monotonic()
        end = t0 + timeout_s

        try:
            while time.monotonic() < end:
                pose = self._get_pose()
                if pose is None:
                    time.sleep(self.tick_s)
                    continue

                err = abs(_wrap_angle(target_theta - pose[2]))
                max_heading_error = max(max_heading_error, err)
                samples += 1

                if err < settle_error_rad:
                    break

                deadline = time.monotonic() + deadline_s
                w = self.motion.rotational_motion(
                    pose[2],
                    target_theta,
                    deadline,
                    use_pd=use_pd,
                    use_hardware=use_hardware,
                )
                self._send(0.0, 0.0, w)
                time.sleep(self.tick_s)
        finally:
            self.stop()

        final_pose = self._wait_for_pose()
        elapsed = time.monotonic() - t0
        final_heading_error = abs(_wrap_angle(target_theta - final_pose[2]))
        score = final_heading_error * 200.0 + elapsed * 10.0

        return CalibrationResult(
            test_name="angular_turn",
            passed=final_heading_error < settle_error_rad,
            score=score,
            start_pose=start_pose,
            final_pose=final_pose,
            target_xy=None,
            target_theta=target_theta,
            elapsed_s=elapsed,
            final_position_error_mm=0.0,
            final_heading_error_rad=final_heading_error,
            max_position_error_mm=0.0,
            max_heading_error_rad=max_heading_error,
            samples=samples,
        )

    def run_linear_forward_test(
        self,
        distance_mm: float = 1000.0,
        gains: Optional[dict] = None,
        use_pd: bool = True,
        use_hardware: bool = False,
        timeout_s: float = 4.0,
        settle_error_mm: float = 100.0,
        deadline_s: float = 0.8,
    ) -> CalibrationResult:
        """Drive forward from the current heading by distance_mm, with w=0."""
        if gains is not None:
            self.motion.apply_gains(gains)

        self.motion.reset()
        start_pose = self._wait_for_pose()
        target_xy = (
            start_pose[0] + distance_mm * math.cos(start_pose[2]),
            start_pose[1] + distance_mm * math.sin(start_pose[2]),
        )
        target_theta = start_pose[2]

        max_pos_error = 0.0
        max_heading_error = 0.0
        samples = 0
        t0 = time.monotonic()
        end = t0 + timeout_s

        try:
            while time.monotonic() < end:
                pose = self._get_pose()
                if pose is None:
                    time.sleep(self.tick_s)
                    continue

                pos_error = math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1])
                heading_error = abs(_wrap_angle(target_theta - pose[2]))
                max_pos_error = max(max_pos_error, pos_error)
                max_heading_error = max(max_heading_error, heading_error)
                samples += 1

                if pos_error < settle_error_mm:
                    break

                deadline = time.monotonic() + deadline_s
                vx, vy = self.motion.translational_motion(
                    pose,
                    target_xy,
                    deadline,
                    use_pd=use_pd,
                    use_hardware=use_hardware,
                )
                self._send(vx, vy, 0.0)
                time.sleep(self.tick_s)
        finally:
            self.stop()

        final_pose = self._wait_for_pose()
        elapsed = time.monotonic() - t0
        final_pos_error = math.hypot(
            target_xy[0] - final_pose[0],
            target_xy[1] - final_pose[1],
        )
        final_heading_error = abs(_wrap_angle(target_theta - final_pose[2]))
        score = final_pos_error + final_heading_error * 200.0 + elapsed * 10.0

        return CalibrationResult(
            test_name="linear_forward",
            passed=final_pos_error < settle_error_mm,
            score=score,
            start_pose=start_pose,
            final_pose=final_pose,
            target_xy=target_xy,
            target_theta=target_theta,
            elapsed_s=elapsed,
            final_position_error_mm=final_pos_error,
            final_heading_error_rad=final_heading_error,
            max_position_error_mm=max_pos_error,
            max_heading_error_rad=max_heading_error,
            samples=samples,
        )

    def save_result(self, result, gains):
        """
        Save gains to settings store using the result score.
        """
        return self.motion.calibrate(gains, score=result.score)
