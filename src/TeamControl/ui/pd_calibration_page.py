import math
import threading
import time

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QPlainTextEdit,
    QFrame,
)

from TeamControl.robot.motion import get_motion_controller
from TeamControl.robot.motion.pd_calibration import PDCalibration
from TeamControl.ui.theme import ACCENT, BG_CARD, BORDER, DANGER, SUCCESS, TEXT, TEXT_DIM


class EnginePoseSource:
    """Pose adapter for PDCalibration using SimEngine's WorldModel."""

    def __init__(self, engine):
        self.engine = engine

    def get_robot_pose(self, robot_id, is_yellow):
        wm = getattr(self.engine, "_wm", None)
        if wm is None:
            return None
        frame = wm.get_latest_frame()
        if frame is None:
            return None

        robots = frame.robots_yellow if is_yellow else frame.robots_blue
        for robot in robots or []:
            if int(robot.id) == int(robot_id):
                theta = getattr(robot, "o", getattr(robot, "orientation", 0.0))
                return (float(robot.x), float(robot.y), float(theta))
        return None


class PDCalibrationPage(QWidget):
    """Small GUI for running the two basic PD calibration tests."""

    log_line = Signal(str)

    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self._engine = engine
        self._last_result = None
        self._last_gains = None
        self._worker = None

        self.log_line.connect(self._append_log)
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title = QLabel("PD Calibration")
        title.setStyleSheet(f"color:{ACCENT}; font-size:18px; font-weight:bold;")
        root.addWidget(title)

        setup = QFrame()
        setup.setStyleSheet(
            f"QFrame {{ background:{BG_CARD}; border:1px solid {BORDER}; border-radius:6px; }}"
        )
        grid = QGridLayout(setup)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setSpacing(8)

        self._robot_id = QComboBox()
        self._robot_id.addItems([str(i) for i in range(6)])
        self._robot_id.setCurrentIndex(0)
        self._is_yellow = QCheckBox("Yellow")
        self._is_yellow.setChecked(True)
        self._use_pd = QCheckBox("Use PD")
        self._use_pd.setChecked(True)
        self._use_hardware = QCheckBox("Use Hardware")
        self._use_hardware.setChecked(False)

        self._turn_kp = self._gain_spin(1.0, step=0.05)
        self._turn_kd = self._gain_spin(0.1, step=0.01)
        self._linear_kp = self._gain_spin(0.002, step=0.0001, decimals=5)
        self._linear_kd = self._gain_spin(0.0005, step=0.0001, decimals=5)
        self._speed_scale = self._gain_spin(1.0, step=0.01)
        self._lateral_drift = self._signed_spin(0.0, -50.0, 50.0, step=0.5)
        self._stop_overshoot = self._signed_spin(0.0, 0.0, 500.0, step=5.0)
        self._min_v = self._signed_spin(0.0, 0.0, 1.0, step=0.01)
        self._min_w = self._signed_spin(0.0, 0.0, 3.0, step=0.01)

        grid.addWidget(QLabel("Robot ID"), 0, 0)
        grid.addWidget(self._robot_id, 0, 1)
        grid.addWidget(self._is_yellow, 0, 2)
        grid.addWidget(self._use_pd, 0, 3)
        grid.addWidget(self._use_hardware, 0, 4)
        grid.addWidget(QLabel("turn_kp"), 1, 0)
        grid.addWidget(self._turn_kp, 1, 1)
        grid.addWidget(QLabel("turn_kd"), 1, 2)
        grid.addWidget(self._turn_kd, 1, 3)
        grid.addWidget(QLabel("linear_kp"), 2, 0)
        grid.addWidget(self._linear_kp, 2, 1)
        grid.addWidget(QLabel("linear_kd"), 2, 2)
        grid.addWidget(self._linear_kd, 2, 3)
        grid.addWidget(QLabel("speed_scale"), 3, 0)
        grid.addWidget(self._speed_scale, 3, 1)
        grid.addWidget(QLabel("drift mm/m"), 3, 2)
        grid.addWidget(self._lateral_drift, 3, 3)
        grid.addWidget(QLabel("overshoot mm"), 4, 0)
        grid.addWidget(self._stop_overshoot, 4, 1)
        grid.addWidget(QLabel("min_v"), 4, 2)
        grid.addWidget(self._min_v, 4, 3)
        grid.addWidget(QLabel("min_w"), 5, 0)
        grid.addWidget(self._min_w, 5, 1)
        root.addWidget(setup)

        actions = QHBoxLayout()
        self._start_btn = QPushButton("Start Calibration Backend")
        self._turn_btn = QPushButton("Run Angular Turn")
        self._linear_btn = QPushButton("Run Linear Forward")
        self._apply_default_btn = QPushButton("Apply Defaults")
        self._save_btn = QPushButton("Save Last Result")
        self._clear_btn = QPushButton("Clear Tuned Gains")

        actions.addWidget(self._start_btn)
        actions.addWidget(self._turn_btn)
        actions.addWidget(self._linear_btn)
        actions.addWidget(self._apply_default_btn)
        actions.addWidget(self._save_btn)
        actions.addWidget(self._clear_btn)
        root.addLayout(actions)

        self._status = QLabel("Start the backend, then run one test.")
        self._status.setStyleSheet(f"color:{TEXT_DIM};")
        root.addWidget(self._status)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setStyleSheet(f"color:{TEXT}; background:#101418;")
        root.addWidget(self._log, 1)

        self._start_btn.clicked.connect(self._start_backend)
        self._turn_btn.clicked.connect(lambda: self._run_test("turn"))
        self._linear_btn.clicked.connect(lambda: self._run_test("linear"))
        self._apply_default_btn.clicked.connect(self._apply_defaults)
        self._save_btn.clicked.connect(self._save_last_result)
        self._clear_btn.clicked.connect(self._clear_tuned_gains)

    def _gain_spin(self, value, step, decimals=4):
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 10.0)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _signed_spin(self, value, lo, hi, step, decimals=2):
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        return spin

    def _gains(self):
        return {
            "turn_kp": self._turn_kp.value(),
            "turn_kd": self._turn_kd.value(),
            "linear_kp": self._linear_kp.value(),
            "linear_kd": self._linear_kd.value(),
            "speed_scale": self._speed_scale.value(),
            "lateral_drift_per_m": self._lateral_drift.value(),
            "stop_overshoot_mm": self._stop_overshoot.value(),
            "min_v": self._min_v.value(),
            "min_w": self._min_w.value(),
        }

    def _motion(self):
        return get_motion_controller(int(self._robot_id.currentText()), self._is_yellow.isChecked())

    def _calibrator(self):
        dispatch_q = getattr(self._engine, "_dispatch_q", None)
        if dispatch_q is None:
            raise RuntimeError("Engine backend is not running; no dispatch queue available")
        return PDCalibration(
            motion=self._motion(),
            pose_source=EnginePoseSource(self._engine),
            dispatch_q=dispatch_q,
        )

    def _start_backend(self):
        if self._engine.is_running:
            self._append_log("[cal] Backend already running")
            return
        self._engine.start(
            "calibration",
            our_id=int(self._robot_id.currentText()),
            opp_id=0,
        )
        self._append_log("[cal] Started calibration backend")

    def _run_test(self, test_name):
        if self._worker and self._worker.is_alive():
            self._append_log("[cal] A calibration test is already running")
            return

        gains = self._gains()
        use_pd = self._use_pd.isChecked()
        use_hardware = self._use_hardware.isChecked()
        self._last_gains = gains
        self._set_status(f"Running {test_name} test...", ACCENT)

        def work():
            try:
                cal = self._calibrator()
                if test_name == "turn":
                    result = cal.run_angular_turn_test(
                        gains=gains,
                        use_pd=use_pd,
                        use_hardware=use_hardware,
                    )
                else:
                    result = cal.run_linear_forward_test(
                        gains=gains,
                        use_pd=use_pd,
                        use_hardware=use_hardware,
                    )
                self._last_result = result
                self.log_line.emit(
                    f"[cal] {result.test_name}: passed={result.passed} "
                    f"score={result.score:.2f} "
                    f"pos_err={result.final_position_error_mm:.1f}mm "
                    f"theta_err={result.final_heading_error_rad:.3f}rad "
                    f"samples={result.samples}"
                )
                self.log_line.emit("[cal] Use Save Last Result if this tuning is good")
            except Exception as exc:
                self.log_line.emit(f"[cal:error] {exc}")
            finally:
                self.log_line.emit("[cal] Test finished")

        self._worker = threading.Thread(target=work, daemon=True)
        self._worker.start()

    def _apply_defaults(self):
        gains = self._motion().apply_default_gains()
        self._set_gain_spins(gains)
        self._append_log("[cal] Applied constants.py default gains")

    def _save_last_result(self):
        if self._last_result is None or self._last_gains is None:
            self._append_log("[cal] No calibration result to save")
            return
        saved = self._motion().calibrate(self._last_gains, score=self._last_result.score)
        self._append_log(f"[cal] Saved gains: {saved}")

    def _clear_tuned_gains(self):
        removed = self._motion().clear_tuned_gains()
        gains = self._motion().get_gains()
        self._set_gain_spins(gains)
        self._append_log(f"[cal] Cleared tuned gains: {removed}; defaults applied")

    def _set_gain_spins(self, gains):
        self._turn_kp.setValue(float(gains["turn_kp"]))
        self._turn_kd.setValue(float(gains["turn_kd"]))
        self._linear_kp.setValue(float(gains["linear_kp"]))
        self._linear_kd.setValue(float(gains["linear_kd"]))
        self._speed_scale.setValue(float(gains.get("speed_scale", 1.0)))
        self._lateral_drift.setValue(float(gains.get("lateral_drift_per_m", 0.0)))
        self._stop_overshoot.setValue(float(gains.get("stop_overshoot_mm", 0.0)))
        self._min_v.setValue(float(gains.get("min_v", 0.0)))
        self._min_w.setValue(float(gains.get("min_w", 0.0)))

    def _set_status(self, text, color=TEXT_DIM):
        self._status.setText(text)
        self._status.setStyleSheet(f"color:{color};")

    def _append_log(self, text):
        self._log.appendPlainText(f"{time.strftime('%H:%M:%S')} {text}")
        if "[cal:error]" in text:
            self._set_status(text, DANGER)
        elif "finished" in text:
            self._set_status("Ready", SUCCESS)
