"""
Dashboard page — field canvas on the left, tabbed sidebar on the right.

Right-panel tabs:
  Monitor   — runtime channels, robot table, game state
  Controls  — robot selector + quick action buttons
"""

import math
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QGridLayout, QFrame, QScrollArea, QTabWidget,
    QPushButton, QComboBox, QSpinBox,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, WARNING, DANGER, TEXT,
    YELLOW_TEAM, BLUE_TEAM, BALL_COLOR,
)


def _card(title_text):
    card = QFrame()
    card.setObjectName("card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(12, 10, 12, 10)
    lay.setSpacing(6)
    title = QLabel(title_text)
    title.setStyleSheet(f"font-size:13px; font-weight:bold; color:{ACCENT}; padding:0;")
    lay.addWidget(title)
    return card, lay


def _val_label(text="—", size=12, bold=True):
    lbl = QLabel(text)
    weight = QFont.Bold if bold else QFont.Normal
    lbl.setFont(QFont("Segoe UI", size, weight))
    return lbl


class DashboardPage(QWidget):
    """Field canvas + tabbed right sidebar."""

    coordinate_hover = Signal(float, float)
    robot_action = Signal(bool, int, str)   # is_yellow, robot_id, action_name
    pick_goto_point = Signal(bool, int)     # is_yellow, robot_id

    def __init__(self, field_canvas, parent=None, engine=None,
                 test_panel=None, calibration_widget=None):
        super().__init__(parent)
        self._field = field_canvas
        self._engine = engine

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._field)

        # ── Right sidebar: tab widget ─────────────────────────────
        self._sidebar_tabs = QTabWidget()
        self._sidebar_tabs.setMinimumWidth(320)
        self._sidebar_tabs.setMaximumWidth(520)
        self._sidebar_tabs.setDocumentMode(True)

        self._sidebar_tabs.addTab(self._build_monitor_tab(), "Monitor")
        self._sidebar_tabs.addTab(self._build_controls_tab(), "Controls")

        if calibration_widget is not None:
            self._sidebar_tabs.addTab(calibration_widget, "Calibration")

        splitter.addWidget(self._sidebar_tabs)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1100, 420])

        root.addWidget(splitter)

        self._field.coordinate_hover.connect(self.coordinate_hover.emit)

        self._frame_times: list[float] = []
        self._current_mode = "vision_only"

    # ── Monitor tab ───────────────────────────────────────────────

    def _build_monitor_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        inner = QWidget()
        inner_lay = QVBoxLayout(inner)
        inner_lay.setContentsMargins(8, 8, 8, 8)
        inner_lay.setSpacing(8)

        self._build_channels_card(inner_lay)
        self._build_robot_card(inner_lay)
        self._build_game_card(inner_lay)
        inner_lay.addStretch()

        scroll.setWidget(inner)
        lay.addWidget(scroll)
        return tab

    # ── Controls tab ──────────────────────────────────────────────

    def _build_controls_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(10)

        # Robot selector card
        sel_card, sel_lay = _card("Robot Selection")
        sel_grid = QGridLayout()
        sel_grid.setSpacing(8)

        sel_grid.addWidget(QLabel("Team:"), 0, 0)
        self._ctrl_team = QComboBox()
        self._ctrl_team.addItems(["Yellow", "Blue"])
        sel_grid.addWidget(self._ctrl_team, 0, 1)

        sel_grid.addWidget(QLabel("Robot ID:"), 1, 0)
        self._ctrl_rid = QSpinBox()
        self._ctrl_rid.setRange(0, 15)
        self._ctrl_rid.setValue(0)
        sel_grid.addWidget(self._ctrl_rid, 1, 1)

        hint = QLabel("Left-click a robot on the field to auto-select")
        hint.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        hint.setWordWrap(True)

        sel_lay.addLayout(sel_grid)
        sel_lay.addWidget(hint)
        lay.addWidget(sel_card)

        # Actions card
        act_card, act_lay = _card("Actions")
        act_grid = QGridLayout()
        act_grid.setSpacing(6)

        go_ball = QPushButton("Go to Ball")
        go_ball.setMinimumHeight(36)
        go_ball.clicked.connect(lambda: self._emit_action("go_to_ball"))

        go_kick = QPushButton("Go && Kick")
        go_kick.setMinimumHeight(36)
        go_kick.clicked.connect(lambda: self._emit_action("go_to_ball_kick"))

        draw_sq = QPushButton("Draw Square")
        draw_sq.setMinimumHeight(36)
        draw_sq.clicked.connect(lambda: self._emit_action("draw_square"))

        goto_pt = QPushButton("Go to Point…")
        goto_pt.setMinimumHeight(36)
        goto_pt.setToolTip("Click a point on the field after pressing this")
        goto_pt.clicked.connect(self._emit_pick_goto_point)

        stop_btn = QPushButton("STOP")
        stop_btn.setObjectName("stopBtn")
        stop_btn.setMinimumHeight(40)
        stop_btn.clicked.connect(lambda: self._emit_action("stop"))

        act_grid.addWidget(go_ball,   0, 0)
        act_grid.addWidget(go_kick,   0, 1)
        act_grid.addWidget(draw_sq,   1, 0)
        act_grid.addWidget(goto_pt,   1, 1)
        act_grid.addWidget(stop_btn,  2, 0, 1, 2)

        self._ctrl_status = QLabel("")
        self._ctrl_status.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        self._ctrl_status.setWordWrap(True)

        act_lay.addLayout(act_grid)
        act_lay.addWidget(self._ctrl_status)
        lay.addWidget(act_card)

        lay.addStretch()
        return tab

    def _emit_action(self, action_name: str):
        is_yellow = self._ctrl_team.currentText() == "Yellow"
        robot_id = self._ctrl_rid.value()
        self.robot_action.emit(is_yellow, robot_id, action_name)
        if action_name == "stop":
            self._ctrl_status.setStyleSheet(f"color:{WARNING}; font-size:11px;")
            self._ctrl_status.setText("Stopped")
        else:
            team = "Yellow" if is_yellow else "Blue"
            self._ctrl_status.setStyleSheet(f"color:{SUCCESS}; font-size:11px;")
            self._ctrl_status.setText(f"{team} #{robot_id} → {action_name.replace('_', ' ')}")

    def _emit_pick_goto_point(self):
        is_yellow = self._ctrl_team.currentText() == "Yellow"
        robot_id = self._ctrl_rid.value()
        self.pick_goto_point.emit(is_yellow, robot_id)
        self._ctrl_status.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._ctrl_status.setText("Click a point on the field…")

    def select_robot(self, is_yellow: bool, robot_id: int):
        """Auto-select a robot (called when user left-clicks a robot on the field)."""
        self._ctrl_team.setCurrentText("Yellow" if is_yellow else "Blue")
        self._ctrl_rid.setValue(robot_id)
        team = "Yellow" if is_yellow else "Blue"
        self._ctrl_status.setStyleSheet(f"color:{SUCCESS}; font-size:11px;")
        self._ctrl_status.setText(f"Selected: {team} #{robot_id}")
        # Switch to Controls tab so the user sees the selection
        self._sidebar_tabs.setCurrentIndex(1)

    # ── Runtime channels card ─────────────────────────────────────

    _CHANNEL_NAMES = {
        "vision":     "Vision",
        "gc":         "Game Controller",
        "robot_recv": "Robot Recv",
        "use_grsim":  "Use grSim",
        "send_grsim": "Send → grSim",
        "record_wm":  "Record WM",
    }

    def _build_channels_card(self, parent_lay):
        card, lay = _card("Runtime Channels")
        grid = QGridLayout()
        grid.setSpacing(4)
        grid.setColumnStretch(1, 1)

        self._channel_dots = {}
        self._channel_name_lbls = {}
        self._channel_latency = {}

        for row, (key, name) in enumerate(self._CHANNEL_NAMES.items()):
            dot = QLabel("●")
            dot.setFixedWidth(18)
            dot.setAlignment(Qt.AlignCenter)
            dot.setStyleSheet(f"color:{TEXT_DIM}; font-size:14px;")
            self._channel_dots[key] = dot

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet(f"color:{TEXT_DIM};")
            self._channel_name_lbls[key] = name_lbl

            lat = QLabel("—")
            lat.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lat.setMinimumWidth(52)
            lat.setStyleSheet(f"color:{TEXT_DIM};")
            self._channel_latency[key] = lat

            grid.addWidget(dot,      row, 0)
            grid.addWidget(name_lbl, row, 1)
            grid.addWidget(lat,      row, 2)
            if key == "vision":
                self._fps_lbl = QLabel("0 fps")
                self._fps_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._fps_lbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
                self._fps_lbl.setStyleSheet(f"color:{TEXT_DIM};")
                grid.addWidget(self._fps_lbl, row, 3)

        lay.addLayout(grid)
        parent_lay.addWidget(card)

    def update_channel_status(self, status):
        for key in self._CHANNEL_NAMES:
            item = status.get(key, {})
            enabled    = bool(item.get("enabled", False))
            latency_ms = item.get("latency_ms")
            stale      = bool(item.get("stale", True))

            dot      = self._channel_dots[key]
            name_lbl = self._channel_name_lbls[key]
            lat      = self._channel_latency[key]

            if not enabled:
                dot.setStyleSheet(f"color:{TEXT_DIM}; font-size:14px;")
                name_lbl.setStyleSheet(f"color:{TEXT_DIM};")
                lat.setText("OFF")
                lat.setStyleSheet(f"color:{TEXT_DIM};")
            elif stale or latency_ms is None:
                dot.setStyleSheet(f"color:{DANGER}; font-size:14px;")
                name_lbl.setStyleSheet(f"color:{TEXT_DIM};")
                lat.setText(">99ms")
                lat.setStyleSheet(f"color:{DANGER}; font-weight:bold;")
            else:
                dot.setStyleSheet(f"color:{SUCCESS}; font-size:14px;")
                name_lbl.setStyleSheet(f"color:{TEXT};")
                lat.setText(f"{latency_ms}ms")
                lat.setStyleSheet(f"color:{SUCCESS}; font-weight:bold;")

    # ── Robot table card ──────────────────────────────────────────

    def _build_robot_card(self, parent_lay):
        card, lay = _card("Robots")
        self._robot_summary = QLabel("Waiting for data…")
        self._robot_summary.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        lay.addWidget(self._robot_summary)

        cols = ["Team", "ID", "X", "Y", "θ°", "Conf"]
        self._robot_table = QTableWidget(0, len(cols))
        self._robot_table.setHorizontalHeaderLabels(cols)
        self._robot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._robot_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._robot_table.setAlternatingRowColors(True)
        self._robot_table.verticalHeader().setVisible(False)
        self._robot_table.setShowGrid(False)
        self._robot_table.setMinimumHeight(200)
        self._robot_table.setMaximumHeight(360)
        hh = self._robot_table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setMinimumSectionSize(40)
        self._robot_table.verticalHeader().setDefaultSectionSize(32)
        self._robot_table.itemSelectionChanged.connect(self._on_table_robot_selected)
        lay.addWidget(self._robot_table)
        parent_lay.addWidget(card)

    def _on_table_robot_selected(self):
        """Clicking a row in the robot table also selects that robot for controls."""
        rows = self._robot_table.selectedItems()
        if not rows:
            return
        row = self._robot_table.currentRow()
        team_item = self._robot_table.item(row, 0)
        id_item   = self._robot_table.item(row, 1)
        if team_item is None or id_item is None:
            return
        is_yellow = team_item.text() == "Y"
        try:
            robot_id = int(id_item.text())
        except ValueError:
            return
        self.select_robot(is_yellow, robot_id)

    # ── Game state card ───────────────────────────────────────────

    def _build_game_card(self, parent_lay):
        card, lay = _card("Game State")
        grid = QGridLayout()
        grid.setSpacing(6)

        grid.addWidget(QLabel("State:"), 0, 0)
        self._gs_state = _val_label("WAITING")
        grid.addWidget(self._gs_state, 0, 1)

        grid.addWidget(QLabel("Mode:"), 1, 0)
        self._gs_mode = _val_label("—")
        grid.addWidget(self._gs_mode, 1, 1)

        grid.addWidget(QLabel("Score:"), 2, 0)
        score_row = QHBoxLayout()
        self._score_y = QLabel("0")
        self._score_y.setStyleSheet(
            f"font-size:22px; font-weight:bold; color:{YELLOW_TEAM};")
        self._score_vs = QLabel(" vs ")
        self._score_vs.setStyleSheet("font-size:14px; font-weight:bold;")
        self._score_b = QLabel("0")
        self._score_b.setStyleSheet(
            f"font-size:22px; font-weight:bold; color:{BLUE_TEAM};")
        score_row.addWidget(self._score_y)
        score_row.addWidget(self._score_vs)
        score_row.addWidget(self._score_b)
        score_row.addStretch()
        grid.addLayout(score_row, 2, 1)

        grid.addWidget(QLabel("Command:"), 3, 0)
        self._gs_command = _val_label("NO DATA")
        grid.addWidget(self._gs_command, 3, 1)

        grid.addWidget(QLabel("Stage:"), 4, 0)
        self._gs_stage = _val_label("NO DATA")
        grid.addWidget(self._gs_stage, 4, 1)

        grid.addWidget(QLabel("Us Yellow:"), 5, 0)
        self._gs_us_yellow = _val_label("NO DATA")
        grid.addWidget(self._gs_us_yellow, 5, 1)

        grid.addWidget(QLabel("Cards:"), 6, 0)
        self._gs_cards = _val_label("NO DATA")
        grid.addWidget(self._gs_cards, 6, 1)

        grid.addWidget(QLabel("Fouls:"), 7, 0)
        self._gs_fouls = _val_label("NO DATA")
        grid.addWidget(self._gs_fouls, 7, 1)

        grid.addWidget(QLabel("YC Timers:"), 8, 0)
        self._gs_yellow_card_times = _val_label("NO DATA", size=10)
        self._gs_yellow_card_times.setWordWrap(True)
        grid.addWidget(self._gs_yellow_card_times, 8, 1)

        lay.addLayout(grid)
        parent_lay.addWidget(card)

    # ── Public update methods ─────────────────────────────────────

    def update_frame(self, snap):
        self._field.set_frame(snap)
        self._update_robot_table(snap)
        self._update_fps()

    def update_game_state(self, state):
        gc_status = state if isinstance(state, dict) else {"state": state}
        game_state = gc_status.get("state")
        if game_state is None:
            self._gs_state.setText("NO DATA")
            self._gs_state.setStyleSheet(f"color:{TEXT_DIM};")
        else:
            name = game_state.name if hasattr(game_state, "name") else str(game_state)
            color_map = {"HALTED": DANGER, "STOPPED": WARNING, "RUNNING": SUCCESS}
            c = color_map.get(name, ACCENT)
            self._gs_state.setText(name)
            self._gs_state.setStyleSheet(f"color:{c}; font-weight:bold;")

        self._set_gc_value(self._gs_command, gc_status.get("command"))
        self._set_gc_value(self._gs_stage, gc_status.get("stage"))

        us_yellow = gc_status.get("us_yellow")
        if us_yellow is None:
            self._gs_us_yellow.setText("NO DATA")
            self._gs_us_yellow.setStyleSheet(f"color:{TEXT_DIM};")
        else:
            self._gs_us_yellow.setText("YES" if us_yellow else "NO")
            color = YELLOW_TEAM if us_yellow else BLUE_TEAM
            self._gs_us_yellow.setStyleSheet(f"color:{color}; font-weight:bold;")

        self._update_gc_discipline(gc_status)

    def _set_gc_value(self, label, value):
        if value is None:
            label.setText("NO DATA")
            label.setStyleSheet(f"color:{TEXT_DIM};")
            return
        label.setText(value.name if hasattr(value, "name") else str(value))
        label.setStyleSheet(f"color:{TEXT}; font-weight:bold;")

    def _update_gc_discipline(self, gc_status):
        yellow_cards = gc_status.get("yellow_cards")
        red_cards = gc_status.get("red_cards")
        fouls = gc_status.get("fouls")
        timers = gc_status.get("yellow_card_times") or []

        if yellow_cards is None and red_cards is None:
            self._gs_cards.setText("NO DATA")
            self._gs_cards.setStyleSheet(f"color:{TEXT_DIM};")
        else:
            yellow_text = "?" if yellow_cards is None else str(yellow_cards)
            red_text = "?" if red_cards is None else str(red_cards)
            self._gs_cards.setText(f"Y {yellow_text} / R {red_text}")
            color = DANGER if red_cards else WARNING if yellow_cards else SUCCESS
            self._gs_cards.setStyleSheet(f"color:{color}; font-weight:bold;")

        if fouls is None:
            self._gs_fouls.setText("NO DATA")
            self._gs_fouls.setStyleSheet(f"color:{TEXT_DIM};")
        else:
            self._gs_fouls.setText(str(fouls))
            self._gs_fouls.setStyleSheet(f"color:{WARNING if fouls else SUCCESS}; font-weight:bold;")

        if not timers:
            self._gs_yellow_card_times.setText("NONE")
            self._gs_yellow_card_times.setStyleSheet(f"color:{TEXT_DIM};")
        else:
            self._gs_yellow_card_times.setText(", ".join(self._format_gc_time(t) for t in timers))
            self._gs_yellow_card_times.setStyleSheet(f"color:{WARNING}; font-weight:bold;")

    def _format_gc_time(self, value):
        try:
            seconds = max(0, int(value) // 1_000_000)
        except (TypeError, ValueError):
            return str(value)
        return f"{seconds // 60}:{seconds % 60:02d}"

    def set_mode(self, mode):
        self._current_mode = mode
        self._gs_mode.setText(mode.upper())
        if mode == "coop":
            from TeamControl.robot.coop import HOME_YELLOW, HOME_BLUE, BALL_START
            self._field.set_targets([
                (*HOME_YELLOW, YELLOW_TEAM),
                (*HOME_BLUE, BLUE_TEAM),
                (*BALL_START, BALL_COLOR),
            ])
            self._field.set_paths([([HOME_YELLOW, HOME_BLUE], ACCENT)])
        else:
            self._field.set_targets([])
            self._field.set_paths([])

    def set_engine_running(self, running):
        if not running:
            self._fps_lbl.setText("0 fps")

    def get_fps(self):
        return len(self._frame_times)

    # ── Internal ──────────────────────────────────────────────────

    def _update_robot_table(self, snap):
        yellow = [r for r in snap.yellow if r is not None]
        blue   = [r for r in snap.blue   if r is not None]
        robots = [(YELLOW_TEAM, "Y", r) for r in yellow] + \
                 [(BLUE_TEAM,   "B", r) for r in blue]

        self._robot_table.blockSignals(True)
        self._robot_table.setRowCount(len(robots))
        for row, (color_hex, team_short, r) in enumerate(robots):
            vals = [team_short, str(r.id), f"{r.x:.0f}",
                    f"{r.y:.0f}", f"{math.degrees(r.o):.1f}",
                    f"{r.confidence:.2f}"]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 0:
                    item.setForeground(QColor(color_hex))
                    item.setFont(QFont("Segoe UI", 10, QFont.Bold))
                self._robot_table.setItem(row, col, item)
        self._robot_table.blockSignals(False)

        ny, nb = len(yellow), len(blue)
        ball = (f"Ball ({snap.ball.x:.0f}, {snap.ball.y:.0f})"
                if snap.ball else "No ball")
        self._robot_summary.setText(
            f"Y:{ny}  B:{nb}  |  {ball}  |  Frame #{snap.frame_number}")

    def _update_fps(self):
        now = time.time()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if t > now - 1.0]
        self._fps_lbl.setText(f"{len(self._frame_times)} fps")
