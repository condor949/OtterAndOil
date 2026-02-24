#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import tempfile
import numpy as np
import json

# Allow importing OtterAndOil packages when run from any directory
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel,
    QSlider, QHBoxLayout, QProgressBar, QFrame, QButtonGroup, QRadioButton,
    QCheckBox, QSizePolicy, QStackedWidget, QListWidget, QListWidgetItem, QDoubleSpinBox,
    QSpinBox, QComboBox, QGroupBox, QFormLayout, QScrollArea, QAbstractItemView,
    QSplitter, QMessageBox,
)
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtCore import Qt, QPoint, QThread, pyqtSignal

BOUNDING_MARGIN = 10  # margin around bounding box of peaks when exporting coordinates

# --- Trajectory geometry ---
def _deg2rad(d):
    return d * math.pi / 180.0

def _normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-10 else v

def build_center_polyline(segments, start_x, start_y, start_angle_deg=0, arc_samples=32):
    """Build center path as list of (x,y) from segments. Angle 0 = east (positive x).
    Returns (pts, segment_end_indices): segment_end_indices[i] = index in pts of last point of segment i."""
    pts = []
    segment_end_indices = []
    x, y = float(start_x), float(start_y)
    angle_rad = _deg2rad(start_angle_deg)
    pts.append((x, y))

    for seg in segments:
        kind = seg.get("type", "line")
        if kind == "line":
            length = float(seg.get("length", 100))
            seg_angle_deg = float(seg.get("angle", 0))
            angle_rad = _deg2rad(seg_angle_deg)
            dx = length * math.cos(angle_rad)
            dy = length * math.sin(angle_rad)
            x, y = x + dx, y + dy
            pts.append((x, y))
            segment_end_indices.append(len(pts) - 1)
        elif kind == "arc":
            radius = float(seg.get("radius", 50))
            arc_angle_deg = float(seg.get("angle", 90))
            direction = seg.get("direction", "left")
            arc_rad = _deg2rad(arc_angle_deg)
            perp = np.array([-math.sin(angle_rad), math.cos(angle_rad)])
            if direction == "right":
                perp = -perp
                arc_rad = -arc_rad
            cx, cy = x + radius * perp[0], y + radius * perp[1]
            # Start angle = actual angle of current point on circle (so first arc point equals (x,y))
            start_angle = math.atan2(y - cy, x - cx)
            n_arc = max(2, arc_samples)
            for k in range(0, n_arc + 1):
                t = k / n_arc
                a = start_angle + t * arc_rad
                px = cx + radius * math.cos(a)
                py = cy + radius * math.sin(a)
                pts.append((px, py))
                x, y = px, py
            angle_rad = angle_rad + arc_rad
            segment_end_indices.append(len(pts) - 1)

    return pts, segment_end_indices

def _segment_angle(p0, p1, p2):
    """Angle at p1 between (p0,p1) and (p1,p2), in radians, in [0, pi]."""
    v1 = np.array([p1[0] - p0[0], p1[1] - p0[1]])
    v2 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-10 or n2 < 1e-10:
        return math.pi
    v1, v2 = v1 / n1, v2 / n2
    cos_a = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return math.acos(cos_a)

def _left_normal(p0, p1):
    """Unit normal to segment (p0,p1) pointing left (counterclockwise)."""
    d = np.array([p1[0] - p0[0], p1[1] - p0[1]])
    n = np.linalg.norm(d)
    if n < 1e-10:
        return np.array([1.0, 0.0])
    d = d / n
    return np.array([-d[1], d[0]])

def _rounding_radius(theta_rad, mode, single_r, r_base_sin, l_max_tan):
    """Compute rounding radius at vertex with angle theta_rad. mode: 'single' or 'angle'."""
    if mode == "single":
        return single_r
    # r(θ): r = r_base * sin(θ/2) gives constant cut length; or r = L_max * tan(θ/2)
    half = theta_rad / 2
    if half < 1e-10:
        return 1e-6
    if l_max_tan is not None and l_max_tan > 0:
        return min(l_max_tan * math.tan(half), single_r)
    return r_base_sin * math.sin(half)

def offset_polyline_with_rounding(points, offset_d, rounding_mode="single", single_r=20,
                                  r_base_sin=30, l_max_tan=None, side="left"):
    """Build offset path (left or right). side: 'left' or 'right'. Returns list of (x,y)."""
    if len(points) < 2:
        return []
    out = []
    n = len(points)
    mult = 1 if side == "left" else -1
    d = mult * offset_d

    for i in range(n - 1):
        p0, p1 = points[i], points[i + 1]
        norm = _left_normal(p0, p1)
        a0 = (points[i][0] + d * norm[0], points[i][1] + d * norm[1])
        a1 = (points[i + 1][0] + d * norm[0], points[i + 1][1] + d * norm[1])
        if i == 0:
            out.append(a0)
        out.append(a1)

        if i < n - 2:
            p2 = points[i + 2]
            theta = _segment_angle(p0, p1, p2)
            r = _rounding_radius(theta, rounding_mode, single_r, r_base_sin, l_max_tan)
            len_prev = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
            len_next = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
            r_max = min(len_prev, len_next) * math.tan(theta / 2) if theta > 1e-10 else single_r
            r = min(r, max(0.1, r_max))

            norm_prev = _left_normal(p0, p1)
            norm_next = _left_normal(p1, p2)
            o1_1 = (p1[0] + d * norm_prev[0], p1[1] + d * norm_prev[1])
            o2_0 = (p1[0] + d * norm_next[0], p1[1] + d * norm_next[1])
            dir_prev = _normalize(np.array([p1[0] - p0[0], p1[1] - p0[1]]))
            dir_next = _normalize(np.array([p2[0] - p1[0], p2[1] - p1[1]]))

            # Center M of rounding arc: offset lines shifted by r inward; M = L1' cap L2'
            v = np.array([o1_1[0] - o2_0[0] + r * (norm_prev[0] - norm_next[0]),
                         o1_1[1] - o2_0[1] + r * (norm_prev[1] - norm_next[1])])
            t_val = - (v[0] * dir_prev[0] + v[1] * dir_prev[1])
            M = np.array([o1_1[0] + r * norm_prev[0] + t_val * dir_prev[0],
                          o1_1[1] + r * norm_prev[1] + t_val * dir_prev[1]])
            T1 = M - r * np.array(norm_prev)
            T2 = M - r * np.array(norm_next)

            ang1 = math.atan2(T1[1] - M[1], T1[0] - M[0])
            ang2 = math.atan2(T2[1] - M[1], T2[0] - M[0])
            while ang2 < ang1:
                ang2 += 2 * math.pi
            if ang2 - ang1 > math.pi:
                ang2 -= 2 * math.pi
            arc_steps = max(4, int(24 * abs(ang2 - ang1) / math.pi))
            for k in range(1, arc_steps):
                t = k / arc_steps
                a = ang1 + t * (ang2 - ang1)
                out.append((M[0] + r * math.cos(a), M[1] + r * math.sin(a)))

    return out

def sample_path_for_peaks(path_points, step):
    """Sample points along path with given step. step = distance between peaks."""
    if len(path_points) < 2 or step <= 0:
        return list(path_points)
    out = [path_points[0]]
    dist_along = 0.0
    next_sample_at = step
    for i in range(1, len(path_points)):
        p0, p1 = path_points[i - 1], path_points[i]
        seg_len = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if seg_len < 1e-10:
            continue
        seg_start = dist_along
        dist_along += seg_len
        while next_sample_at <= dist_along:
            t = (next_sample_at - seg_start) / seg_len
            t = max(0, min(1, t))
            px = p0[0] + t * (p1[0] - p0[0])
            py = p0[1] + t * (p1[1] - p0[1])
            out.append((px, py))
            next_sample_at += step
    return out


class CircleWorker(QThread):
    progress = pyqtSignal(int)
    result = pyqtSignal(list)

    def __init__(self, points, width, height, n_circles, min_radius):
        super().__init__()
        self.points = points
        self.width = width
        self.height = height
        self.n_circles = n_circles
        self.min_radius = min_radius

    def run(self):
        circles = []
        area_covered = 0
        max_attempts = 100
        area_target = 0.9 * self.calculate_polygon_area()

        for i in range(self.n_circles):
            for _ in range(max_attempts):
                x = np.random.randint(0, self.width)
                y = np.random.randint(0, self.height)
                r = np.random.randint(self.min_radius, 100)

                if self.is_circle_inside_polygon(x, y, r) and self.is_non_overlapping_circle(circles, x, y, r):
                    circles.append((x, y, r))
                    area_covered += np.pi * r ** 2
                    break

            self.progress.emit(int((len(circles) / self.n_circles) * 100))

            if area_covered >= area_target:
                break
        self.progress.emit(100)
        self.result.emit(circles)

    def calculate_polygon_area(self):
        if len(self.points) < 3:
            return 0
        area = 0
        n = len(self.points)
        for i in range(n):
            x1, y1 = self.points[i]
            x2, y2 = self.points[(i + 1) % n]
            area += x1 * y2 - x2 * y1
        return abs(area) / 2

    def is_circle_inside_polygon(self, x, y, r):
        for angle in np.linspace(0, 2 * np.pi, 36):
            px = x + r * np.cos(angle)
            py = y + r * np.sin(angle)
            if not self.is_point_inside_polygon((px, py)):
                return False
        return True

    def is_point_inside_polygon(self, point):
        x, y = point
        inside = False
        n = len(self.points)
        for i in range(n):
            x1, y1 = self.points[i]
            x2, y2 = self.points[(i + 1) % n]
            if ((y1 > y) != (y2 > y)) and (x < ((x2 - x1) * (y - y1) / (y2 - y1) + x1)):
                inside = not inside
        return inside

    def is_non_overlapping_circle(self, circles, x, y, r):
        for cx, cy, cr in circles:
            dist = np.hypot(cx - x, cy - y)
            if dist < r + cr:
                return False
        return True


class DrawingArea(QWidget):
    def __init__(self):
        super().__init__()
        self.points = []
        self.closed = False
        self.circles = []
        self.trajectory_center = []
        self.trajectory_inner = []
        self.trajectory_outer = []
        self.trajectory_peaks = []  # list of (x, y, sigma)
        self.trajectory_segment_end_indices = []  # segment_end_indices[i] = last pt index of segment i
        self.trajectory_n_inner = 0  # число пиков по внутренней границе (первые n_inner в trajectory_peaks)
        self.active_segment_index = None
        self._display_mode = "polygon"  # "polygon" | "trajectory"
        self._show_peaks = True
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_show_peaks(self, show):
        self._show_peaks = bool(show)
        self.update()

    def set_display_mode(self, mode):
        self._display_mode = mode
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        if self._display_mode == "trajectory":
            if self.trajectory_center or self.trajectory_peaks:
                self._paint_trajectory(painter)
        else:
            self._paint_polygon(painter)

    def _paint_polygon(self, painter):
        pen = QPen(QColor("blue"), 2)
        painter.setPen(pen)
        for i in range(1, len(self.points)):
            painter.drawLine(self.points[i - 1][0], self.points[i - 1][1],
                             self.points[i][0], self.points[i][1])
        if self.closed and len(self.points) > 2:
            painter.drawLine(self.points[-1][0], self.points[-1][1],
                             self.points[0][0], self.points[0][1])
            painter.setBrush(QBrush(QColor(0, 255, 0, 100)))
            painter.drawPolygon(*[QPoint(*p) for p in self.points])
        if self._show_peaks:
            painter.setBrush(QBrush(QColor(255, 0, 0, 150)))
            painter.setPen(QPen(QColor("red"), 1))
            for x, y, r in self.circles:
                r_draw = max(2, int(r))
                painter.drawEllipse(QPoint(int(x), int(y)), r_draw, r_draw)

    def _paint_trajectory(self, painter):
        if self.trajectory_center and len(self.trajectory_center) >= 2:
            painter.setPen(QPen(QColor("blue"), 2))
            for i in range(1, len(self.trajectory_center)):
                p0, p1 = self.trajectory_center[i - 1], self.trajectory_center[i]
                painter.drawLine(int(p0[0]), int(p0[1]), int(p1[0]), int(p1[1]))
        if self.trajectory_inner and len(self.trajectory_inner) >= 2:
            painter.setPen(QPen(QColor(0, 140, 0), 1))
            for i in range(1, len(self.trajectory_inner)):
                p0, p1 = self.trajectory_inner[i - 1], self.trajectory_inner[i]
                painter.drawLine(int(p0[0]), int(p0[1]), int(p1[0]), int(p1[1]))
        if self.trajectory_outer and len(self.trajectory_outer) >= 2:
            painter.setPen(QPen(QColor(0, 140, 0), 1))
            for i in range(1, len(self.trajectory_outer)):
                p0, p1 = self.trajectory_outer[i - 1], self.trajectory_outer[i]
                painter.drawLine(int(p0[0]), int(p0[1]), int(p1[0]), int(p1[1]))
        if self._show_peaks:
            painter.setBrush(QBrush(QColor(255, 0, 0, 150)))
            painter.setPen(QPen(QColor("red"), 1))
            for x, y, sigma in self.trajectory_peaks:
                r = max(2, min(sigma, 80))
                painter.drawEllipse(QPoint(int(x), int(y)), int(r), int(r))
        # Highlight active segment start and end points
        if (self.active_segment_index is not None and self.trajectory_segment_end_indices
                and 0 <= self.active_segment_index < len(self.trajectory_segment_end_indices)
                and self.trajectory_center):
            end_idx = self.trajectory_segment_end_indices[self.active_segment_index]
            start_idx = self.trajectory_segment_end_indices[self.active_segment_index - 1] if self.active_segment_index > 0 else 0
            for idx in (start_idx, end_idx):
                if 0 <= idx < len(self.trajectory_center):
                    px, py = self.trajectory_center[idx][0], self.trajectory_center[idx][1]
                    painter.setBrush(QBrush(QColor(255, 165, 0, 200)))
                    painter.setPen(QPen(QColor(200, 100, 0), 2))
                    r = 10
                    painter.drawEllipse(QPoint(int(px), int(py)), r, r)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self._display_mode == "polygon" and not self.closed:
            self.points.append((event.x(), event.y()))
            self.update()

    def close_shape(self):
        if len(self.points) > 2:
            self.closed = True
            self.update()

    def clear(self):
        self.points = []
        self.closed = False
        self.circles = []
        self.trajectory_center = []
        self.trajectory_inner = []
        self.trajectory_outer = []
        self.trajectory_peaks = []
        self.trajectory_segment_end_indices = []
        self.trajectory_n_inner = 0
        self.active_segment_index = None
        self.update()

    def set_circles(self, circles):
        self.circles = circles
        self.update()

    def set_trajectory_data(self, center, inner, outer, peaks, segment_end_indices=None, n_inner=0):
        self.trajectory_center = list(center)
        self.trajectory_inner = list(inner)
        self.trajectory_outer = list(outer)
        self.trajectory_peaks = list(peaks)
        self.trajectory_segment_end_indices = list(segment_end_indices) if segment_end_indices else []
        self.trajectory_n_inner = n_inner
        self.update()

    def set_active_segment(self, index):
        self.active_segment_index = index if index >= 0 else None
        self.update()

    def clear_trajectory(self):
        self.trajectory_center = []
        self.trajectory_inner = []
        self.trajectory_outer = []
        self.trajectory_peaks = []
        self.trajectory_segment_end_indices = []
        self.trajectory_n_inner = 0
        self.active_segment_index = None
        self.update()

    def _virtual_space_bounds(self, points_with_radius):
        """points_with_radius: list of (x, y, radius). Returns (left, top, width_v, height_v)."""
        if not points_with_radius:
            return 0, 0, 2 * BOUNDING_MARGIN, 2 * BOUNDING_MARGIN
        min_x = min(x - r for x, y, r in points_with_radius)
        max_x = max(x + r for x, y, r in points_with_radius)
        min_y = min(y - r for x, y, r in points_with_radius)
        max_y = max(y + r for x, y, r in points_with_radius)
        left = min_x - BOUNDING_MARGIN
        top = min_y - BOUNDING_MARGIN
        right = max_x + BOUNDING_MARGIN
        bottom = max_y + BOUNDING_MARGIN
        width_v = max(right - left, 2 * BOUNDING_MARGIN)
        height_v = max(bottom - top, 2 * BOUNDING_MARGIN)
        return left, top, width_v, height_v

    def _center_to_export_coords(self, x_center, y_center, left, top, width_v, height_v, spread):
        x_v = x_center - left
        y_v = y_center - top
        x0 = (x_v - width_v / 2) / spread
        y0 = (y_v - height_v / 2) / spread
        return x0, y0

    def save_circles_to_json(self, spread, scale, filename="peaks.json"):
        left, top, width_v, height_v = self._virtual_space_bounds(self.circles)
        circle_data = []
        for x, y, r in self.circles:
            amplitude = int((r / 100) * 30 + 10)
            x0, y0 = self._center_to_export_coords(x, y, left, top, width_v, height_v, spread)
            circle_data.append({
                "x0": int(x0),
                "y0": int(y0),
                "amplitude": amplitude,
                "sigma_x": r * scale,
                "sigma_y": r * scale
            })
        with open(filename, 'w') as f:
            json.dump(circle_data, f, indent=2)

    def save_trajectory_peaks_to_json(self, spread, scale, filename="peaks.json", amp_scale=5, invert_outer=False):
        points_with_radius = [(x, y, sigma) for x, y, sigma in self.trajectory_peaks]
        left, top, width_v, height_v = self._virtual_space_bounds(points_with_radius)
        peak_data = []
        # Базовый amplitude: шкала 1–500 -> целевая суммарная высота уменьшена в amp_scale раз
        SIGMA_MIN, SIGMA_MAX = 1, 500
        peaks_list = list(self.trajectory_peaks)
        n_inner = getattr(self, 'trajectory_n_inner', 0)
        # Сколько окружностей (других пиков) пересекаются с окружностью текущего пика.
        # Две окружности пересекаются/касаются, если расстояние между центрами <= r1 + r2.
        def count_intersecting_circles(px, py, psigma):
            n = 0
            for x, y, sigmaj in peaks_list:
                d = math.hypot(x - px, y - py)
                if d <= psigma + sigmaj:
                    n += 1
            return max(1, n)
        for i, (x, y, sigma) in enumerate(peaks_list):
            size_norm = min(100, max(0, (sigma - SIGMA_MIN) / (SIGMA_MAX - SIGMA_MIN) * 100))
            a_base = ((size_norm / 100) * 90 + 10) / amp_scale
            n_intersecting = count_intersecting_circles(x, y, sigma)
            # Делим на число пересекающихся окружностей; итог в диапазоне 1–100
            amplitude = int(max(1, min(100, round(a_base / n_intersecting))))
            if invert_outer and i >= n_inner:
                amplitude = -amplitude
            x0, y0 = self._center_to_export_coords(x, y, left, top, width_v, height_v, spread)
            peak_data.append({
                "x0": int(x0),
                "y0": int(y0),
                "amplitude": amplitude,
                "sigma_x": sigma * scale,
                "sigma_y": sigma * scale
            })
        with open(filename, 'w') as f:
            json.dump(peak_data, f, indent=2)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Space Generator — Область / Траектория")
        self.setGeometry(50, 50, 1200, 700)
        self._settings_panel_width = 440
        self.segments = []  # list of {"type": "line"|"arc", ...}

        # --- Left part: drawing area ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(4, 4, 4, 4)

        mode_group = QButtonGroup()
        self.mode_polygon = QRadioButton("Область (полигон)")
        self.mode_trajectory = QRadioButton("Траектория")
        self.mode_polygon.setChecked(True)
        mode_group.addButton(self.mode_polygon)
        mode_group.addButton(self.mode_trajectory)
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Режим:"))
        mode_layout.addWidget(self.mode_polygon)
        mode_layout.addWidget(self.mode_trajectory)
        mode_layout.addStretch()
        self.show_peaks_cb = QCheckBox("Пики")
        self.show_peaks_cb.setChecked(True)
        self.show_peaks_cb.stateChanged.connect(
            lambda s: self.drawing_area.set_show_peaks(s == Qt.Checked)
        )
        mode_layout.addWidget(self.show_peaks_cb)
        target_iso_row = QHBoxLayout()
        target_iso_row.addWidget(QLabel("Уровень целевой изолинии:"))
        self.target_isoline_spin = QDoubleSpinBox()
        self.target_isoline_spin.setRange(-1000, 10000)
        self.target_isoline_spin.setValue(0)
        self.target_isoline_spin.setDecimals(1)
        self.target_isoline_spin.setToolTip("Высота плоскости целевой изолинии на 2D графике; уровень 0 по Z сдвинут на это значение")
        target_iso_row.addWidget(self.target_isoline_spin)
        mode_layout.addLayout(target_iso_row)
        self.preview_field_btn = QPushButton("Превью поля")
        self.preview_field_btn.setStyleSheet("background-color: #c8e6c9;")
        self.preview_field_btn.clicked.connect(self.preview_field)
        mode_layout.addWidget(self.preview_field_btn)
        self.preview_field_2d_btn = QPushButton("Превью 2D")
        self.preview_field_2d_btn.setStyleSheet("background-color: #b3e5fc;")
        self.preview_field_2d_btn.clicked.connect(self.preview_field_2d)
        mode_layout.addWidget(self.preview_field_2d_btn)
        self.toggle_panel_btn = QPushButton("◀ Скрыть настройки")
        self.toggle_panel_btn.setStyleSheet("background-color: #ddd;")
        self.toggle_panel_btn.clicked.connect(self.toggle_settings_panel)
        mode_layout.addWidget(self.toggle_panel_btn)
        left_layout.addLayout(mode_layout)

        frame = QFrame()
        frame.setFrameShape(QFrame.Box)
        frame.setLineWidth(2)
        frame.setStyleSheet("border: 2px solid black;")
        frame_layout = QVBoxLayout(frame)
        frame_layout.setContentsMargins(0, 0, 0, 0)
        self.drawing_area = DrawingArea()
        self.drawing_area.setMinimumSize(400, 400)
        frame_layout.addWidget(self.drawing_area)
        frame.setLayout(frame_layout)
        left_layout.addWidget(frame, 1)

        # --- Right part: settings (collapsible) ---
        self.settings_panel = QWidget()
        self.settings_panel.setMinimumWidth(340)
        self.settings_panel.setMaximumWidth(600)
        settings_layout = QVBoxLayout(self.settings_panel)
        settings_layout.setContentsMargins(4, 4, 4, 4)

        settings_scroll = QScrollArea()
        settings_scroll.setWidgetResizable(True)
        settings_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        settings_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        settings_content = QWidget()
        settings_content.setMinimumWidth(380)
        settings_inner = QVBoxLayout(settings_content)
        settings_inner.setContentsMargins(0, 0, 0, 0)

        self.stacked = QStackedWidget()

        # --- Page 0: Polygon mode ---
        polygon_page = QWidget()
        poly_layout = QVBoxLayout()

        self.clear_button = QPushButton("Clear")
        self.clear_button.setStyleSheet("background-color: lightgray;")
        self.clear_button.clicked.connect(self.clear_drawing)

        self.closure_button = QPushButton("Close Shape")
        self.closure_button.setStyleSheet("background-color: lightgray;")
        self.closure_button.clicked.connect(self.close_shape)

        self.circles_button = QPushButton("Circles")
        self.circles_button.setStyleSheet("background-color: lightgray;")
        self.circles_button.clicked.connect(self.generate_circles)
        self.circles_button.setEnabled(False)

        self.radio_group = QButtonGroup()
        self.radio2 = QRadioButton("1:2")
        self.radio4 = QRadioButton("1:4")
        self.radio8 = QRadioButton("1:8")
        self.radio_group.addButton(self.radio2, 2)
        self.radio_group.addButton(self.radio4, 4)
        self.radio_group.addButton(self.radio8, 8)
        radio_layout = QHBoxLayout()
        radio_layout.addWidget(self.radio2)
        radio_layout.addWidget(self.radio4)
        radio_layout.addWidget(self.radio8)

        self.save_button = QPushButton("Save Circles to JSON")
        self.save_button.setStyleSheet("background-color: lightgray;")
        self.save_button.clicked.connect(self.save_circles)
        self.save_button.setEnabled(False)

        slider_label = QLabel("Num Circles:")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(10)
        self.slider.setMaximum(500)
        self.slider.setValue(50)
        self.slider_value_label = QLabel("50")
        self.slider.valueChanged.connect(self.update_slider_label)

        min_radius_label = QLabel("Min Radius:")
        self.min_radius_slider = QSlider(Qt.Horizontal)
        self.min_radius_slider.setMinimum(2)
        self.min_radius_slider.setMaximum(20)
        self.min_radius_slider.setValue(5)
        self.min_radius_slider_value_label = QLabel("5")
        self.min_radius_slider.valueChanged.connect(self.update_min_radius_slider_label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)

        slider_layout = QHBoxLayout()
        slider_layout.addWidget(slider_label)
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.slider_value_label)
        min_radius_layout = QHBoxLayout()
        min_radius_layout.addWidget(min_radius_label)
        min_radius_layout.addWidget(self.min_radius_slider)
        min_radius_layout.addWidget(self.min_radius_slider_value_label)

        buttons_layout = QHBoxLayout()
        buttons_layout.addWidget(self.clear_button)
        buttons_layout.addWidget(self.closure_button)
        buttons_layout.addWidget(self.circles_button)
        buttons_layout.addWidget(self.save_button)

        poly_layout.addLayout(buttons_layout)
        poly_layout.addLayout(radio_layout)
        poly_layout.addLayout(slider_layout)
        poly_layout.addLayout(min_radius_layout)
        poly_layout.addWidget(self.progress_bar)
        polygon_page.setLayout(poly_layout)
        self.stacked.addWidget(polygon_page)

        # --- Page 1: Trajectory mode ---
        traj_page = QWidget()
        traj_layout = QVBoxLayout()

        start_grp = QGroupBox("Начало траектории")
        start_form = QFormLayout()
        self.start_x_spin = QDoubleSpinBox()
        self.start_x_spin.setRange(-10000, 10000)
        self.start_x_spin.setValue(300)
        self.start_y_spin = QDoubleSpinBox()
        self.start_y_spin.setRange(-10000, 10000)
        self.start_y_spin.setValue(300)
        self.start_angle_spin = QDoubleSpinBox()
        self.start_angle_spin.setRange(-360, 360)
        self.start_angle_spin.setValue(0)
        self.start_angle_spin.setSuffix(" °")
        start_form.addRow("X:", self.start_x_spin)
        start_form.addRow("Y:", self.start_y_spin)
        start_form.addRow("Угол (направление):", self.start_angle_spin)
        start_grp.setLayout(start_form)
        traj_layout.addWidget(start_grp)

        seg_grp = QGroupBox("Сегменты")
        seg_hl = QHBoxLayout()
        self.segment_list = QListWidget()
        self.segment_list.setMinimumHeight(280)
        self.segment_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.segment_list.currentRowChanged.connect(self.on_segment_selected)
        seg_hl.addWidget(self.segment_list)
        seg_btns = QVBoxLayout()
        add_line_btn = QPushButton("+ Линия")
        add_line_btn.clicked.connect(self.add_line_segment)
        add_arc_btn = QPushButton("+ Дуга")
        add_arc_btn.clicked.connect(self.add_arc_segment)
        remove_seg_btn = QPushButton("Удалить")
        remove_seg_btn.clicked.connect(self.remove_segment)
        seg_btns.addWidget(add_line_btn)
        seg_btns.addWidget(add_arc_btn)
        seg_btns.addWidget(remove_seg_btn)
        seg_btns.addStretch()
        seg_hl.addLayout(seg_btns)
        seg_grp.setLayout(seg_hl)
        traj_layout.addWidget(seg_grp, 1)  # stretch so list height matches frame

        params_grp = QGroupBox("Параметры выбранного сегмента")
        params_form = QFormLayout()
        self.seg_length_spin = QDoubleSpinBox()
        self.seg_length_spin.setRange(1, 10000)
        self.seg_length_spin.setValue(100)
        self.seg_angle_spin = QDoubleSpinBox()
        self.seg_angle_spin.setRange(-360, 360)
        self.seg_angle_spin.setValue(0)
        self.seg_angle_spin.setSuffix(" °")
        self.seg_radius_spin = QDoubleSpinBox()
        self.seg_radius_spin.setRange(1, 1000)
        self.seg_radius_spin.setValue(50)
        self.seg_arc_angle_spin = QDoubleSpinBox()
        self.seg_arc_angle_spin.setRange(-360, 360)
        self.seg_arc_angle_spin.setValue(90)
        self.seg_arc_angle_spin.setSuffix(" °")
        self.seg_direction_combo = QComboBox()
        self.seg_direction_combo.addItems(["left", "right"])
        self.seg_length_spin.valueChanged.connect(self.apply_segment_params)
        self.seg_angle_spin.valueChanged.connect(self.apply_segment_params)
        self.seg_radius_spin.valueChanged.connect(self.apply_segment_params)
        self.seg_arc_angle_spin.valueChanged.connect(self.apply_segment_params)
        self.seg_direction_combo.currentIndexChanged.connect(self.apply_segment_params)
        params_form.addRow("Длина (линия):", self.seg_length_spin)
        params_form.addRow("Угол (линия), °:", self.seg_angle_spin)
        params_form.addRow("Радиус (дуга):", self.seg_radius_spin)
        params_form.addRow("Угол дуги, °:", self.seg_arc_angle_spin)
        params_form.addRow("Направление дуги:", self.seg_direction_combo)
        params_grp.setLayout(params_form)
        traj_layout.addWidget(params_grp)

        offset_grp = QGroupBox("Смещение и скругление")
        off_form = QFormLayout()
        self.offset_spin = QDoubleSpinBox()
        self.offset_spin.setRange(1, 500)
        self.offset_spin.setValue(30)
        self.rounding_combo = QComboBox()
        self.rounding_combo.addItems(["Единый радиус", "Радиус от угла (L_max·tan(θ/2))"])
        self.rounding_combo.currentIndexChanged.connect(self.toggle_rounding_params)
        self.single_r_spin = QDoubleSpinBox()
        self.single_r_spin.setRange(0.1, 500)
        self.single_r_spin.setValue(20)
        self.l_max_spin = QDoubleSpinBox()
        self.l_max_spin.setRange(0, 500)
        self.l_max_spin.setValue(30)
        off_form.addRow("Смещение (ширина коридора):", self.offset_spin)
        off_form.addRow("Скругление:", self.rounding_combo)
        off_form.addRow("Радиус скругления (единый):", self.single_r_spin)
        off_form.addRow("L_max (макс. срез):", self.l_max_spin)
        offset_grp.setLayout(off_form)
        traj_layout.addWidget(offset_grp)

        step_grp = QGroupBox("Пики вдоль траектории")
        step_form = QFormLayout()
        self.peak_step_spin = QDoubleSpinBox()
        self.peak_step_spin.setRange(5, 500)
        self.peak_step_spin.setValue(25)
        step_form.addRow("Шаг между пиками:", self.peak_step_spin)
        self.amp_scale_spin = QDoubleSpinBox()
        self.amp_scale_spin.setRange(1, 50)
        self.amp_scale_spin.setValue(5)
        self.amp_scale_spin.setToolTip("Во сколько раз уменьшить амплитуду пиков (1 = без уменьшения)")
        step_form.addRow("Масштаб амплитуды (÷):", self.amp_scale_spin)
        self.invert_outer_check = QCheckBox("Инвертировать внешние пики (два поля по разные стороны от целевой изолинии)")
        self.invert_outer_check.setToolTip("Внешние пики сохраняются с отрицательной амплитудой; изолиния Z=0 разделяет два поля")
        self.invert_outer_check.setChecked(False)
        step_form.addRow("", self.invert_outer_check)
        step_grp.setLayout(step_form)
        traj_layout.addWidget(step_grp)

        traj_buttons = QHBoxLayout()
        self.gen_traj_button = QPushButton("Построить траекторию и пики")
        self.gen_traj_button.setStyleSheet("background-color: lightblue;")
        self.gen_traj_button.clicked.connect(self.generate_trajectory_peaks)
        self.save_traj_button = QPushButton("Сохранить пики в JSON")
        self.save_traj_button.setStyleSheet("background-color: lightgray;")
        self.save_traj_button.clicked.connect(self.save_trajectory)
        self.save_traj_button.setEnabled(False)
        traj_buttons.addWidget(self.gen_traj_button)
        traj_buttons.addWidget(self.save_traj_button)
        traj_layout.addLayout(traj_buttons)

        traj_radio = QHBoxLayout()
        traj_radio.addWidget(QLabel("Масштаб сохранения:"))
        traj_radio.addWidget(self.radio2)
        traj_radio.addWidget(self.radio4)
        traj_radio.addWidget(self.radio8)
        traj_layout.addLayout(traj_radio)

        traj_page.setLayout(traj_layout)
        self.stacked.addWidget(traj_page)

        settings_inner.addWidget(self.stacked)

        exit_button = QPushButton("Exit")
        exit_button.setStyleSheet("background-color: red; color: white;")
        exit_button.clicked.connect(self.close)
        settings_inner.addWidget(exit_button)

        settings_scroll.setWidget(settings_content)
        settings_layout.addWidget(settings_scroll, 1)

        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(left_widget)
        self.splitter.addWidget(self.settings_panel)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setSizes([800, self._settings_panel_width])
        self.splitter.setChildrenCollapsible(False)

        self.mode_polygon.toggled.connect(self.on_mode_changed)
        self.mode_trajectory.toggled.connect(self.on_mode_changed)
        self.on_mode_changed()
        self.toggle_rounding_params()

        self.setCentralWidget(self.splitter)

    def toggle_settings_panel(self):
        if self.settings_panel.isVisible():
            self._saved_panel_width = self.splitter.sizes()[1]
            self.settings_panel.hide()
            self.toggle_panel_btn.setText("Настройки ▶")
        else:
            self.settings_panel.show()
            w = getattr(self, "_saved_panel_width", self._settings_panel_width)
            total = self.splitter.width()
            self.splitter.setSizes([max(100, total - w), w])
            self.toggle_panel_btn.setText("◀ Скрыть настройки")

    def _get_spread_and_scale(self):
        spread = 2
        for button in self.radio_group.buttons():
            if button.isChecked():
                spread = self.radio_group.id(button)
                break
        scale = 0.8 if spread == 2 else (0.4 if spread == 4 else 0.2)
        return spread, scale

    def _peaks_bounds_from_json(self, json_path, margin_units=5, sigma_factor=3):
        """Read peaks JSON and return (x_range, y_range) for space so the field fits."""
        with open(json_path, 'r') as f:
            data = json.load(f)
        if not data:
            return (-80, 80), (-80, 80)
        xs = [p["x0"] for p in data]
        ys = [p["y0"] for p in data]
        sig_x = max(p.get("sigma_x", 1) for p in data)
        sig_y = max(p.get("sigma_y", 1) for p in data)
        pad_x = sigma_factor * sig_x + margin_units
        pad_y = sigma_factor * sig_y + margin_units
        min_x = min(xs) - pad_x
        max_x = max(xs) + pad_x
        min_y = min(ys) - pad_y
        max_y = max(ys) + pad_y
        if max_x - min_x < 10:
            c = (min_x + max_x) / 2
            min_x, max_x = c - 5, c + 5
        if max_y - min_y < 10:
            c = (min_y + max_y) / 2
            min_y, max_y = c - 5, c + 5
        return (min_x, max_x), (min_y, max_y)

    def preview_field(self):
        spread, scale = self._get_spread_and_scale()
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".json", prefix="space_preview_")
        try:
            os.close(tmp_fd)
            if self.mode_polygon.isChecked():
                if not self.drawing_area.circles:
                    QMessageBox.information(
                        self, "Превью поля",
                        "Нет данных для превью. Замкните полигон и нажмите «Circles»."
                    )
                    return
                self.drawing_area.save_circles_to_json(spread, scale, tmp_path)
            else:
                if not self.drawing_area.trajectory_peaks:
                    QMessageBox.information(
                        self, "Превью поля",
                        "Нет данных для превью. Постройте траекторию и пики."
                    )
                    return
                self.drawing_area.save_trajectory_peaks_to_json(
                    spread, scale, tmp_path, amp_scale=self.amp_scale_spin.value(),
                    invert_outer=self.invert_outer_check.isChecked()
                )
            import spaces as sp
            x_range, y_range = self._peaks_bounds_from_json(tmp_path)
            grid_size = 300
            space = sp.create_instance(
                "gaussian",
                x_range=x_range,
                y_range=y_range,
                grid_size=grid_size,
                shift_xyz=None,
                space_filename=tmp_path,
                target_isoline=self.target_isoline_spin.value(),
            )
            space.plotting_surface(store_plot=False)
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    def preview_field_2d(self):
        """Export current peaks to temp JSON and show 2D isolines plot (as in OtterAndOil)."""
        spread, scale = self._get_spread_and_scale()
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".json", prefix="space_preview_")
        try:
            os.close(tmp_fd)
            if self.mode_polygon.isChecked():
                if not self.drawing_area.circles:
                    QMessageBox.information(
                        self, "Превью 2D",
                        "Нет данных для превью. Замкните полигон и нажмите «Circles»."
                    )
                    return
                self.drawing_area.save_circles_to_json(spread, scale, tmp_path)
            else:
                if not self.drawing_area.trajectory_peaks:
                    QMessageBox.information(
                        self, "Превью 2D",
                        "Нет данных для превью. Постройте траекторию и пики."
                    )
                    return
                self.drawing_area.save_trajectory_peaks_to_json(
                    spread, scale, tmp_path, amp_scale=self.amp_scale_spin.value(),
                    invert_outer=self.invert_outer_check.isChecked()
                )
            import spaces as sp
            import matplotlib.pyplot as plt
            x_range, y_range = self._peaks_bounds_from_json(tmp_path)
            grid_size = 300
            isolines_count = 10
            target_isoline = self.target_isoline_spin.value()
            space = sp.create_instance(
                "gaussian",
                x_range=x_range,
                y_range=y_range,
                grid_size=grid_size,
                shift_xyz=None,
                space_filename=tmp_path,
                target_isoline=target_isoline,
            )
            fig, ax = plt.subplots(figsize=(8, 8))
            contour = ax.contour(
                space.get_X(), space.get_Y(), space.get_Z(),
                levels=isolines_count, cmap='viridis'
            )
            ax.clabel(contour, inline=True)
            ax.contour(
                space.get_X(), space.get_Y(), space.get_Z(),
                levels=[space.target_isoline], colors='red', linewidths=2
            )
            # Центры пиков
            if space.peaks:
                ax.scatter(
                    [p.x0 for p in space.peaks], [p.y0 for p in space.peaks],
                    c='black', s=20, marker='o', zorder=5, label='Центры пиков'
                )
            # Центральная линия траектории (только в режиме траектории)
            if self.mode_trajectory.isChecked() and getattr(self.drawing_area, 'trajectory_center', None) and len(self.drawing_area.trajectory_center) >= 2:
                pts = [(x, y, sigma) for x, y, sigma in self.drawing_area.trajectory_peaks]
                left, top, width_v, height_v = self.drawing_area._virtual_space_bounds(pts)
                center_export = []
                for x, y in self.drawing_area.trajectory_center:
                    x0, y0 = self.drawing_area._center_to_export_coords(x, y, left, top, width_v, height_v, spread)
                    center_export.append((x0, y0))
                ax.plot(
                    [p[0] for p in center_export], [p[1] for p in center_export],
                    'b-', linewidth=2, zorder=4, label='Целевая траектория'
                )
            ax.set_xlabel('X, m / East')
            ax.set_ylabel('Y, m / North')
            ax.set_title('Поле (изолинии)')
            ax.set_aspect('equal', adjustable='box')
            from matplotlib.lines import Line2D
            handles, labels = ax.get_legend_handles_labels()
            handles.append(Line2D([0], [0], color='red', linewidth=2))
            labels.append('Целевая изолиния')
            ax.legend(handles=handles, labels=labels, loc='best', fontsize=8)
            plt.tight_layout()
            plt.show()
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    def on_mode_changed(self):
        if self.mode_polygon.isChecked():
            self.stacked.setCurrentIndex(0)
            self.drawing_area.set_display_mode("polygon")
        else:
            self.stacked.setCurrentIndex(1)
            self.drawing_area.set_display_mode("trajectory")

    def toggle_rounding_params(self):
        use_angle = self.rounding_combo.currentIndex() == 1
        self.l_max_spin.setEnabled(use_angle)
        self.single_r_spin.setEnabled(not use_angle or True)

    def add_line_segment(self):
        self.segments.append({"type": "line", "length": 100, "angle": 0})
        self._refresh_segment_list()

    def add_arc_segment(self):
        self.segments.append({"type": "arc", "radius": 50, "angle": 90, "direction": "left"})
        self._refresh_segment_list()

    def remove_segment(self):
        row = self.segment_list.currentRow()
        if 0 <= row < len(self.segments):
            self.segments.pop(row)
            self._refresh_segment_list()

    def _refresh_segment_list(self):
        self.segment_list.blockSignals(True)
        self.segment_list.clear()
        for i, s in enumerate(self.segments):
            if s.get("type") == "line":
                txt = "Line: L={} A={}°".format(s.get("length", 0), s.get("angle", 0))
            else:
                txt = "Arc: R={} A={}° {}".format(s.get("radius", 0), s.get("angle", 0), s.get("direction", "left"))
            self.segment_list.addItem(txt)
        self.segment_list.blockSignals(False)
        if self.segments:
            self.segment_list.setCurrentRow(0)
            self.on_segment_selected(0)

    def on_segment_selected(self, row):
        self.drawing_area.set_active_segment(row if row >= 0 else None)
        if row < 0 or row >= len(self.segments):
            return
        s = self.segments[row]
        self.seg_length_spin.blockSignals(True)
        self.seg_angle_spin.blockSignals(True)
        self.seg_radius_spin.blockSignals(True)
        self.seg_arc_angle_spin.blockSignals(True)
        self.seg_direction_combo.blockSignals(True)
        self.seg_length_spin.setValue(float(s.get("length", 100)))
        self.seg_angle_spin.setValue(float(s.get("angle", 0)))
        self.seg_radius_spin.setValue(float(s.get("radius", 50)))
        self.seg_arc_angle_spin.setValue(float(s.get("angle", 90)) if s.get("type") == "arc" else 90)
        idx = self.seg_direction_combo.findText(s.get("direction", "left"))
        if idx >= 0:
            self.seg_direction_combo.setCurrentIndex(idx)
        self.seg_length_spin.setEnabled(s.get("type") == "line")
        self.seg_angle_spin.setEnabled(s.get("type") == "line")
        self.seg_radius_spin.setEnabled(s.get("type") == "arc")
        self.seg_arc_angle_spin.setEnabled(s.get("type") == "arc")
        self.seg_direction_combo.setEnabled(s.get("type") == "arc")
        self.seg_length_spin.blockSignals(False)
        self.seg_angle_spin.blockSignals(False)
        self.seg_radius_spin.blockSignals(False)
        self.seg_arc_angle_spin.blockSignals(False)
        self.seg_direction_combo.blockSignals(False)

    def apply_segment_params(self):
        row = self.segment_list.currentRow()
        if row < 0 or row >= len(self.segments):
            return
        s = self.segments[row]
        if s.get("type") == "line":
            s["length"] = self.seg_length_spin.value()
            s["angle"] = self.seg_angle_spin.value()
        else:
            s["radius"] = self.seg_radius_spin.value()
            s["angle"] = self.seg_arc_angle_spin.value()
            s["direction"] = self.seg_direction_combo.currentText()
        self._refresh_segment_list()
        self.segment_list.setCurrentRow(row)

    def generate_trajectory_peaks(self):
        if not self.segments:
            return
        cx = self.drawing_area.width() / 2
        cy = self.drawing_area.height() / 2
        start_x = self.start_x_spin.value()
        start_y = self.start_y_spin.value()
        start_angle = self.start_angle_spin.value()
        center, segment_end_indices = build_center_polyline(self.segments, start_x, start_y, start_angle)
        if len(center) < 2:
            return
        offset_d = self.offset_spin.value()
        rounding_mode = "single" if self.rounding_combo.currentIndex() == 0 else "angle"
        single_r = self.single_r_spin.value()
        l_max = self.l_max_spin.value() if rounding_mode == "angle" else None
        r_base_sin = single_r
        inner = offset_polyline_with_rounding(
            center, offset_d, rounding_mode, single_r, r_base_sin, l_max, "left"
        )
        outer = offset_polyline_with_rounding(
            center, offset_d, rounding_mode, single_r, r_base_sin, l_max, "right"
        )
        step = self.peak_step_spin.value()
        inner_pts = sample_path_for_peaks(inner, step)
        outer_pts = sample_path_for_peaks(outer, step)
        peaks = []
        for x, y in inner_pts:
            peaks.append((x, y, offset_d))
        for x, y in outer_pts:
            peaks.append((x, y, offset_d))
        self.drawing_area.set_trajectory_data(center, inner, outer, peaks, segment_end_indices, n_inner=len(inner_pts))
        row = self.segment_list.currentRow()
        self.drawing_area.set_active_segment(row if row >= 0 else None)
        self.save_traj_button.setEnabled(True)

    def save_trajectory(self):
        spread = 2
        for button in self.radio_group.buttons():
            if button.isChecked():
                spread = self.radio_group.id(button)
                break
        scale = 0.8 if spread == 2 else (0.4 if spread == 4 else 0.2)
        self.drawing_area.save_trajectory_peaks_to_json(
            spread, scale, amp_scale=self.amp_scale_spin.value(),
            invert_outer=self.invert_outer_check.isChecked()
        )

    def generate_circles(self):
        n_circles = self.slider.value()
        min_radius = self.min_radius_slider.value()
        points = self.drawing_area.points
        width = self.drawing_area.width()
        height = self.drawing_area.height()

        if not points or not self.drawing_area.closed:
            return

        self.worker = CircleWorker(points, width, height, n_circles, min_radius)
        self.worker.progress.connect(self.progress_bar.setValue)
        self.worker.result.connect(self.drawing_area.set_circles)
        self.worker.result.connect(self.enable_save_button)
        self.worker.start()

    def save_circles(self):
        spread = 0
        for button in self.radio_group.buttons():
            if button.isChecked():
                spread = self.radio_group.id(button)
                break
        if spread == 2:
            scale = 0.8
        elif spread == 4:
            scale = 0.4
        elif spread == 8:
            scale = 0.2
        self.drawing_area.save_circles_to_json(spread, scale)

    def clear_drawing(self):
        self.drawing_area.clear()
        self.circles_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self.progress_bar.setValue(0)

    def close_shape(self):
        self.drawing_area.close_shape()
        self.circles_button.setEnabled(True)

    def enable_save_button(self, circles):
        self.save_button.setEnabled(True)

    def update_min_radius_slider_label(self, value):
        self.min_radius_slider_value_label.setText(str(value))

    def update_slider_label(self, value):
        self.slider_value_label.setText(str(value))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())
