#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import json

from PIL.ImageOps import scale
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel,
    QSlider, QHBoxLayout, QProgressBar, QFrame, QButtonGroup, QRadioButton,
    QSizePolicy
)
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtCore import Qt, QPoint, QThread, pyqtSignal


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
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

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

        painter.setBrush(QBrush(QColor(255, 0, 0, 150)))
        painter.setPen(QPen(QColor("red"), 1))
        for x, y, r in self.circles:
            painter.drawEllipse(QPoint(x, y), r, r)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and not self.closed:
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
        self.update()

    def set_circles(self, circles):
        self.circles = circles
        self.update()

    def save_circles_to_json(self, spread, scale, filename="peaks.json"):
        # Convert circles to the required format and save to JSON
        circle_data = []
        for x, y, r in self.circles:
            amplitude = int((r / 100) * 30 + 10)  # Amplitude is proportional to the radius, max is 50 for max radius
            circle_data.append({
                "x0": int((x - self.width() / 2) / spread),
                "y0": int((y - self.height() / 2) / spread),
                "amplitude": amplitude,
                "sigma_x": r * scale,
                "sigma_y": r * scale
            })

        # Write to JSON file
        with open(filename, 'w') as f:
            json.dump(circle_data, f, indent=2)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drawing Area with Circles")
        self.width = 600
        self.height = 600
        self.setGeometry(100, 100, self.width, self.height)

        main_layout = QVBoxLayout()

        # Create a frame to hold the drawing area
        frame = QFrame()
        frame.setFrameShape(QFrame.Box)
        frame.setLineWidth(2)
        frame.setStyleSheet("border: 2px solid black;")

        frame_layout = QVBoxLayout()
        frame_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        frame_layout.setSpacing(0)  # Remove spacing

        self.drawing_area = DrawingArea()
        frame_layout.addWidget(self.drawing_area)
        frame.setLayout(frame_layout)

        self.clear_button = QPushButton("Clear")
        self.clear_button.setStyleSheet("background-color: lightgray;")
        self.clear_button.clicked.connect(self.clear_drawing)

        self.closure_button = QPushButton("Close Shape")
        self.closure_button.setStyleSheet("background-color: lightgray;")
        self.closure_button.clicked.connect(self.close_shape)

        self.circles_button = QPushButton("Circles")
        self.circles_button.setStyleSheet("background-color: lightgray;")
        self.circles_button.clicked.connect(self.generate_circles)
        self.circles_button.setEnabled(False)  # Initially inactive

        # Radio Buttons
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
        self.save_button.setEnabled(False)  # Initially inactive

        # Number of Circles slider
        slider_label = QLabel("Num Circles:")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(10)
        self.slider.setMaximum(500)
        self.slider.setValue(50)
        self.slider_value_label = QLabel("50")
        self.slider.valueChanged.connect(self.update_slider_label)

        # Minimal Radius slider
        min_radius_label = QLabel("Min Radius:")
        self.min_radius_slider = QSlider(Qt.Horizontal)
        self.min_radius_slider.setMinimum(2)
        self.min_radius_slider.setMaximum(20)
        self.min_radius_slider.setValue(5)
        self.min_radius_slider_value_label = QLabel("5")
        self.min_radius_slider.valueChanged.connect(self.update_min_radius_slider_label)

        # Progress Bar of drawing circles
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

        # Exit button
        exit_button = QPushButton("Exit")
        exit_button.setStyleSheet("background-color: red; color: white;")
        exit_button.clicked.connect(self.close)

        main_layout.addWidget(frame)  # Add the framed drawing area
        main_layout.addLayout(buttons_layout)
        main_layout.addLayout(radio_layout)
        main_layout.addLayout(slider_layout)
        main_layout.addLayout(min_radius_layout)
        main_layout.addWidget(self.progress_bar)
        main_layout.addWidget(exit_button)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

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
    window.show()
    sys.exit(app.exec_())