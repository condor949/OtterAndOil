import sys
import numpy as np
import json
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QSlider, QHBoxLayout, QProgressBar
)
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtCore import Qt, QPoint, QThread, pyqtSignal


class CircleWorker(QThread):
    progress = pyqtSignal(int)
    result = pyqtSignal(list)

    def __init__(self, points, width, height, n_circles):
        super().__init__()
        self.points = points
        self.width = width
        self.height = height
        self.n_circles = n_circles

    def run(self):
        circles = []
        area_covered = 0
        max_attempts = 100
        area_target = 0.9 * self.calculate_polygon_area()

        for i in range(self.n_circles):
            for _ in range(max_attempts):
                x = np.random.randint(0, self.width)
                y = np.random.randint(0, self.height)
                r = np.random.randint(20, 100)

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
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
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
        self.setFixedSize(500, 400)
        self.points = []
        self.closed = False
        self.circles = []

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

    def save_circles_to_json(self, filename="peaks.json"):
        # Convert circles to the required format and save to JSON
        circle_data = []
        for x, y, r in self.circles:
            amplitude = (r / 100) * 30 # Amplitude is proportional to the radius, max is 50 for max radius
            circle_data.append({
                "x0": int((x-250)/8),
                "y0": int((y-300)/8),
                "amplitude": 10+int(amplitude),
                "sigma_x": r*0.2,
                "sigma_y": r*0.2
            })

        # Write to JSON file
        with open(filename, 'w') as f:
            json.dump(circle_data, f, indent=2)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drawing Area with Circles")
        self.setGeometry(100, 100, 600, 500)

        main_layout = QVBoxLayout()
        self.drawing_area = DrawingArea()

        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.drawing_area.clear)

        closure_button = QPushButton("Close Shape")
        closure_button.clicked.connect(self.drawing_area.close_shape)

        circles_button = QPushButton("Circles")
        circles_button.clicked.connect(self.generate_circles)

        save_button = QPushButton("Save Circles to JSON")
        save_button.clicked.connect(self.save_circles)

        slider_label = QLabel("Number of Circles:")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(10)
        self.slider.setMaximum(500)
        self.slider.setValue(50)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)

        slider_layout = QHBoxLayout()
        slider_layout.addWidget(slider_label)
        slider_layout.addWidget(self.slider)

        main_layout.addWidget(self.drawing_area)
        main_layout.addWidget(clear_button)
        main_layout.addWidget(closure_button)
        main_layout.addWidget(circles_button)
        main_layout.addWidget(save_button)
        main_layout.addLayout(slider_layout)
        main_layout.addWidget(self.progress_bar)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def generate_circles(self):
        n_circles = self.slider.value()
        points = self.drawing_area.points
        width = self.drawing_area.width()
        height = self.drawing_area.height()

        if not points or not self.drawing_area.closed:
            return

        self.worker = CircleWorker(points, width, height, n_circles)
        self.worker.progress.connect(self.progress_bar.setValue)
        self.worker.result.connect(self.drawing_area.set_circles)
        self.worker.start()

    def save_circles(self):
        self.drawing_area.save_circles_to_json()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
