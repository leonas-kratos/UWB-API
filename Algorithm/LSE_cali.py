import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# ===== CALIBRATION PARAMETERS =====
# Mô hình hiệu chỉnh tuyến tính: y = slope * x + offset
CALIBRATION_MODELS = {
    '4795': {'slope': 0.98451916, 'offset': 198.104},
    'c91a': {'slope': 0.98262259, 'offset': 202.704},
    '1912': {'slope': 0.98322596, 'offset': 195.528},
    '0e0d': {'slope': 0.98216827, 'offset': 190.613},
}

# Ánh xạ anchor ID sang index
ANCHOR_ID_MAP = {
    '1912': 0,  # Anchor 0
    '4795': 1,  # Anchor 1
    '0e0d': 2,  # Anchor 2
    'c91a': 3,  # Anchor 3
}

ANCHORS = [
    (0, 0),
    (5470, 0),
    (5420, 5050),
    (770, 5050),  
]

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def apply_calibration(raw_distance, anchor_id):
    """
    Áp dụng hiệu chỉnh tuyến tính cho khoảng cách đo được
    
    Args:
        raw_distance: Khoảng cách thô từ UWB (mm)
        anchor_id: ID của anchor (string hex)
    
    Returns:
        Khoảng cách đã hiệu chỉnh (mm)
    """
    if anchor_id in CALIBRATION_MODELS:
        model = CALIBRATION_MODELS[anchor_id]
        calibrated = model['slope'] * raw_distance + model['offset']
        return calibrated
    else:
        return raw_distance

class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UWB Tracking - LSE with Linear Calibration")
        self.setGeometry(100, 100, 900, 700)
        self.data_queue = Queue()
        self.trail_history = []
        self.line_count = 0
        self.start_time = time.time()
        self.fps_time = time.time()
        self.last_update_time = time.time()

        # Pattern để parse anchor ID và distance
        self.serial_pattern = re.compile(r'0x([\da-f]{4}):\s*=(\d+)')

        # Thống kê calibration
        self.calibration_stats = {
            'total_measurements': 0,
            'calibrated_measurements': 0,
            'raw_distances': [],
            'calibrated_distances': []
        }

        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(15)

    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.1
            )
            print(f"✓ Serial port {SERIAL_PORT} opened successfully")
        except Exception as e:
            print("✗ Không thể mở cổng serial:", e)
            sys.exit(1)

    def init_ui(self):
        # Widget chính
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QVBoxLayout(main_widget)
        
        # Plot widget
        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)
        
        # Info panel
        info_panel = QtWidgets.QHBoxLayout()
        
        self.info_label = QtWidgets.QLabel("Calibration: 0/0 (0.0%)")
        self.info_label.setStyleSheet("font-size: 11px; padding: 5px; background-color: #2b2b2b;")
        info_panel.addWidget(self.info_label)
        
        self.distance_label = QtWidgets.QLabel("Distances: - - - -")
        self.distance_label.setStyleSheet("font-size: 11px; padding: 5px; background-color: #2b2b2b;")
        info_panel.addWidget(self.distance_label)
        
        layout.addLayout(info_panel)
        
        # Cấu hình plot
        self.plot_widget.setXRange(-500, 6500)
        self.plot_widget.setYRange(-500, 6000)
        self.plot_widget.setTitle("UWB Tracking - LSE with Linear Calibration (No Filter)")
        self.plot_widget.setLabel('left', 'Y (mm)')
        self.plot_widget.setLabel('bottom', 'X (mm)')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        # Vẽ anchors với labels
        for i, (x, y) in enumerate(ANCHORS):
            self.plot_widget.plot([x], [y], pen=None, symbol='s', 
                                symbolSize=15, symbolBrush='b')
            anchor_label = pg.TextItem(text=f"A{i}", color='b', anchor=(0.5, 1.5))
            anchor_label.setPos(x, y)
            self.plot_widget.addItem(anchor_label)

        # TAG point
        self.fpt_scatter = self.plot_widget.plot([], [], pen=None, symbol='o', 
                                                symbolSize=18, symbolBrush='r')
        
        # Trail
        self.trail_curve = self.plot_widget.plot([], [], pen=pg.mkPen('r', width=2.5))
        
        # TAG label
        self.fpt_text = pg.TextItem(text="TAG", color='r', anchor=(0.5, -1.5))
        self.fpt_text.setFont(pg.QtGui.QFont("Arial", 10, pg.QtGui.QFont.Bold))
        self.plot_widget.addItem(self.fpt_text)

        # Vùng tracking
        self.plot_widget.plot(
            [1200, 1200, 4800, 4800, 1200],
            [600, 2710, 2710, 600, 600],
            pen=pg.mkPen('g', width=2, style=QtCore.Qt.DashLine)
        )

    def start_serial_thread(self):
        threading.Thread(target=self.serial_reader, daemon=True).start()

    def serial_reader(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace')
                if line and '=' in line:
                    self.line_count += 1
                    self.data_queue.put(line)

                current_time = time.time()
                if current_time - self.start_time >= 1:
                    self.line_count = 0
                    self.start_time = current_time
            except Exception as e:
                print("Serial error:", e)

    def parse_and_calibrate_distances(self, raw_data):
        """
        Parse dữ liệu serial và áp dụng calibration
        
        Returns:
            tuple: (distances_list, raw_distances_dict, calibrated_distances_dict)
        """
        matches = self.serial_pattern.findall(raw_data)
        distances = [None] * 4
        raw_dict = {}
        calibrated_dict = {}
        
        for anchor_id, distance_str in matches:
            raw_distance = int(distance_str)
            self.calibration_stats['total_measurements'] += 1
            
            # Áp dụng calibration
            calibrated_distance = apply_calibration(raw_distance, anchor_id)
            
            # Lưu thống kê
            raw_dict[anchor_id] = raw_distance
            calibrated_dict[anchor_id] = calibrated_distance
            
            if anchor_id in CALIBRATION_MODELS:
                self.calibration_stats['calibrated_measurements'] += 1
            
            # Gán vào đúng vị trí
            if anchor_id in ANCHOR_ID_MAP:
                idx = ANCHOR_ID_MAP[anchor_id]
                distances[idx] = calibrated_distance
        
        if all(d is not None for d in distances):
            return distances, raw_dict, calibrated_dict
        else:
            return None, raw_dict, calibrated_dict

    def lse_trilateration(self, distances):
        """Thuật toán Least Squares cho 4 anchor"""
        if len(distances) < 4 or any(d is None for d in distances):
            return np.array([np.nan, np.nan])

        x1, y1 = ANCHORS[0]
        d1 = distances[0]
        A = []
        b = []

        for i in range(1, 4):
            xi, yi = ANCHORS[i]
            di = distances[i]

            A.append([2 * (xi - x1), 2 * (yi - y1)])
            b.append((d1**2 - di**2) - (x1**2 - xi**2) - (y1**2 - yi**2))

        A = np.array(A)
        b = np.array(b)

        try:
            pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            return pos
        except Exception as e:
            print("LSE error:", e)
            return np.array([np.nan, np.nan])

    def update_plot(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        fps = 1.0 / max(current_time - self.fps_time, 0.001)
        self.fps_time = current_time
        updated = False

        while not self.data_queue.empty():
            raw_data = self.data_queue.get()
            
            # Parse và calibrate
            distances, raw_dict, calibrated_dict = self.parse_and_calibrate_distances(raw_data)

            if distances is not None:
                # Tính vị trí bằng LSE
                pos = self.lse_trilateration(distances)

                if not np.isnan(pos).any():
                    self.trail_history.append(pos)
                    if len(self.trail_history) > 1500:
                        self.trail_history = self.trail_history[-1500:]

                    # In tọa độ và khoảng cách
                    print(f"X: {pos[0]:.2f}, Y: {pos[1]:.2f}")

                    # Cập nhật plot
                    self.fpt_scatter.setData([pos[0]], [pos[1]])
                    self.fpt_text.setPos(pos[0], pos[1])
                    
                    # Cập nhật distance label
                    dist_text = " | ".join([f"A{i}: {d:.0f}mm" for i, d in enumerate(distances)])
                    self.distance_label.setText(dist_text)
                    
                    updated = True

        # Cập nhật trail
        if updated and self.trail_history:
            trail = np.array(self.trail_history)
            self.trail_curve.setData(trail[:, 0], trail[:, 1])

        # Cập nhật info
        total = self.calibration_stats['total_measurements']
        calibrated = self.calibration_stats['calibrated_measurements']
        cal_percent = (calibrated / total * 100) if total > 0 else 0
        
        self.setWindowTitle(f"LSE + Calibration (FPS: {fps:.1f})")
        self.info_label.setText(
            f"Calibration: {calibrated}/{total} ({cal_percent:.1f}%) | "
            f"Trail: {len(self.trail_history)} pts"
        )

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    tracker = FPTTracker()
    tracker.show()
    sys.exit(app.exec_())
