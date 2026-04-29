
Điểm nổi bật trong thư mục
Các tài liệu chứa mã Python để theo dõi vị trí UWB theo thời gian thực bằng cách sử dụng các bộ lọc Kalman, EKF, UKF và Particle Filter.

import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

# Cấu hình Anchor (4 anchor), thay đổi tọa độ nếu cần
ANCHORS = [
    (0, 0),
    (5470, 0),
    (5420, 5050),
    (770, 5050),
]


# Cấu hình cổng Serial và tốc độ baud
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

class UKFPositionTracker:
    def __init__(self):
        # State vector: [x, y, vx, vy] - vị trí và vận tốc
        self.dim_x = 4
        # Measurement vector: [d1, d2, d3, d4] - khoảng cách đến 4 anchor
        self.dim_z = 4

        # Khởi tạo sigma points cho UKF
        points = MerweScaledSigmaPoints(n=self.dim_x,
                                      alpha=0.1,
                                      beta=2.0,
                                      kappa=1.0)

        # Khởi tạo UKF
        self.ukf = UnscentedKalmanFilter(dim_x=self.dim_x,
                                       dim_z=self.dim_z,
                                       dt=0.1,
                                       fx=self.fx,
                                       hx=self.hx,
                                       points=points)

        # Ma trận hiệp phương trình ban đầu
        self.ukf.P = np.eye(self.dim_x) * 100
        # Ma trận nhiễu quá trình
        self.ukf.Q = np.eye(self.dim_x) * 0.01
        # Ma trận nhiễu đo lường
        self.ukf.R = np.eye(self.dim_z) * 0.01

        self.initialized = False

    def fx(self, x, dt):
        """Hàm chuyển trạng thái (mô hình chuyển động)"""
        # Mô hình chuyển động với vận tốc không đổi
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return F @ x

    def hx(self, x):
        """Hàm đo lường (tính khoảng cách từ vị trí ước lượng đến các anchor)"""
        distances = []
        for anchor in ANCHORS:
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            distance = np.sqrt(dx**2 + dy**2)
            distances.append(distance)
        return np.array(distances)

    def initialize(self, initial_pos):
        """Khởi tạo bộ lọc với vị trí ban đầu"""
        self.ukf.x = np.array([initial_pos[0], initial_pos[1], 0, 0])  # [x, y, vx, vy]
        self.initialized = True

    def update(self, distances, dt=0.1):
        """Cập nhật bộ lọc với các khoảng cách đo được"""
        if not self.initialized:
            return np.array([np.nan, np.nan])

        self.ukf.dt = dt
        self.ukf.predict()
        self.ukf.update(np.array(distances))

        return self.ukf.x[:2]  # Trả về vị trí [x, y]

class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tracking FPT - Real-Time Graph with UKF")
        self.setGeometry(100, 100, 800, 600)
        self.data_queue = Queue()
        self.position_history = []
        self.trail_history = []  # Lịch sử quỹ đạo di chuyển của FPT
        self.line_count = 0
        self.start_time = time.time()
        self.fps_time = time.time()
        self.last_update_time = time.time()

        # Biểu thức chính quy tìm dữ liệu khoảng cách
        self.serial_pattern = re.compile(r'0x[\da-f]{4}:\s*=(\d+)')

        # Khởi tạo bộ lọc UKF
        self.ukf_tracker = UKFPositionTracker()
        self.ukf_initialized = False

        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        # Timer cập nhật đồ thị
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
        except Exception as e:
            print("Không thể mở cổng serial:", e)
            sys.exit(1)

    def init_ui(self):
        # Khởi tạo widget đồ thị của PyQtGraph
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        # Cấu hình giới hạn trục
        self.plot_widget.setXRange(-2000, 10000)
        self.plot_widget.setYRange(-2000, 10000)
        self.plot_widget.setTitle("Tracking Tag - Vị trí theo thời gian thực với UKF")

        # Vẽ các anchor dưới dạng các điểm xanh
        anchor_x, anchor_y = zip(*ANCHORS)
        self.plot_widget.plot(anchor_x, anchor_y, pen=None, symbol='o', symbolSize=20, symbolBrush='b')

        # Khởi tạo điểm FPT (điểm đỏ)
        self.fpt_scatter = self.plot_widget.plot([], [], pen=None, symbol='o', symbolSize=20, symbolBrush='r')
        # Vẽ quỹ đạo (trail) của FPT
        self.trail_curve = self.plot_widget.plot([], [], pen=pg.mkPen('r', width=1))

        # Hiển thị nhãn "FPT" gần điểm đối tượng
        self.fpt_text = pg.TextItem(text="TAG", color='w', anchor=(0.5, -1.0))
        self.plot_widget.addItem(self.fpt_text)
        self.plot_widget.plot(
            [1200, 1200, 4800, 4800, 1200],
            [600, 2710, 2710, 600, 600],
            pen=pg.mkPen('g', width=2)  # màu xanh lá
        )

    def start_serial_thread(self):
        threading.Thread(target=self.serial_reader, daemon=True).start()

    def serial_reader(self):
        """Đọc dữ liệu từ cổng serial và đếm số dòng mỗi giây"""
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace')
                if line:
                    self.line_count += 1
                    if '=' in line:
                        self.data_queue.put(line)
                    # Kiểm tra nếu đã qua 1 giây thì in số dòng đã đọc
                    current_time = time.time()
                    if current_time - self.start_time >= 1:
                        # print(f"Lines per second: {self.line_count}")
                        self.line_count = 0
                        self.start_time = current_time
            except Exception as e:
                print("Serial error:", e)

    def lse_trilateration(self, distances):
        """Thuật toán Least Squares cho 4 anchor nhằm ước lượng vị trí đối tượng FPT"""
        if len(distances) < 4:
            return np.array([np.nan, np.nan])
        x1, y1 = ANCHORS[0]
        d1 = distances[0]
        A = []
        b = []
        for i in range(1, 4):
            xi, yi = ANCHORS[i]
            di = distances[i]
            A.append([xi - x1, yi - y1])
            b.append(0.5 * (xi**2 + yi**2 - di**2 - (x1**2 + y1**2 - d1**2)))
        try:
            A = np.array(A)
            b = np.array(b)
            return np.linalg.lstsq(A, b, rcond=None)[0]
        except Exception as e:
            print("LSE error:", e)
            return np.array([np.nan, np.nan])

    def update_plot(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        fps = 1.0 / (current_time - self.fps_time)
        self.fps_time = current_time
        updated = False

        # Xử lý dữ liệu mới từ serial
        while not self.data_queue.empty():
            raw_data = self.data_queue.get()
            distances = list(map(int, self.serial_pattern.findall(raw_data)))
            if len(distances) >= 4:
                # Ước lượng ban đầu bằng LSE
                initial_pos = self.lse_trilateration(distances[:4])

                if not np.isnan(initial_pos).any():
                    # Khởi tạo UKF nếu chưa được khởi tạo
                    if not self.ukf_initialized:
                        self.ukf_tracker.initialize(initial_pos)
                        self.ukf_initialized = True
                        filtered_pos = initial_pos
                    else:
                        # Cập nhật UKF với khoảng cách mới
                        filtered_pos = self.ukf_tracker.update(distances[:4], dt)

                    if not np.isnan(filtered_pos).any():
                        self.position_history.append(filtered_pos)
                        if len(self.position_history) > 100:
                            self.position_history = self.position_history[-100:]

                        # Làm mượt kết quả bằng trung bình động
                        #smoothed_pos = np.mean(self.position_history, axis=0)
                        smoothed_pos = filtered_pos

                        self.trail_history.append(smoothed_pos)
                        if len(self.trail_history) > 1000:
                            self.trail_history = self.trail_history[-6000:]

                        # IN TỌA ĐỘ RA TERMINAL
                        print(f"X: {smoothed_pos[0]:.2f}, Y: {smoothed_pos[1]:.2f}")

                        # Cập nhật vị trí của FPT
                        self.fpt_scatter.setData([filtered_pos[0]], [filtered_pos[1]])
                        self.fpt_text.setPos(filtered_pos[0], filtered_pos[1])
                        updated = True

        # Cập nhật quỹ đạo nếu có dữ liệu mới
        if updated and self.trail_history:
            trail = np.array(self.trail_history)
            self.trail_curve.setData(trail[:, 0], trail[:, 1])

        self.setWindowTitle(f"Real-Time UWB Tracking with UKF (FPS: {fps:.2f})")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    tracker = FPTTracker()
    tracker.show()
    sys.exit(app.exec_())
