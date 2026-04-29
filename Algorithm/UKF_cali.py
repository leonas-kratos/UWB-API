import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Cấu hình Anchor (4 anchor), thay đổi tọa độ nếu cần
ANCHORS = [
    (0, 0),       # Anchor 0
    (5470, 0),    # Anchor 1
    (5420, 5050), # Anchor 2
    (770, 5050),  # Anchor 3
]

# Cấu hình cổng Serial và tốc độ baud
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# ===== CALIBRATION PARAMETERS =====
# Mô hình hiệu chỉnh tuyến tính: y = slope * x + offset
# Dựa trên kết quả calibration từ các anchor
CALIBRATION_MODELS = {
    '4795': {'slope': 0.98451916, 'offset': 198.104},
    'c91a': {'slope': 0.98262259, 'offset': 202.704},
    '1912': {'slope': 0.98322596, 'offset': 195.528},
    '0e0d': {'slope': 0.98216827, 'offset': 190.613},
}

def apply_calibration(hex_addr, raw_distance):
    """Áp dụng calibration cho khoảng cách đo được"""
    if hex_addr in CALIBRATION_MODELS:
        model = CALIBRATION_MODELS[hex_addr]
        calibrated = model['slope'] * raw_distance + model['offset']
        return calibrated
    return raw_distance  # Nếu không có model, trả về giá trị gốc

class UKFPositionTracker:
    def __init__(self):
        # State vector: [x, y, vx, vy] - vị trí và vận tốc
        self.dim_x = 4
        # Measurement vector: [d1, d2, d3, d4] - khoảng cách đến 4 anchor
        self.dim_z = 4

        # Tham số UKF
        self.alpha = 0.1
        self.beta = 2.0
        self.kappa = 1.0

        # Tính lambda
        self.lambda_ = self.alpha**2 * (self.dim_x + self.kappa) - self.dim_x

        # Số lượng sigma points
        self.n_sigma = 2 * self.dim_x + 1

        # Tính weights
        self.Wm = np.zeros(self.n_sigma)
        self.Wc = np.zeros(self.n_sigma)

        self.Wm[0] = self.lambda_ / (self.dim_x + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.dim_x + self.lambda_) + (1 - self.alpha**2 + self.beta)

        for i in range(1, self.n_sigma):
            self.Wm[i] = 1 / (2 * (self.dim_x + self.lambda_))
            self.Wc[i] = 1 / (2 * (self.dim_x + self.lambda_))

        # State và covariance
        self.x = np.zeros(self.dim_x)  # [x, y, vx, vy]
        self.P = np.eye(self.dim_x) * 100  # Covariance matrix

        # Process noise covariance
        self.Q = np.eye(self.dim_x) * 10000

        # Measurement noise covariance
        self.R = np.eye(self.dim_z) * 50

        self.dt = 0.1
        self.initialized = False

    def generate_sigma_points(self, x, P):
        """Tạo sigma points"""
        n = len(x)
        sigma_points = np.zeros((self.n_sigma, n))

        # Tính square root của P
        try:
            L = np.linalg.cholesky((n + self.lambda_) * P)
        except np.linalg.LinAlgError:
            # Nếu P không positive definite, dùng eigenvalue decomposition
            eigval, eigvec = np.linalg.eigh(P)
            eigval = np.maximum(eigval, 0)  # Đảm bảo eigenvalues không âm
            L = eigvec @ np.diag(np.sqrt(eigval * (n + self.lambda_)))

        # Sigma point đầu tiên
        sigma_points[0] = x

        # Các sigma points còn lại
        for i in range(n):
            sigma_points[i + 1] = x + L[:, i]
            sigma_points[n + i + 1] = x - L[:, i]

        return sigma_points

    def fx(self, x, dt):
        """Hàm chuyển trạng thái (mô hình chuyển động)"""
        # Mô hình chuyển động với vận tốc không đổi
        x_new = np.zeros(4)
        x_new[0] = x[0] + x[2] * dt  # x = x + vx * dt
        x_new[1] = x[1] + x[3] * dt  # y = y + vy * dt
        x_new[2] = x[2]              # vx không đổi
        x_new[3] = x[3]              # vy không đổi
        return x_new

    def hx(self, x):
        """Hàm đo lường (tính khoảng cách từ vị trí ước lượng đến các anchor)"""
        distances = np.zeros(self.dim_z)
        for i, anchor in enumerate(ANCHORS):
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            distances[i] = np.sqrt(dx**2 + dy**2)
        return distances

    def initialize(self, initial_pos):
        """Khởi tạo bộ lọc với vị trí ban đầu"""
        self.x = np.array([initial_pos[0], initial_pos[1], 0, 0])  # [x, y, vx, vy]
        self.initialized = True

    def predict(self):
        """Bước dự đoán của UKF"""
        # Tạo sigma points từ state hiện tại
        sigma_points = self.generate_sigma_points(self.x, self.P)

        # Propagate sigma points qua hàm chuyển trạng thái
        sigma_points_f = np.zeros((self.n_sigma, self.dim_x))
        for i in range(self.n_sigma):
            sigma_points_f[i] = self.fx(sigma_points[i], self.dt)

        # Tính mean của predicted state
        self.x = np.sum(self.Wm[:, np.newaxis] * sigma_points_f, axis=0)

        # Tính covariance của predicted state
        self.P = np.zeros((self.dim_x, self.dim_x))
        for i in range(self.n_sigma):
            y = sigma_points_f[i] - self.x
            self.P += self.Wc[i] * np.outer(y, y)

        self.P += self.Q

        return sigma_points_f

    def update(self, z):
        """Bước cập nhật của UKF"""
        # Tạo sigma points từ predicted state
        sigma_points = self.generate_sigma_points(self.x, self.P)

        # Transform sigma points qua hàm đo lường
        sigma_points_h = np.zeros((self.n_sigma, self.dim_z))
        for i in range(self.n_sigma):
            sigma_points_h[i] = self.hx(sigma_points[i])

        # Tính mean của predicted measurement
        z_mean = np.sum(self.Wm[:, np.newaxis] * sigma_points_h, axis=0)

        # Tính covariance của measurement
        P_zz = np.zeros((self.dim_z, self.dim_z))
        for i in range(self.n_sigma):
            y = sigma_points_h[i] - z_mean
            P_zz += self.Wc[i] * np.outer(y, y)

        P_zz += self.R

        # Tính cross-covariance
        P_xz = np.zeros((self.dim_x, self.dim_z))
        for i in range(self.n_sigma):
            dx = sigma_points[i] - self.x
            dz = sigma_points_h[i] - z_mean
            P_xz += self.Wc[i] * np.outer(dx, dz)

        # Tính Kalman gain
        K = P_xz @ np.linalg.inv(P_zz)

        # Update state
        self.x = self.x + K @ (z - z_mean)

        # Update covariance
        self.P = self.P - K @ P_zz @ K.T

    def filter(self, z, dt=0.1):
        """Thực hiện một bước lọc hoàn chỉnh"""
        if not self.initialized:
            return np.array([np.nan, np.nan])

        self.dt = dt
        self.predict()
        self.update(np.array(z))

        return self.x[:2]  # Trả về vị trí [x, y]

class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tracking FPT - Real-Time Graph with UKF")
        self.setGeometry(100, 100, 800, 600)
        self.data_queue = Queue()
        self.position_history = []
        self.trail_history = []
        self.line_count = 0
        self.start_time = time.time()
        self.fps_time = time.time()
        self.last_update_time = time.time()

        # Mapping địa chỉ hex sang index anchor - BẮT BUỘC
        self.anchor_map = {
            '1912': 0,  # Anchor 0 (0, 0)
            '4795': 1,  # Anchor 1 (5470, 0)
            '0e0d': 2,  # Anchor 2 (5420, 5050)
            'c91a': 3   # Anchor 3 (770, 5050)
        }

        # Pattern để bắt cả địa chỉ hex và khoảng cách
        self.serial_pattern = re.compile(r'0x([\da-f]{4}):\s*=(\d+)')

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
        self.plot_widget.setXRange(-2000, 7000)
        self.plot_widget.setYRange(-2000, 7000)
        self.plot_widget.setTitle("Tracking Tag - Vị trí theo thời gian thực với UKF")

        # Vẽ các anchor dưới dạng các điểm xanh
        anchor_x, anchor_y = zip(*ANCHORS)
        self.plot_widget.plot(anchor_x, anchor_y, pen=None, symbol='o', symbolSize=20, symbolBrush='b')

        # Khởi tạo điểm FPT (điểm đỏ)
        self.fpt_scatter = self.plot_widget.plot([], [], pen=None, symbol='o', symbolSize=20, symbolBrush='r')
        # Vẽ quỹ đạo (trail) của FPT
        self.trail_curve = self.plot_widget.plot([], [], pen=pg.mkPen('r', width=1))

        # Hiển thị nhãn "TAG" gần điểm đối tượng
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

            # Parse với địa chỉ hex
            matches = self.serial_pattern.findall(raw_data)

            # Tạo mảng distances với 4 phần tử, khởi tạo None
            distances = [None] * 4

            # Sắp xếp đúng vị trí theo anchor_map và áp dụng calibration - BẮT BUỘC
            for hex_addr, dist in matches:
                if hex_addr in self.anchor_map:
                    anchor_idx = self.anchor_map[hex_addr]
                    raw_distance = int(dist)
                    # ÁP DỤNG CALIBRATION
                    calibrated_distance = apply_calibration(hex_addr, raw_distance)
                    distances[anchor_idx] = calibrated_distance

            # Kiểm tra đủ 4 anchor
            if None not in distances:
                # Ước lượng ban đầu bằng LSE
                initial_pos = self.lse_trilateration(distances)

                if not np.isnan(initial_pos).any():
                    # Khởi tạo UKF nếu chưa được khởi tạo
                    if not self.ukf_initialized:
                        self.ukf_tracker.initialize(initial_pos)
                        self.ukf_initialized = True
                        filtered_pos = initial_pos
                    else:
                        # Cập nhật UKF với khoảng cách đã calibrated
                        filtered_pos = self.ukf_tracker.filter(distances, dt)

                    if not np.isnan(filtered_pos).any():
                        self.position_history.append(filtered_pos)
                        if len(self.position_history) > 100:
                            self.position_history = self.position_history[-100:]

                        smoothed_pos = filtered_pos

                        self.trail_history.append(smoothed_pos)
                        if len(self.trail_history) > 1000:
                            self.trail_history = self.trail_history[-1000:]

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
