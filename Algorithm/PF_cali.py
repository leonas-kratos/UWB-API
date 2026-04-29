
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
    (0, 0),
    (5470, 0),
    (5420, 5050),
    (770, 5050),
]

CALIBRATION_MODELS = {
    '4795': {'slope': 0.98451916, 'offset': 198.104},
    'c91a': {'slope': 0.98262259, 'offset': 202.704},
    '1912': {'slope': 0.98322596, 'offset': 195.528},
    '0e0d': {'slope': 0.98216827, 'offset': 190.613},
}

# Cấu hình cổng Serial và tốc độ baud
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

class ParticleFilterTracker:
    def __init__(self, num_particles=90):
        self.num_particles = num_particles

        # State vector: [x, y, vx, vy] - vị trí và vận tốc cho mỗi particle
        self.particles = np.zeros((num_particles, 4))
        self.weights = np.ones(num_particles) / num_particles

        # Noise parameters
        self.process_noise_std = np.array([10, 10, 5, 5])  # [x, y, vx, vy]
        self.measurement_noise_std = 50

        self.initialized = False
        self.dt = 0.1

    def initialize(self, initial_pos):
        """Khởi tạo các particles xung quanh vị trí ban đầu"""
        self.particles[:, 0] = np.random.normal(initial_pos[0], 100, self.num_particles)  # x
        self.particles[:, 1] = np.random.normal(initial_pos[1], 100, self.num_particles)  # y
        self.particles[:, 2] = np.random.normal(0, 10, self.num_particles)  # vx
        self.particles[:, 3] = np.random.normal(0, 10, self.num_particles)  # vy

        self.weights = np.ones(self.num_particles) / self.num_particles
        self.initialized = True

    def predict(self, dt=0.1):
        """Bước dự đoán - cập nhật vị trí các particles"""
        if not self.initialized:
            return

        self.dt = dt

        # Mô hình chuyển động: position = position + velocity * dt
        self.particles[:, 0] += self.particles[:, 2] * dt  # x = x + vx*dt
        self.particles[:, 1] += self.particles[:, 3] * dt  # y = y + vy*dt

        # Thêm noise quá trình
        noise = np.random.normal(0, self.process_noise_std, self.particles.shape)
        self.particles += noise

    def update_weights(self, distances):
        """Cập nhật trọng số dựa trên đo lường khoảng cách"""
        if not self.initialized or len(distances) < 4:
            return

        for i, particle in enumerate(self.particles):
            likelihood = 1.0

            for j, anchor in enumerate(ANCHORS[:4]):
                # Tính khoảng cách từ particle đến anchor
                dx = particle[0] - anchor[0]
                dy = particle[1] - anchor[1]
                predicted_distance = np.sqrt(dx**2 + dy**2)

                # Tính likelihood dựa trên sai số đo lường
                error = distances[j] - predicted_distance
                likelihood *= np.exp(-0.5 * (error / self.measurement_noise_std)**2)

            self.weights[i] = likelihood

        # Normalize weights
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles

    def resample(self):
        """Resample particles dựa trên trọng số"""
        if not self.initialized:
            return

        # Kiểm tra effective sample size
        eff_sample_size = 1.0 / np.sum(self.weights**2)
        threshold = self.num_particles / 3.0

        if eff_sample_size < threshold:
            # Systematic resampling
            indices = np.random.choice(
                self.num_particles,
                self.num_particles,
                p=self.weights
            )
            self.particles = self.particles[indices]
            self.weights = np.ones(self.num_particles) / self.num_particles

    def update(self, distances, dt=0.1):
        """Cập nhật bộ lọc với các khoảng cách đo được"""
        if not self.initialized:
            return np.array([np.nan, np.nan])

        # Predict
        self.predict(dt)

        # Update weights
        self.update_weights(distances)

        # Resample
        self.resample()

        # Trả về ước lượng vị trí (weighted mean)
        position = np.average(self.particles[:, :2], weights=self.weights, axis=0)
        return position

class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tracking FPT - Real-Time Graph with Particle Filter")
        self.setGeometry(100, 100, 800, 600)
        self.data_queue = Queue()
        self.position_history = []
        self.trail_history = []  # Lịch sử quỹ đạo di chuyển của FPT
        self.line_count = 0
        self.start_time = time.time()
        self.fps_time = time.time()
        self.last_update_time = time.time()

        # Biểu thức chính quy tìm dữ liệu khoảng cách
        self.serial_pattern = re.compile(r'0x([\da-f]{4}):\s*=(\d+)')

        # Khởi tạo bộ lọc Particle Filter
        self.pf_tracker = ParticleFilterTracker()
        self.pf_initialized = False

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
        self.plot_widget.setTitle("Tracking Tag - Vị trí theo thời gian thực với Particle Filter")

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

    def calibrate_distances(self, raw_data):
        """Hiệu chỉnh khoảng cách dựa trên anchor ID"""
        matches = self.serial_pattern.findall(raw_data)
        calibrated_distances = []

        for anchor_id, raw_value in matches:
            raw_distance = int(raw_value)

            # Áp dụng calibration nếu có model cho anchor này
            if anchor_id in CALIBRATION_MODELS:
                model = CALIBRATION_MODELS[anchor_id]
                calibrated = model['slope'] * raw_distance + model['offset']
                calibrated_distances.append(calibrated)
            else:
                calibrated_distances.append(raw_distance)

        return calibrated_distances

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
            distances = self.calibrate_distances(raw_data)
            if len(distances) >= 4:
                # Ước lượng ban đầu bằng LSE
                initial_pos = self.lse_trilateration(distances[:4])

                if not np.isnan(initial_pos).any():
                    # Khởi tạo Particle Filter nếu chưa được khởi tạo
                    if not self.pf_initialized:
                        self.pf_tracker.initialize(initial_pos)
                        self.pf_initialized = True
                        filtered_pos = initial_pos
                    else:
                        # Cập nhật Particle Filter với khoảng cách mới
                        filtered_pos = self.pf_tracker.update(distances[:4], dt)

                    if not np.isnan(filtered_pos).any():
                        self.position_history.append(filtered_pos)
                        if len(self.position_history) > 100:
                            self.position_history = self.position_history[-100:]

                        # Làm mượt kết quả bằng trung bình động
                        #smoothed_pos = np.mean(self.position_history, axis=0)
                        smoothed_pos = filtered_pos

                        self.trail_history.append(smoothed_pos)
                        if len(self.trail_history) > 1000:
                            self.trail_history = self.trail_history[-3000:]

                        # IN TỌA ĐỘ RA TERMINAL
                        print(f"X: {smoothed_pos[0]:.2f}, Y: {smoothed_pos[1]:.2f}")

                        self.fpt_scatter.setData([filtered_pos[0]], [filtered_pos[1]])
                        self.fpt_text.setPos(filtered_pos[0], filtered_pos[1])
                        updated = True

        # Cập nhật quỹ đạo nếu có dữ liệu mới
        if updated and self.trail_history:
            trail = np.array(self.trail_history)
            self.trail_curve.setData(trail[:, 0], trail[:, 1])

        self.setWindowTitle(f"Real-Time UWB Tracking with Particle Filter (FPS: {fps:.2f})")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    tracker = FPTTracker()
    tracker.show()
    sys.exit(app.exec_())
