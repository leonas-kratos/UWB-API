import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import os
import argparse
from pathlib import Path

# ======== Cấu hình đường lý thuyết ========
theoretical_path = np.array([
    [1200, 2700],
    [1200, 600],
    [4800, 600],
    [4800, 2700],
    [1200, 2700]
])

# ======== Hàm đọc file ========
def read_xy_from_file(filepath):
    """Đọc dữ liệu X, Y từ file"""
    data_points = []
    with open(filepath, "r", encoding='utf-8') as f:
        for line in f:
            if "X:" in line and "Y:" in line:
                try:
                    parts = line.strip().split(',')
                    x = float(parts[0].split(':')[1])
                    y = float(parts[1].split(':')[1])
                    data_points.append((x, y))
                except:
                    pass
    return np.array(data_points)

# ======== Hàm tính sai số ========
def calculate_errors(data_points, theoretical_path):
    """Tính sai số từ điểm đến đường lý thuyết"""
    errors = []
    for point in data_points:
        dists = cdist([point], theoretical_path)
        min_dist_to_points = np.min(dists)
        min_dist_to_segments = np.inf
        for i in range(len(theoretical_path) - 1):
            p1, p2 = theoretical_path[i], theoretical_path[i + 1]
            seg_len = np.linalg.norm(p2 - p1)
            if seg_len == 0:
                dist = np.linalg.norm(point - p1)
            else:
                t = max(0, min(1, np.dot(point - p1, p2 - p1) / (seg_len ** 2)))
                proj = p1 + t * (p2 - p1)
                dist = np.linalg.norm(point - proj)
            if dist < min_dist_to_segments:
                min_dist_to_segments = dist
        errors.append(min(min_dist_to_points, min_dist_to_segments))
    return np.array(errors)

# ======== Hàm vẽ CDF ========
def plot_cdf(errors, label, color=None, linestyle='-'):
    """Vẽ đồ thị CDF"""
    sorted_errors = np.sort(errors)
    cdf = np.arange(len(errors)) / len(errors)
    if color:
        plt.plot(sorted_errors, cdf, label=label, linewidth=2, color=color, linestyle=linestyle)
    else:
        plt.plot(sorted_errors, cdf, label=label, linewidth=2, linestyle=linestyle)

# ======== Hàm đọc dữ liệu từ thư mục ========
def load_pf_files_from_directory(directory):
    """
    Đọc tất cả file .txt trong thư mục
    Trả về: [(filename, filepath), ...]
    """
    files_list = []
    
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):
            filepath = os.path.join(directory, filename)
            files_list.append((filename, filepath))
    
    # Sắp xếp theo tên file
    files_list.sort(key=lambda x: x[0])
    
    return files_list

# ======== Hàm main ========
def main():
    parser = argparse.ArgumentParser(description='So sánh PF trước và sau calibration')
    parser.add_argument('--before', type=str, required=True, 
                        help='Thư mục chứa dữ liệu PF trước calibration')
    parser.add_argument('--after', type=str, required=True,
                        help='Thư mục chứa dữ liệu PF sau calibration')
    parser.add_argument('--lse', type=str, required=True,
                        help='File LSE calibration')
    parser.add_argument('--output', type=str, default='pf_cdf_comparison.png',
                        help='Tên file output cho đồ thị CDF')

    args = parser.parse_args()

    # Kiểm tra thư mục
    if not os.path.isdir(args.before):
        print(f"[Error] Thư mục 'before' không tồn tại: {args.before}")
        return
    
    if not os.path.isdir(args.after):
        print(f"[Error] Thư mục 'after' không tồn tại: {args.after}")
        return
    
    if not os.path.isfile(args.lse):
        print(f"[Error] File LSE không tồn tại: {args.lse}")
        return

    # Đọc dữ liệu
    print(f"Đang đọc dữ liệu từ thư mục 'before': {args.before}")
    before_files = load_pf_files_from_directory(args.before)
    
    print(f"Đang đọc dữ liệu từ thư mục 'after': {args.after}")
    after_files = load_pf_files_from_directory(args.after)
    
    print(f"Đang đọc dữ liệu LSE: {args.lse}")
    lse_data = read_xy_from_file(args.lse)

    print(f"\nTìm thấy:")
    print(f"  - Before: {len(before_files)} file")
    print(f"  - After: {len(after_files)} file")
    print(f"  - LSE: {len(lse_data)} điểm")

    # Tính errors và RMSE cho LSE
    lse_errors = calculate_errors(lse_data, theoretical_path)
    lse_rmse = np.sqrt(np.mean(lse_errors**2))

    # ======== Vẽ CDF ========
    plt.figure(figsize=(14, 8))
    
    # Vẽ LSE trước
    plot_cdf(lse_errors, f'LSE (RMSE={lse_rmse:.1f} mm)', color='purple', linestyle='--')

    # Vẽ Before calibration
    print("\n" + "="*60)
    print("BEFORE CALIBRATION")
    print("="*60)
    
    before_rmse_list = []
    for filename, filepath in before_files:
        data_points = read_xy_from_file(filepath)
        
        if len(data_points) == 0:
            print(f"[Warning] Không đọc được dữ liệu từ {filename}")
            continue
        
        errors = calculate_errors(data_points, theoretical_path)
        rmse = np.sqrt(np.mean(errors**2))
        before_rmse_list.append(rmse)
        
        label = f'Before - {filename.replace(".txt", "")} (RMSE={rmse:.1f} mm)'
        plot_cdf(errors, label, color='red')
        
        print(f"{filename:40s}: RMSE = {rmse:8.3f} mm")

    # Vẽ After calibration
    print("\n" + "="*60)
    print("AFTER CALIBRATION")
    print("="*60)
    
    after_rmse_list = []
    for filename, filepath in after_files:
        data_points = read_xy_from_file(filepath)
        
        if len(data_points) == 0:
            print(f"[Warning] Không đọc được dữ liệu từ {filename}")
            continue
        
        errors = calculate_errors(data_points, theoretical_path)
        rmse = np.sqrt(np.mean(errors**2))
        after_rmse_list.append(rmse)
        
        label = f'After - {filename.replace(".txt", "")} (RMSE={rmse:.1f} mm)'
        plot_cdf(errors, label, color='green')
        
        print(f"{filename:40s}: RMSE = {rmse:8.3f} mm")

    # ======== Tính RMSE trung bình ========
    print("\n" + "="*60)
    print("SO SÁNH RMSE TRUNG BÌNH")
    print("="*60)
    print(f"LSE RMSE:              {lse_rmse:.3f} mm")
    
    if before_rmse_list:
        avg_before_rmse = np.mean(before_rmse_list)
        print(f"Before Avg RMSE:       {avg_before_rmse:.3f} mm")
        print(f"  Improvement vs LSE:  {lse_rmse - avg_before_rmse:+.3f} mm")
    
    if after_rmse_list:
        avg_after_rmse = np.mean(after_rmse_list)
        print(f"After Avg RMSE:        {avg_after_rmse:.3f} mm")
        print(f"  Improvement vs LSE:  {lse_rmse - avg_after_rmse:+.3f} mm")
    
    if before_rmse_list and after_rmse_list:
        improvement = avg_before_rmse - avg_after_rmse
        print(f"\nCalibration Effect:    {improvement:+.3f} mm ({improvement/avg_before_rmse*100:+.1f}%)")

    # Hoàn thiện đồ thị CDF
    plt.xlabel("Sai số (mm)", fontsize=18)
    plt.ylabel("Xác suất tích lũy (CDF)", fontsize=18)
    plt.title("So sánh CDF: Before vs After Calibration vs LSE", fontsize=20, weight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10, loc='lower right', ncol=1, framealpha=0.9)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.tight_layout()
    
    # Lưu đồ thị
    plt.savefig(args.output, dpi=300, bbox_inches='tight')
    print(f"\nĐồ thị CDF đã được lưu tại: {args.output}")
    
    plt.show()

if __name__ == "__main__":
    main()
