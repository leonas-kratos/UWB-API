import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

# ======== Matplotlib configuration ========
plt.rcParams.update({
    "font.size": 18,             # cỡ chữ lớn hơn
    "axes.labelsize": 20,
    "axes.titlesize": 22,
    "xtick.labelsize": 20,
    "ytick.labelsize": 20,
    "legend.fontsize": 30,
    "lines.linewidth": 3,        # đường dày hơn
    "lines.markersize": 10,      # marker to hơn
})

# ======== Theoretical path configuration ========
theoretical_path = np.array([
    [1200, 2700],
    [1200, 600],
    [4800, 600],
    [4800, 2700],
    [1200, 2700]
])

# ======== File reading function ========
def read_xy_from_file(filepath):
    """Read X, Y data from file"""
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

# ======== Error calculation function ========
def calculate_errors(data_points, theoretical_path):
    """Calculate error from points to theoretical path"""
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

# ======== CDF plotting function ========
def plot_cdf(errors, label, color):
    """Plot CDF graph"""
    sorted_errors = np.sort(errors)
    cdf = np.arange(len(errors)) / len(errors)
    plt.plot(sorted_errors, cdf, label=label, color=color)

# ======== Main ========
# Read data from LSE.txt
print("Reading LSE.txt...")
lse_data = read_xy_from_file("LSE.txt")
lse_errors = calculate_errors(lse_data, theoretical_path)
lse_rmse = np.sqrt(np.mean(lse_errors**2))
print(f"LSE: {len(lse_data)} points, RMSE = {lse_rmse:.3f} mm")

# Read data from LSE_cali.txt
print("Reading LSE_cali.txt...")
lse_cali_data = read_xy_from_file("LSE_cali.txt")
lse_cali_errors = calculate_errors(lse_cali_data, theoretical_path)
lse_cali_rmse = np.sqrt(np.mean(lse_cali_errors**2))
print(f"LSE_cali: {len(lse_cali_data)} points, RMSE = {lse_cali_rmse:.3f} mm")

# Vẽ đồ thị CDF
plt.figure(figsize=(12, 8))
plot_cdf(lse_errors, f"LSE raw (RMSE={lse_rmse:.1f} mm)", 'red')
plot_cdf(lse_cali_errors, f"LSE_cali (RMSE={lse_cali_rmse:.1f} mm)", 'blue')
plt.xlabel("Error (mm)")
plt.ylabel("CDF")
plt.grid(True)
plt.legend(loc='lower right')
plt.tight_layout()
plt.savefig('lse_cdf_comparison.png', dpi=300, bbox_inches='tight')
print("\nPlot saved: lse_cdf_comparison.png")
plt.show()
