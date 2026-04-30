# UWB-API

> Evaluation of State Estimation Filters for Indoor Robot Positioning

## Abstract

This study evaluates three state estimation algorithms — **EKF**, **UKF**, and **PF** — for UWB-based indoor robot positioning where GPS is unavailable.

| Filter | Strength | Weakness |
|--------|----------|----------|
| EKF | Fast, low computational cost | Sensitive to noise & nonlinearity |
| UKF | Balanced accuracy & efficiency | Moderate complexity |
| PF  | Handles non-Gaussian noise well | High computational cost |

**Keywords:** UWB, indoor positioning, localization, Kalman filter, particle filter

---

## 1. System Overview

### Hardware
- **UWB modules:** DWM1001 (UWB transceiver + ARM Cortex-M4 + BLE)
- **Layout:** 4 fixed **Anchors** + 1 mobile **Tag** on a 4-wheel robot
- **Processing unit:** Raspberry Pi 4 running EKF/UKF/PF in Python
- **Ranging method:** Two-Way Ranging (TWR), update rate up to **160 Hz**, `dt = 0.1 s`

### Setup
```
Anchor positions (mm):
  A1 = (0, 0)        A2 = (5470, 0)
  A4 = (770, 5050)   A3 = (5420, 5050)

Test area: ~5.5 m × 5 m (LOS conditions)
Robot speed: 0.5 m/s along rectangular trajectory A→B→C→D→A
```

---

## 2. Mathematical Model

### State Vector (2D Constant-Velocity)
$$\mathbf{x}_k = [x_k,\ y_k,\ v_{x,k},\ v_{y,k}]^\top$$

### State Transition
$$\mathbf{x}_{k+1} = \mathbf{F}\,\mathbf{x}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q})$$

### Measurement Model (nonlinear — distance to each anchor)
$$z_{i,k} = \sqrt{(x_k - x_{a_i})^2 + (y_k - y_{a_i})^2} + v_{i,k}$$

---

## 3. Algorithms

### Initialization — Least-Squares Estimation (LSE)
Used to compute the initial position from the first 4 TWR measurements:
$$\mathbf{x}_\text{LS} = (\mathbf{A}^\top\mathbf{A})^{-1}\mathbf{A}^\top\mathbf{b}$$

### EKF
Linearizes the measurement function via Jacobian:
$$\mathbf{H}_k[i,:] = \left[\frac{\hat{x}-x_{a_i}}{\hat{d}_i},\ \frac{\hat{y}-y_{a_i}}{\hat{d}_i},\ 0,\ 0\right]$$
- **Predict:** $\mathbf{x}^- = \mathbf{F}\mathbf{x}$, $\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$
- **Update:** standard Kalman gain + innovation correction

### UKF
Uses **2n+1 = 9 sigma points** to propagate the distribution through the nonlinear measurement function — no Jacobian required.  
Parameters: $\alpha = 0.1$, $\kappa = 1.0$, $\beta = 2$.

### Particle Filter (PF)
Monte Carlo approximation of the posterior:
$$p(\mathbf{x}_k|\mathbf{z}_{1:k}) \approx \sum_{i=1}^{N} w_k^{(i)}\,\delta(\mathbf{x}_k - \mathbf{x}_k^{(i)})$$
Weights updated via Gaussian likelihood; resampling triggered when $N_\text{eff}$ drops below threshold.

---

## 4. Bias Calibration

Linear calibration model per anchor:
$$d' = m \cdot d + c$$

Calibration data: distances 1.2–5.4 m (step 0.6 m), 5000 samples each.

**Result:**

| Metric | Raw | Calibrated |
|--------|-----|------------|
| RMSE range | 100–190 mm | 15–65 mm |
| LSE RMSE | 111.6 mm | 72.7 mm |
| Error reduction | — | **~35%** |

---

## 5. Results

### Scenario 1 — PF: Particle Count vs. Performance

| Particles | Sampling Rate (Hz) | RMSE Raw (mm) | RMSE Calibrated (mm) |
|:---------:|:------------------:|:-------------:|:--------------------:|
| **10**    | **128**            | **83.1**      | **56.9**             |
| 30        | 119                | 106.6         | 64.5                 |
| 50        | 108                | 100.8         | 73.8                 |
| 70        | 94                 | 107.5         | 76.3                 |
| 90        | 87                 | 109.8         | 63.9                 |

> **Finding:** Fewer particles = higher update rate = better accuracy in this setup. **10 particles** gives optimal trade-off.

---

### Scenario 2 — EKF/UKF Sensitivity to Q and R

#### Varying Process Noise Q (fixed R)

| | Best Q | Min RMSE | Sensitivity |
|--|--------|----------|-------------|
| **EKF** (raw) | $10^3$ | 51.3 mm | High — large swings |
| **UKF** (raw) | $10^{-3}$ | 52.3 mm | Low — narrow band 52–80 mm |
| **EKF** (calib.) | $10$ | 80.5 mm | High |
| **UKF** (calib.) | $10^{-3}$ | **55.5 mm** | Low — stable 55–76 mm |

#### Varying Measurement Noise R (fixed Q)

| | Best R | Min RMSE | Notes |
|--|--------|----------|-------|
| **EKF** (raw) | $10^{-4}$ | 63.9 mm | Moderate variation |
| **UKF** (raw) | $10$ | 53.1 mm | More stable than EKF |
| **EKF** (calib.) | $10^2$ | **44.8 mm** | Best single result |
| **UKF** (calib.) | $10^{-5}$ | 59.9 mm | Consistent, no sharp drops |

> **Finding:** After calibration, a well-tuned EKF achieves the **lowest RMSE (44.8 mm)**. UKF is more robust across a wider parameter range.

---

## 6. Conclusion

| Method | RMSE (calibrated) | Computation | Robustness |
|--------|:-----------------:|:-----------:|:----------:|
| LSE baseline | 72.7 mm | Very low | — |
| PF (10 particles) | 56.9 mm | Medium | High |
| UKF (optimal Q) | 55.5 mm | Low-Medium | **High** |
| EKF (optimal R) | **44.8 mm** | **Lowest** | Medium |

- **EKF** — best peak accuracy, requires careful tuning of Q and R  
- **UKF** — best overall robustness, recommended for practical deployment  
- **PF** — flexible but computationally heavier; fewer particles can outperform more  
- **Bias calibration** is critical: reduces RMSE by ~35% across all methods

### Future Work
- Learning-based state estimation (e.g., KalmanNet)
- SLAM integration for dynamic environments
- IMU/UWB fusion for improved robustness

---

## References

1. Gamarra et al., *IPIN 2023* — Hybrid BLE AoA + GNSS seamless positioning
2. Huang et al., *ISPACS 2024* — NLOS impact on UWB 3D positioning
3. Joseph & Sasi, *ICCSDET 2018* — Indoor positioning via WiFi fingerprint
4. Mallick et al., *ICCAIS 2021* — Comparison of KF, EKF, UKF, CKF, PF in GMTI
5. Wei et al., *ICMSP 2023* — DS-TWR outdoor localization optimization
6. Moosavi & Ghassabian, *IntechOpen 2018* — Calibration curve linearity
7. Du, *ICAICE 2024* — UWB/BLE indoor positioning for hospitals
8. Feng et al., *IEEE IoT Journal 2020* — Kalman filter IMU+UWB integration
9. Revach et al., *IEEE TSP 2022* — KalmanNet neural-aided filtering
10. Duong et al., *TELKOMNIKA 2025* — UWB positioning with low-cost MCUs
