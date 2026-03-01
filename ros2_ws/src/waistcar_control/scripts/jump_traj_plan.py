import numpy as np
import csv

# =========================
# Parameters (tune here)
# =========================

H = 0.25        # 障碍物高度 (m)
D = 0.60        # 障碍物距离 (m)
z0 = 0.30       # 起跳时质心高度 (m)
g  = 9.81

safety_margin = 0.05  # 过障安全裕度 (m)

# Timing
T_prepare = 0.25   # 压缩 / 储能
T_takeoff = 0.18   # 起跳
T_air = 0.40       # 空中

# Wheel capability (empirical)
vx = 1.6           # 水平速度 m/s（可调）
wheel_vel_max = 18.0  # rad/s

# Waist angle limits
waist_front = 0.6     # rad（起跳前摆）
waist_recover = -0.3  # rad（空中回收）

dt = 0.002  # 500 Hz


# =========================
# Utilities
# =========================

def quintic(t, T, p0, v0, a0, p1, v1, a1):
    """
    5 次多项式
    """
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([p0, v0, a0, p1, v1, a1])
    coef = np.linalg.solve(A, b)

    tt = np.array([1, t, t**2, t**3, t**4, t**5])
    dtt = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
    p = coef @ tt
    v = coef @ dtt
    return p, v


# =========================
# Step 1: compute minimum vz (obstacle clearance)
# =========================

tD = D / vx
vz_min = (H + safety_margin - z0 + 0.5 * g * tD**2) / tD

print(f"[INFO] required vz = {vz_min:.2f} m/s")


# =========================
# Step 2: generate time grid
# =========================

T_total = T_prepare + T_takeoff + T_air
ts = np.arange(0, T_total, dt)

# Output arrays
waist_pos_ref = np.zeros_like(ts)
c_pitch_vel_ref = np.zeros_like(ts)
wheel_vel_ref = np.zeros_like(ts)

# =========================
# Step 3: Phase A (preload / energy storage)
# =========================

for i, t in enumerate(ts):
    if t <= T_prepare:
        waist_pos_ref[i], _ = quintic(
            t, T_prepare,
            0.0, 0.0, 0.0,
            waist_front, 0.0, 0.0
        )

# =========================
# Step 4: Phase B (takeoff)
# =========================

for i, t in enumerate(ts):
    if T_prepare < t <= T_prepare + T_takeoff:
        tau = t - T_prepare

        # pitch 角速度 → vz
        c_pitch_vel_ref[i], _ = quintic(
            tau, T_takeoff,
            0.0, 0.0, 0.0,
            vz_min, 0.0, 0.0
        )

        wheel_vel_ref[i] = wheel_vel_max

# =========================
# Step 5: Phase C (in-air recovery)
# =========================

for i, t in enumerate(ts):
    if t > T_prepare + T_takeoff:
        tau = t - (T_prepare + T_takeoff)
        waist_pos_ref[i], _ = quintic(
            tau, T_air,
            waist_front, 0.0, 0.0,
            waist_recover, 0.0, 0.0
        )

        wheel_vel_ref[i] = 0.0
        c_pitch_vel_ref[i] = 0.0


# =========================
# Step 6: save CSV
# =========================

with open("jump_traj.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "c_pitch_vel_ref", "waist_pos_ref", "wheel_vel_ref"])
    for i in range(len(ts)):
        writer.writerow([
            ts[i],
            c_pitch_vel_ref[i],
            waist_pos_ref[i],
            wheel_vel_ref[i]
        ])

print("[DONE] jump_traj.csv generated")
