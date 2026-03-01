#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from datetime import datetime

import numpy as np

STATE_KEYS = [
    "theta_r", "theta_b", "theta_w",
    "theta_dot_r", "theta_dot_b", "theta_dot_w"
]

def is_hdf5_mat(path: str) -> bool:
    """MATLAB v7.3 uses HDF5 signature."""
    with open(path, "rb") as f:
        sig = f.read(8)
    return sig == b"\x89HDF\r\n\x1a\n"

def load_mat_any(path: str) -> dict:
    """Load MATLAB .mat (v7.3 HDF5 or older) into a dict of numpy arrays."""
    if is_hdf5_mat(path):
        import h5py
        d = {}
        with h5py.File(path, "r") as f:
            for k in f.keys():
                obj = f[k]
                if hasattr(obj, "shape"):  # dataset
                    d[k] = np.squeeze(np.array(obj))
        return d
    else:
        from scipy.io import loadmat
        raw = loadmat(path)
        return {k: v for k, v in raw.items() if not k.startswith("__")}

def squeeze_1d(x):
    return np.squeeze(np.array(x))

def get_vec(d: dict, key: str):
    return squeeze_1d(d[key]) if key in d else None

def normalize_u(u) -> np.ndarray:
    """Return u as Nx2 (columns: u_waist, u_wheel)."""
    u = np.squeeze(np.array(u))
    if u.ndim != 2:
        raise RuntimeError(f"u 应该是二维数组，但得到 shape={u.shape}")
    # allow 2xN or Nx2
    if u.shape[0] == 2 and u.shape[1] != 2:
        return u.T
    if u.shape[1] == 2:
        return u
    raise RuntimeError(f"u 形状不对: {u.shape} (期望 2xN 或 Nx2)")

def script_dir() -> str:
    """Directory where this script is located."""
    return os.path.dirname(os.path.abspath(__file__))

def choose_mat_file_in_script_dir() -> str:
    """
    Search .mat in the script directory (NOT current working directory).
    Priority: data3_high.mat, else the first .mat alphabetically.
    """
    sdir = script_dir()

    cand = os.path.join(sdir, "data3_high.mat")
    if os.path.exists(cand):
        return cand

    mats = [f for f in os.listdir(sdir) if f.lower().endswith(".mat")]
    if not mats:
        raise RuntimeError(
            f"在脚本目录找不到任何 .mat 文件：{sdir}\n"
            f"请把 .mat 文件放到该目录，或运行时传入 mat 路径参数。"
        )
    mats.sort()
    return os.path.join(sdir, mats[0])

def main():
    # If user provides a mat path, use it; otherwise search in script directory
    if len(sys.argv) >= 2:
        mat_path = sys.argv[1]
        if not os.path.isabs(mat_path):
            # allow relative path from current working dir
            mat_path = os.path.abspath(mat_path)
    else:
        mat_path = choose_mat_file_in_script_dir()

    if not os.path.exists(mat_path):
        raise RuntimeError(f"找不到文件: {mat_path}")

    d = load_mat_any(mat_path)

    t = get_vec(d, "t")
    if t is None:
        raise RuntimeError("mat 文件里找不到变量 't'。")

    u_raw = d.get("u", None)
    if u_raw is None:
        raise RuntimeError("mat 文件里找不到变量 'u'。")
    u = normalize_u(u_raw)  # Nx2

    # states
    X_cols = []
    missing = []
    for k in STATE_KEYS:
        v = get_vec(d, k)
        if v is None:
            missing.append(k)
        else:
            X_cols.append(v)
    if missing:
        raise RuntimeError(f"mat 文件里缺少这些变量: {missing}")

    X = np.vstack(X_cols).T  # Nx6

    # length check
    N = len(t)
    if X.shape[0] != N:
        raise RuntimeError(f"长度不一致: len(t)={N}, X.shape={X.shape}")
    if u.shape[0] != N:
        raise RuntimeError(f"长度不一致: len(t)={N}, u.shape={u.shape}")

    # optional extras if present (append at end)
    extra_names = []
    extra_cols = []
    for opt_key in ["vcy", "vcx", "acy", "yc"]:
        v = get_vec(d, opt_key)
        if v is not None and len(v) == N:
            extra_names.append(opt_key)
            extra_cols.append(v)

    out = np.column_stack(
        [t, X, u] + ([np.vstack(extra_cols).T] if extra_cols else [])
    )

    header = ["t"] + STATE_KEYS + ["u_waist", "u_wheel"] + extra_names

    base = os.path.splitext(os.path.basename(mat_path))[0]

    # 时间戳：YYYYMMDD-HHMMSS（例如 20251229-163045）
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")

    out_txt = os.path.join(script_dir(), f"{base}_traj_{ts}.txt")

    # Write TXT with a real first line header (no '#')
    with open(out_txt, "w", encoding="utf-8") as f:
        f.write("\t".join(header) + "\n")
        np.savetxt(f, out, delimiter="\t", fmt="%.8f")

    print(f"[OK] Loaded: {mat_path}")
    print(f"[OK] Saved:  {out_txt}")
    print(f"[INFO] Columns: {' '.join(header)}")
    print(f"[INFO] Script dir: {script_dir()}")

if __name__ == "__main__":
    main()
