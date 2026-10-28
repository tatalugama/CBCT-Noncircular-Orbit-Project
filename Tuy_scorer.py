"""
Score orbits and compute Tuy completeness in one pass.

  py -3.10 orbit_analysis.py

Orbits come from two places, both scored together:
  1. the ORBITS dict below, typed in by hand
  2. every .csv in ORBIT_DIR — the format the optimizer writes
     (8 rows of per-step deltas, header optional)

Output per orbit: FOV score, collision score, fitness, Tuy completeness.
Also writes orbit_analysis.csv.

Note: TRAJ_DIR is an OUTPUT folder. Unity writes source-position CSVs there
and it is cleared at the start of every run — don't put anything in it.

Requires a Unity build whose BatchRunner writes trajectories, since the CSVs
need the '# head_unity=' and '# roi_radius_m=' header lines.
"""

import csv
import json
import os
import re
import shutil
import subprocess

import numpy as np
import pandas as pd

# ── Config ────────────────────────────────────────────────────────────────────

BUILD     = r"E:\Thehan\HeadlessC\CBCT.exe"

ORBIT_DIR = r"E:\Thehan\CBCT Result Orbits\Final Orbits"          # INPUT  — folder of orbit-definition CSVs
TRAJ_DIR  = "trajectories"    # OUTPUT — Unity writes source geometry here
WORK_DIR  = "batch"
OUT_CSV   = "orbit_analysis.csv"

# Tuy settings
N_PLANES           = 2000    # sampled plane orientations
ANG_THRES_DEG      = 1.0
ANG_THRES_ONLY     = False   # False = adaptive tolerance from source spacing
EVAL_SPACING_M     = 0.02    # evaluation grid spacing inside the ROI
COMPLETE_THRESHOLD = 0.99    # a point counts as complete at or above this

# Unity -> CT frame
UNITY_UNITS_TO_MM = 1000.0            # 1 Unity unit = 1 m
UNITY_TO_CT_AXES  = ('x', 'z', 'y')   # Unity LH Y-up -> CT RH Z-up

NUM_STEPS = 8
NUM_DOF   = 6

# ── Orbits typed in by hand ───────────────────────────────────────────────────
# Rows are steps; columns are [Lateral, CrCd, InPlane, Gantry, ColHeight, GantryLR].

ORBITS = {

    "pure_circular_45": [
        # Lateral  CrCd   InPlane  Gantry  ColHeight  GantryLR
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
    ],

    "zero_motion": [
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
        [  0.0,   0.0,   0.0,    0.0,    0.0,       0.0  ],
    ],

    # Reynolds et al. sawtooth: 360 deg lateral, f=2, +/-25 deg cranial
    "raw_sawtooth_20deg": [
        [ 45.0, +20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -20.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +20.0,   0.0,    0.0,    0.0,       0.0  ],
    ],
    "raw_sawtooth_15deg": [
        [ 45.0, +15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -15.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +15.0,   0.0,    0.0,    0.0,       0.0  ],
    ],
    "raw_sawtooth_10deg": [
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
    ],

    "raw_sine_10deg": [
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, -10.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 45.0, +10.0,   0.0,    0.0,    0.0,       0.0  ],
    ],

   "rawDoubleCircle10": [
       # Lateral  CrCd    InPlane  Gantry   ColHeight  GantryLR
        [0.0,    10.0,    0.0,     0.0,     0.0,       0.0],   # step 1
        [180.0,    -20.0,    0.0,     0.0,     0.0,       0.0],   # step 2
        [180.0,    20.0,    0.0,     0.0,     0.0,       0.0],   # step 3
        [0.0,    -10.0,    0.0,     0.0,     0.0,       0.0],   # step 4
        [0.0,    -10.0,    0.0,     0.0,     0.0,       0.0],   # step 5
        [180.0,    20.0,    0.0,     0.0,     0.0,       0.0],   # step 6
        [180.0,    -20.0,    0.0,     0.0,     0.0,       0.0],   # step 7
        [0.0,    10.0,    0.0,     0.0,     0.0,       0.0],   # step 8
    ], 
    "rawDoubleCircle20": [
    # Lateral  CrCd    InPlane  Gantry   ColHeight  GantryLR
       [0.0,    20.0,    0.0,     0.0,     0.0,       0.0],   # step 1
       [180.0,    -40.0,    0.0,     0.0,     0.0,       0.0],   # step 2
       [180.0,    40.0,    0.0,     0.0,     0.0,       0.0],   # step 3
       [0.0,    -20.0,    0.0,     0.0,     0.0,       0.0],   # step 4
       [0.0,    -20.0,    0.0,     0.0,     0.0,       0.0],   # step 5
       [180.0,    40.0,    0.0,     0.0,     0.0,       0.0],   # step 6
       [180.0,    -40.0,    0.0,     0.0,     0.0,       0.0],   # step 7
       [0.0,    20.0,    0.0,     0.0,     0.0,       0.0],   # step 8
    ],
    "rawDoubleCircleArc": [
    # Lateral  CrCd    InPlane  Gantry   ColHeight  GantryLR
       [0.0,    0.0,    0.0,     -5.0,     0.0,       0.0],   # step 1
       [180.0,    0.0,    0.0,     0.0,     0.0,       0.0],   # step 2
       [180.0,    0.0,    0.0,     0.0,     0.0,       0.0],   # step 3
       [0.0,    0.0,    0.0,     5.0,     0.0,       0.0],   # step 4
       [0.0,    0.0,    0.0,     5.0,     0.0,       0.0],   # step 5
       [180.0,    0.0,    0.0,     0.0,     0.0,       0.0],   # step 6
       [180.0,    0.0,    0.0,     0.0,     0.0,       0.0],   # step 7
       [0.0,    0.0,    0.0,     -10.0,     0.0,       0.0],   # step 8
    ],


}

# ── Orbit CSV loading ─────────────────────────────────────────────────────────

def load_orbit_csv(path):
    """
    Read an orbit-definition CSV into 8 rows of 6 floats — the same shape as
    the ORBITS entries. Accepts the optimizer's header row or none at all.
    """
    with open(path, newline='') as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]

    if not rows:
        raise ValueError("file is empty")

    # Header present if the first cell won't parse as a number
    try:
        float(rows[0][0])
    except ValueError:
        rows = rows[1:]

    if len(rows) != NUM_STEPS:
        raise ValueError(f"expected {NUM_STEPS} data rows, got {len(rows)}")

    return [[float(v.strip()) for v in row[:NUM_DOF]] for row in rows]


def collect_orbits():
    """ORBITS dict first, then every CSV in ORBIT_DIR, keyed by filename."""
    orbits = dict(ORBITS)

    if not os.path.isdir(ORBIT_DIR):
        print(f"[note] no '{ORBIT_DIR}' folder — using only the ORBITS dict")
        return orbits

    for fn in sorted(os.listdir(ORBIT_DIR)):
        if not fn.lower().endswith('.csv'):
            continue
        path = os.path.join(ORBIT_DIR, fn)
        label = os.path.splitext(fn)[0]
        try:
            orbits[label] = load_orbit_csv(path)
        except Exception as e:
            print(f"[skip] {fn}: {e}")

    return orbits

# ── Unity ─────────────────────────────────────────────────────────────────────

def run_unity(candidates):
    os.makedirs(WORK_DIR, exist_ok=True)
    in_file  = os.path.join(WORK_DIR, "candidates.json")
    out_file = os.path.join(WORK_DIR, "results.json")
    log_file = os.path.join(WORK_DIR, "unity.log")

    payload = {"candidates": [{"p": [float(v) for v in c]} for c in candidates]}
    with open(in_file, 'w') as f:
        json.dump(payload, f)

    if os.path.exists(out_file):
        os.remove(out_file)
    if os.path.isdir(TRAJ_DIR):
        shutil.rmtree(TRAJ_DIR)

    subprocess.run([
        BUILD,
        "-batchmode", "-nographics",
        "-logFile",  os.path.abspath(log_file),
        "-in",       os.path.abspath(in_file),
        "-out",      os.path.abspath(out_file),
        "-trajdir",  os.path.abspath(TRAJ_DIR),
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not os.path.exists(out_file):
        raise RuntimeError(f"Unity wrote no results — see {log_file}")

    with open(out_file) as f:
        return json.load(f)["results"]

# ── Tuy ───────────────────────────────────────────────────────────────────────

_AXIS_IDX = {'x': 0, 'y': 1, 'z': 2}
_REMAP = [_AXIS_IDX[a] for a in UNITY_TO_CT_AXES]


def unity_to_ct_mm(points_xyz):
    points_xyz = np.atleast_2d(np.asarray(points_xyz, dtype=float))
    return points_xyz[:, _REMAP] * UNITY_UNITS_TO_MM


def fibonacci_sphere(n_planes):
    i = np.arange(n_planes)
    phi = np.arccos(1 - 2 * (i + 0.5) / n_planes)
    golden = np.pi * (1 + 5 ** 0.5)
    theta = golden * i
    return np.stack([np.sin(phi) * np.cos(theta),
                     np.sin(phi) * np.sin(theta),
                     np.cos(phi)], axis=1)


def tuy_completeness(eval_locs, src_loc_w, norm_vec_planes,
                     ang_thres=1.0, ang_thres_only=False):
    eval_locs = np.asarray(eval_locs)
    n_proj = src_loc_w.shape[1]
    metric = np.zeros(len(eval_locs))

    for i, loc in enumerate(eval_locs):
        src_loc_vec = src_loc_w - loc[:, np.newaxis]
        src_loc_vec = src_loc_vec / np.linalg.norm(src_loc_vec, axis=0)[np.newaxis, :]

        cos_angle = np.clip(np.matmul(norm_vec_planes, src_loc_vec), -1., 1.)
        angles = np.arccos(cos_angle) - np.pi / 2.

        if ang_thres_only:
            angle_l = -np.deg2rad(ang_thres) * np.ones(n_proj)
            angle_u = np.deg2rad(ang_thres) * np.ones(n_proj)
        else:
            temp1 = src_loc_vec[:, 1:].T
            temp2 = src_loc_vec[:, :-1].T
            cos_adj = np.clip(np.einsum('ij,ij->i', temp1, temp2), -1., 1.)
            ang_adj = np.abs(np.arccos(cos_adj)) / 2.
            angle_u = np.minimum(np.hstack((ang_adj, ang_adj[-1])), np.deg2rad(ang_thres))
            angle_l = -np.minimum(np.hstack((ang_adj[0], ang_adj)), np.deg2rad(ang_thres))

        crossed = np.logical_and(angles >= angle_l[np.newaxis, :],
                                 angles < angle_u[np.newaxis, :])
        metric[i] = np.mean(np.sum(crossed, axis=1) >= 1)

    return metric


def read_traj_header(path):
    """Pull head_unity and roi_radius_m out of the '#' comment header."""
    head, radius = None, None
    with open(path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            m = re.search(r'head_unity=([-\d.eE]+),([-\d.eE]+),([-\d.eE]+)', line)
            if m:
                head = np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))])
            m = re.search(r'roi_radius_m=([-\d.eE]+)', line)
            if m:
                radius = float(m.group(1))
    return head, radius


def roi_grid_unity(head_unity, radius_m, spacing_m):
    n = max(1, int(np.floor(radius_m / spacing_m)))
    axis = np.arange(-n, n + 1) * spacing_m
    gx, gy, gz = np.meshgrid(axis, axis, axis, indexing='ij')
    offs = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    offs = offs[np.linalg.norm(offs, axis=1) <= radius_m]
    return np.asarray(head_unity, dtype=float)[None, :] + offs


def analyse_tuy(path, planes):
    head, radius = read_traj_header(path)
    if head is None or radius is None:
        raise ValueError("no '# head_unity=' / '# roi_radius_m=' header")

    df = pd.read_csv(path, comment='#')
    src = unity_to_ct_mm(df[['src_x', 'src_y', 'src_z']].to_numpy()).T

    pts = unity_to_ct_mm(roi_grid_unity(head, radius, EVAL_SPACING_M))
    vals = tuy_completeness(pts, src, planes,
                            ang_thres=ANG_THRES_DEG,
                            ang_thres_only=ANG_THRES_ONLY)

    return {
        'tuy_mean':          float(np.mean(vals)),
        'tuy_effective_fov': 100.0 * float(np.mean(vals >= COMPLETE_THRESHOLD)),
    }

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    orbits = collect_orbits()
    labels = list(orbits.keys())

    if not labels:
        print("Nothing to score.")
        return

    candidates = []
    for label in labels:
        rows = orbits[label]
        if len(rows) != NUM_STEPS:
            raise ValueError(f"'{label}' has {len(rows)} steps, expected {NUM_STEPS}")
        flat = [v for row in rows for v in row]
        if len(flat) != NUM_STEPS * NUM_DOF:
            raise ValueError(f"'{label}' flattens to {len(flat)} values, "
                             f"expected {NUM_STEPS * NUM_DOF}")
        candidates.append(flat)

    print(f"\nScoring {len(candidates)} orbit(s) against {BUILD}")
    results = run_unity(candidates)

    print(f"Computing Tuy completeness "
          f"({N_PLANES} planes, {EVAL_SPACING_M * 100:.0f}cm grid)\n")
    planes = fibonacci_sphere(N_PLANES)

    rows_out = []
    for i, (label, r) in enumerate(zip(labels, results)):
        row = {
            'orbit':             label,
            'fov':               r['sampling'],
            'collision':         r['collisionPenalty'],
            'fitness':           r['fitness'],
            'tuy_mean':          float('nan'),
            'tuy_effective_fov': float('nan'),
        }

        traj = os.path.join(TRAJ_DIR, f"traj_{i:03d}.csv")
        if os.path.exists(traj):
            try:
                row.update(analyse_tuy(traj, planes))
                os.replace(traj, os.path.join(TRAJ_DIR, f"{label}.csv"))
            except Exception as e:
                print(f"  [tuy skipped] {label}: {e}")
        else:
            print(f"  [tuy skipped] {label}: no trajectory file")

        rows_out.append(row)

    hdr = (f"{'orbit':<28}{'FOV':>10}{'collision':>12}{'fitness':>12}"
           f"{'tuy mean':>11}{'tuy FOV':>10}")
    print(hdr)
    print("-" * len(hdr))
    for row in rows_out:
        tm = ("      n/a" if np.isnan(row['tuy_mean'])
              else f"{row['tuy_mean'] * 100:9.2f}%")
        tf = ("     n/a" if np.isnan(row['tuy_effective_fov'])
              else f"{row['tuy_effective_fov']:8.2f}%")
        print(f"{row['orbit']:<28}"
              f"{row['fov']:>10.0f}"
              f"{row['collision']:>12.0f}"
              f"{row['fitness']:>12.0f}"
              f"{tm:>11}{tf:>10}")

    with open(OUT_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        for row in rows_out:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v)
                        for k, v in row.items()})

    print(f"\n[saved] {OUT_CSV}")
    print(f"[saved] {TRAJ_DIR}/  — source geometry per orbit")


if __name__ == '__main__':
    main()