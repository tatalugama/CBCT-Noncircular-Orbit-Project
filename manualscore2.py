"""
Score one or more orbits manually against the Unity headless build.
No optimizer — just input orbits and get back scores.

Usage:
  py -3.10 manual_score.py

Orbits come from two places:
  1. the ORBITS dict below, typed in by hand
  2. any CSV listed in CSV_FILES (same format the optimizer writes)
Everything is scored in one Unity launch.
"""

import csv
import json
import os
import subprocess

BUILD    = r"E:\Thehan\Headless2\CBCT.exe"
WORK_DIR = "batch"

DOF_NAMES = ['Lateral_deg', 'CranioCaudal_deg', 'InPlane_deg',
             'Gantry_cm',   'ColumnHeight_cm',  'GantryLR_cm']

# ── CSVs to load ──────────────────────────────────────────────────────────────
# Add paths here — each becomes an orbit alongside the ones in ORBITS below.

CSV_FILES = [
    "orbit_results_orbit.csv",
]

# ── Orbits typed in by hand ───────────────────────────────────────────────────
# Each orbit is 8 steps x 6 DOF.
# Format: one row per step [Lateral, CranioCaudal, InPlane, Gantry, ColumnHeight, GantryLR]

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
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
        [ 0.0,    0.0,   0.0,    0.0,    0.0,       0.0  ],
    ],
}

# ─────────────────────────────────────────────────────────────────────────────

def load_csv(path):
    """Read an orbit CSV into the same 8x6 list-of-rows format as ORBITS."""
    with open(path, newline='') as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]

    # Skip the header row if the first cell isn't a number
    try:
        float(rows[0][0])
    except ValueError:
        rows = rows[1:]

    if len(rows) != 8:
        raise ValueError(f"expected 8 data rows, got {len(rows)}")

    return [[float(v.strip()) for v in row[:6]] for row in rows]


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

    subprocess.run([
        BUILD,
        "-batchmode", "-nographics",
        "-logFile", os.path.abspath(log_file),
        "-in",  os.path.abspath(in_file),
        "-out", os.path.abspath(out_file),
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not os.path.exists(out_file):
        raise RuntimeError(f"Unity wrote no results — see {log_file}")

    with open(out_file) as f:
        return json.load(f)["results"]


def main():
    orbits = dict(ORBITS)

    # Load each CSV and add it under its filename
    for path in CSV_FILES:
        if not os.path.exists(path):
            print(f"[skip] {path} not found")
            continue
        try:
            orbits[os.path.basename(path)] = load_csv(path)
        except Exception as e:
            print(f"[skip] {path}: {e}")

    labels     = list(orbits.keys())
    candidates = []

    for label in labels:
        flat = []
        for row in orbits[label]:
            flat.extend(row)
        if len(flat) != 48:
            raise ValueError(f"Orbit '{label}' has {len(flat)} values, expected 48")
        candidates.append(flat)

    print(f"Scoring {len(candidates)} orbit(s) against {BUILD}...\n")
    results = run_unity(candidates)

    for label, r in zip(labels, results):
        print(f"{'='*60}")
        print(f"Orbit: {label}")
        print(f"  fitness          {r['fitness']:+.0f}")
        print(f"  sampling         {r['sampling']:+.0f}  ({r['sampledVoxels']}/{r['roiVoxels']} ROI voxels)")
        print(f"  collision        {r['collisionPenalty']:+.0f}  "
              f"(touching {int(r.get('touchingProjections', 0))} projections, "
              f"max {r['maxDepthCm']:.2f} cm)")

        rows = orbits[label]
        print(f"  cumulative position after each step:")
        cum = [0.0] * 6
        for i, row in enumerate(rows):
            for d in range(6):
                cum[d] += row[d]
            parts = [f"{DOF_NAMES[d].replace('_deg','').replace('_cm','')}={cum[d]:+.1f}"
                     for d in range(6)]
            print(f"    step {i+1}: {', '.join(parts)}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()