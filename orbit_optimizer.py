"""
CMA-ES optimizer for C-arm orbit design.
Unity is a pure objective function. Per generation:
  1. write batch/candidates.json  — 20 candidates, 48 floats each
  2. launch the headless build with -in / -out
  3. read batch/results.json      — fitness plus a breakdown per candidate
Outputs on completion:
  orbit_results.json        full summary
  orbit_results_orbit.csv   the best orbit, in the format the interactive
                            Unity UI's "Import Orbit CSV" button expects
"""
import numpy as np
import cma
import csv
import json
import os
import subprocess
import time

# ── Paths ─────────────────────────────────────────────────────────────────────

WORK_DIR        = "batch"
CANDIDATES_FILE = os.path.join(WORK_DIR, "candidates.json")
RESULTS_FILE    = os.path.join(WORK_DIR, "results.json")
UNITY_LOG_FILE  = os.path.join(WORK_DIR, "unity.log")

SUMMARY_FILE = "orbit_results.json"
ORBIT_CSV    = "orbit_results_orbit.csv"

# ── Problem shape ─────────────────────────────────────────────────────────────

NUM_STEPS  = 8
NUM_DOF    = 6
NUM_PARAMS = NUM_STEPS * NUM_DOF

DOF_NAMES = ['Lateral_deg', 'CranioCaudal_deg', 'InPlane_deg',
             'Gantry_cm',   'ColumnHeight_cm',  'GantryLR_cm']

# ── Joint limits ──────────────────────────────────────────────────────────────

LATERAL_MAX_PER_STEP = 180.0

CUM_LIMIT = np.array(
    [None, 45.0, 60.0, 20.0, (-8.0, 20.0), 20.0],
    dtype=object
)

# ── Warm start ────────────────────────────────────────────────────────────────
# Edit these values to seed the optimizer with a known good orbit.
# Each row is one step: [Lateral, CranioCaudal, InPlane, Gantry, ColumnHeight, GantryLR]
# These are PER-STEP DELTAS, same format as the "per-step deltas" printout.
# To start from scratch, leave all values as 0.0.

WARM_START = np.array([
    # Lateral  CrCd    InPlane  Gantry   ColHeight  GantryLR
       45.0,    -10.0,    0.0,     0.0,     0.0,       0.0,   # step 1
       45.0,    10.0,    0.0,     0.0,     0.0,       0.0,   # step 2
       45.0,    10.0,    0.0,     0.0,     0.0,       0.0,   # step 3
       45.0,    -10.0,    0.0,     0.0,     0.0,       0.0,   # step 4
       45.0,    -10.0,    0.0,     0.0,     0.0,       0.0,   # step 5
       45.0,    10.0,    0.0,     0.0,     0.0,       0.0,   # step 6
       45.0,    10.0,    0.0,     0.0,     0.0,       0.0,   # step 7
       45.0,    -10.0,    0.0,     0.0,     0.0,       0.0,   # step 8
], dtype=float)

# ── Search settings ───────────────────────────────────────────────────────────

SIGMA_PER_DOF = np.array([0.01, 0.01, 3.0, 2, 2, 2])

CMAES_OPTIONS = {
    'maxiter':        500,
    'popsize':        20,
    'tolflatfitness': 20,
    'tolstagnation':  500,
    'tolfun':         0,
    'tolx':           1e-6,
    'verb_disp':      0,
    'verb_log':       0,
    'CMA_stds':       np.tile(SIGMA_PER_DOF, NUM_STEPS),
}

# ── Joint clamping ────────────────────────────────────────────────────────────

def clamp_cumulative(x):
    v   = np.array(x, dtype=float).reshape(NUM_STEPS, NUM_DOF)
    out = np.zeros_like(v)
    cum = np.zeros(NUM_DOF)

    for s in range(NUM_STEPS):
        for d in range(NUM_DOF):
            if d == 0:
                applied = float(np.clip(v[s, d],
                                        -LATERAL_MAX_PER_STEP,
                                         LATERAL_MAX_PER_STEP))
            else:
                lim = CUM_LIMIT[d]
                if isinstance(lim, tuple):
                    lo, hi = lim
                else:
                    lo, hi = -float(lim), float(lim)
                target  = float(np.clip(cum[d] + v[s, d], lo, hi))
                applied = target - cum[d]
            cum[d]   += applied
            out[s, d] = applied

    return out.reshape(-1)


def cumulative_trace(x):
    return np.cumsum(np.array(x, dtype=float).reshape(NUM_STEPS, NUM_DOF), axis=0)

# ── Unity ─────────────────────────────────────────────────────────────────────

class UnityScorer:
    def __init__(self, exe_path, timeout=900):
        if not os.path.isfile(exe_path):
            raise FileNotFoundError(f"Unity build not found: {exe_path}")
        self.exe_path = exe_path
        self.timeout  = timeout
        self.batch    = 0
        os.makedirs(WORK_DIR, exist_ok=True)

    def score(self, candidates):
        self.batch += 1

        payload = {"candidates": [{"p": [float(v) for v in c]} for c in candidates]}
        with open(CANDIDATES_FILE, 'w') as f:
            json.dump(payload, f)

        if os.path.exists(RESULTS_FILE):
            os.remove(RESULTS_FILE)

        cmd = [
            self.exe_path,
            "-batchmode", "-nographics",
            "-logFile", os.path.abspath(UNITY_LOG_FILE),
            "-in",  os.path.abspath(CANDIDATES_FILE),
            "-out", os.path.abspath(RESULTS_FILE),
        ]

        try:
            subprocess.run(cmd, timeout=self.timeout,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"Unity timed out after {self.timeout}s — see {UNITY_LOG_FILE}")

        if not os.path.exists(RESULTS_FILE):
            raise RuntimeError(f"Unity wrote no results — see {UNITY_LOG_FILE}")

        with open(RESULTS_FILE) as f:
            results = json.load(f)["results"]

        if len(results) != len(candidates):
            raise RuntimeError(f"sent {len(candidates)} candidates, got {len(results)} results")

        return results

# ── Main loop ─────────────────────────────────────────────────────────────────

def optimize(exe_path, lateral_seed=0.0, cma_seed=42):
    scorer = UnityScorer(exe_path)

    # Build the starting mean: WARM_START, with lateral_seed applied if nonzero
    seed = WARM_START.copy()
    if lateral_seed != 0.0:
        for s in range(NUM_STEPS):
            seed[s * NUM_DOF + 0] = lateral_seed

    CMAES_OPTIONS['seed'] = cma_seed

    print("\n[CMA-ES] starting")
    print(f"  parameters   {NUM_PARAMS} ({NUM_STEPS} steps x {NUM_DOF} DOF)")
    print(f"  warm start   {'all zeros' if np.all(seed == 0) else 'custom'}")
    print(f"  cma seed     {cma_seed}")
    print(f"  population   {CMAES_OPTIONS['popsize']}")
    print(f"  limits       Lateral free (max {LATERAL_MAX_PER_STEP:.0f} deg/step per step)")
    for d in range(1, NUM_DOF):
        lim = CUM_LIMIT[d]
        if isinstance(lim, tuple):
            print(f"               {DOF_NAMES[d]}: [{lim[0]}, +{lim[1]}] cumulative")
        else:
            print(f"               {DOF_NAMES[d]}: +/-{lim} cumulative")
    print(f"  build        {exe_path}\n")

    es = cma.CMAEvolutionStrategy(seed, 1.0, CMAES_OPTIONS)

    best_fit = -np.inf
    best_p   = None
    best_r   = None
    evals    = 0
    t0       = time.time()

    while not es.stop():
        raw     = es.ask()
        clamped = [clamp_cumulative(c) for c in raw]

        try:
            results = scorer.score(clamped)
        except Exception as e:
            print(f"\n[ERROR] {e}")
            break

        fits = [r["fitness"] for r in results]
        es.tell(raw, [-f for f in fits])
        evals += len(clamped)

        i = int(np.argmax(fits))
        if fits[i] > best_fit:
            best_fit, best_p, best_r = fits[i], clamped[i].copy(), results[i]

        r = results[i]
        print(f"gen {scorer.batch:4d}  evals {evals:6d}  "
              f"best {best_fit:+12.0f}  gen {fits[i]:+12.0f}  "
              f"[sampled {r['sampledVoxels']:5d}/{r['roiVoxels']}  "
              f"coll {r['collisionPenalty']:10.0f} (max {r['maxDepthCm']:5.1f}cm)  "
              f"cent {r['centeringPenalty']:10.0f} (max {r['maxMissCm']:5.1f}cm)]  "
              f"{(time.time() - t0) / 60:.1f}min")

    print(f"\n[CMA-ES] stopped: {es.stop()}")
    print(f"[CMA-ES] {evals} evaluations in "
          f"{time.strftime('%H:%M:%S', time.gmtime(time.time() - t0))}")

    if best_p is None:
        print("[CMA-ES] nothing to save")
        return None, None

    save(best_p, best_fit, best_r, evals)
    return best_p, best_fit

# ── Output ────────────────────────────────────────────────────────────────────

def save(p, fitness, result, evals):
    steps = [{DOF_NAMES[d]: float(p[s * NUM_DOF + d]) for d in range(NUM_DOF)}
             for s in range(NUM_STEPS)]

    with open(SUMMARY_FILE, 'w') as f:
        json.dump({
            'fitness':     float(fitness),
            'evaluations': evals,
            'breakdown':   result,
            'orbit':       steps,
            'params':      [float(v) for v in p],
        }, f, indent=2)

    with open(ORBIT_CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(DOF_NAMES)
        for step in steps:
            w.writerow([f"{step[k]:.4f}" for k in DOF_NAMES])

    print(f"\n[saved] {SUMMARY_FILE}")
    print(f"[saved] {ORBIT_CSV}  (import with Unity UI's Import Orbit CSV button)")

    print(f"\n[breakdown] fitness {result['fitness']:.0f} = "
          f"sampling {result['sampling']:.0f} "
          f"- collision {result['collisionPenalty']:.0f} "
          f"- centering {result['centeringPenalty']:.0f}")
    print(f"            {result['sampledVoxels']}/{result['roiVoxels']} ROI voxels sampled, "
          f"worst penetration {result['maxDepthCm']:.1f}cm, "
          f"worst miss {result['maxMissCm']:.1f}cm")

    print("\n[orbit — per-step deltas]")
    for i, step in enumerate(steps):
        parts = [f"{k.replace('_deg','').replace('_cm','')} {v:+7.1f}"
                 for k, v in step.items()]
        print(f"  step {i+1}: {'  '.join(parts)}")

    print("\n[cumulative position after each step]")
    trace = cumulative_trace(p)
    for s in range(NUM_STEPS):
        parts = [f"{DOF_NAMES[d].replace('_deg','').replace('_cm','')} {trace[s, d]:+7.1f}"
                 for d in range(NUM_DOF)]
        print(f"  step {s+1}: {'  '.join(parts)}")

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='C-arm orbit optimizer')
    ap.add_argument('--build', required=True, help='headless Unity build exe')
    ap.add_argument('--lateral-seed', type=float, default=0.0,
                    help='lateral deg per step for the warm start '
                         '(45 seeds a full 360 deg sweep)')
    ap.add_argument('--seed', type=int, default=42,
                    help='CMA-ES random seed (change per instance for parallel runs)')
    args = ap.parse_args()
    optimize(args.build, args.lateral_seed, args.seed)