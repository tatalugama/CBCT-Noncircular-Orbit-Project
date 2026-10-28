# C-arm Orbit Simulator and Scorer

This software is split into two parts: A simulation of a Siemens Artis Pheno
C-arm created in the video game development software Unity, and a Python
toolchain that uses that simulation as an engine for an objective function to
design non-circular CBCT orbits. The problem to be solved is minimizing the
C-arm collisions with patient body and table while maximizing effective FOV.

We'll call the simulation the "game" and the objective function code the
"optimizer". In the game, you can move the arm around manually and input orbit
commands for each degree of freedom that it has. Collisions with the
patient/table object will make the apparatus turn red, and you can move the
patient/table object around as well. You can map out the field of view visually
with the "Trace" setting.

The optimizer works by using a "headless" (no visuals) build of the game as an
objective function. The Unity scene compiles to a headless executable that takes
a list of candidate orbits as JSON, simulates each one, and returns numbers —
how much of the region of interest got sampled, and how badly the arm hit the
table. Python drives that executable, either with CMA-ES to search for good
orbits or with a manual script to score orbits you typed in yourself.



IMPORTANT: For space issues, the folder "Headless2" had to be split into two folders. It is CRUCIAL that you copy the contents of "COPY\_INTO\_HEADLESS\_2" into "Headless2". Don't copy the whole folder, just the file inside.

\---

## What's in here

**The interactive Unity app.** The full scene with rendering, camera controls,
keyboard joint control, and the orbit-sequence UI. This is what you use to look
at things and sanity-check that an orbit does what you think it does.

**The headless build.** The same scene compiled with `Dedicated Server Optimizations` and no graphics. It has no UI and does nothing unless you pass it
command-line arguments. Its only job is to score orbits quickly and quit.

**The Unity scripts.** Both the interactive controls and the scoring code. They
live in the same project; the headless build just has the interactive ones
disabled at runtime.

**The Python toolchain.** The optimizer, the manual scorer, and the Tuy
completeness analysis.

\---

## Opening the project in Unity

The three folders `Assets/`, `Packages/`, and `ProjectSettings/` *are* the Unity
project. Everything else Unity needs it regenerates on its own, so there's
nothing to install or configure beyond getting the right editor version.

**1. Install the matching editor version.** The project was built with Unity
**6000.4.10f1**. Install that exact version through Unity Hub — go to Installs,
then Install Editor, then the Archive tab, and find it in the release list. The
version matters more than it looks like it should: if you open the project in a
newer editor, Unity will offer to upgrade it and then rewrite the serialized
scene and asset files on import. That usually works, but it is a one-way door,
and any difference it introduces in how the joint hierarchy or the colliders
deserialize will quietly change what the scorer measures. If you only have a
different version available, make a copy of the folders first so you still have
an unmodified original.

**2. Put the three folders together.** Create an empty folder — the name becomes
the project name — and place `Assets/`, `Packages/`, and `ProjectSettings/`
inside it, side by side at the top level. Don't nest them any deeper. If the
repo also contains a `python/` folder or loose scripts, those can sit alongside
the three without bothering Unity, since it only looks inside `Assets/`.

Avoid putting the project inside OneDrive, Dropbox, or any other synced folder.
Unity rewrites thousands of files in `Library/` constantly and sync clients fight
with it, which produces import errors that look like project corruption.

**3. Add it to Unity Hub.** In the Projects tab, use the Add button and pick
"Add project from disk," then select the folder you created — the one *containing*
the three, not one of the three. It should appear in the list with the right
editor version detected. Open it from there.

**4. Wait out the first import.** Opening a project with no `Library/` folder
means Unity imports every asset from scratch and recompiles all the scripts.
Depending on the machine this takes anywhere from a couple of minutes to
fifteen or more, and the editor will look frozen for stretches of it. Let it
finish. Unity also restores the packages listed in `Packages/manifest.json` on
this first open, which needs an internet connection — the Input System package
in particular, which this project depends on and which will produce a wall of
compile errors if it fails to resolve.

**5. Open the scene.** Unity may open to an empty untitled scene rather than the
real one. Find the `.unity` scene file in the Project window under `Assets/` and
double-click it. This is the step that actually loads the C-arm, the table, the
head marker, and the `OptimizationManager` object that carries the scorer.

**6. Check that it came through intact.** Some console noise on first open is
normal — shader warnings especially. What matters is the following three things.
Select the `OptimizationManager` object and confirm that `OrbitScorer`'s `Arm Roots` array is populated and its `Head` field points at the head marker, since
an empty `Arm Roots` means collisions silently score as zero. Confirm the table
and body objects still carry the `Obstacle` tag, because the scorer finds them by
tag and nothing else. And press Play to confirm the sequence panel draws and
right-click-dragging orbits the camera.

If the console shows `The referenced script on this Behaviour is missing!` for
specific GameObjects, a script those objects depend on didn't make it into
`Assets/` — the component is gone from the object and that part of the scene will
do nothing. Compare the list in the console against the scripts named in this
README to see what's absent.

### Making your own headless build

Once the project opens, a scoring build is made through File, then Build
Settings. Choose the Dedicated Server platform (or Windows standalone with
Dedicated Server Optimizations enabled), make sure the scene is the one checked
in the build list, and build to its own empty folder.

The thing to get right before you press Build is that **every Inspector value is
baked into the executable at that moment** — the scoring weights, the voxel grid
resolution, the ROI radius, the sampling threshold, the `Arm Roots` contents, and
the CranioCaudal motion profile. None of them can be changed afterwards and none
of them are visible from outside the build. So set them deliberately, build, then
run the executable once with `-batchmode -nographics -logFile` and keep the log
somewhere alongside the build. Those `\[OrbitScorer]` startup lines are the only
record of what that build contains, and you will want them later.

\---

## The machine model

The arm is a chain of nested GameObjects, each one a joint. The full path down to
the detector looks like this:

```
GantryFloor
 └ ColumnHeight
    └ Cylinder
       └ Joint1
          └ Cylinder (1)
             └ InPlane
                └ Lateral
                   └ CranioCaudal
                      └ SIDSlider
                         └ Virtual Module  (detector geometry + colliders)
```

Six degrees of freedom are driven, and they are always given in this order,
everywhere in the project:

|#|DOF|Type|Axis / direction|
|-|-|-|-|
|0|Lateral|rotation|local X — the main orbital sweep|
|1|CranioCaudal|rotation|local Y — tilts the orbit plane|
|2|InPlane|rotation|local Z|
|3|Gantry|translation|world-space direction set in the Inspector|
|4|ColumnHeight|translation|vertical|
|5|GantryLR|translation|lateral|

### The one thing that will bite you

**ColumnHeight and GantryLR are the same GameObject.** They move it in different
directions, but they share a single Transform. Any code that sets
`localPosition` once per DOF will silently throw away one of them — whichever is
written first gets overwritten by the second, and that DOF becomes dead without
any error.

The scoring code handles this by resetting all three translation joints to their
home positions first, then *adding* each contribution:

```csharp
// reset everything first
if (gantryJoint)       gantryJoint.localPosition       = gantryHome;
if (columnHeightJoint) columnHeightJoint.localPosition = columnHome;
if (gantryLRJoint)     gantryLRJoint.localPosition     = gantryLRHome;

// then add, so a shared transform accumulates both
if (gantryJoint)       gantryJoint.localPosition       += ...;
if (columnHeightJoint) columnHeightJoint.localPosition += ...;
if (gantryLRJoint)     gantryLRJoint.localPosition     += ...;
```

If you ever write new playback or scoring code, do it this way. The interactive
sequence builder gets this right already because its coroutines use `+=`.

\---

## How an orbit is represented

An orbit is **48 numbers**: 8 steps × 6 DOF. Each number is a *delta* applied
during that step, not an absolute position. So an orbit with `45.0` in the
Lateral column of all eight rows is a full 360° circular sweep, because the
deltas accumulate.

As a CSV it looks like this, which is the format the optimizer writes and the
Unity UI imports:

```
Lateral\_deg,CranioCaudal\_deg,InPlane\_deg,Gantry\_cm,ColumnHeight\_cm,GantryLR\_cm
45.0000,0.0000,0.0000,0.0000,0.0000,0.0000
45.0000,0.0000,0.0000,0.0000,0.0000,0.0000
...
```

As JSON going into the headless build, the same orbit is one flat array of 48
floats inside a candidates list.

Joint limits are enforced on the Python side, **cumulatively** — because each
step starts where the last one ended, a limit constrains the absolute joint
position, not the size of any single step. Two individually legal 35° cranial
steps must not be allowed to total 70°. The current limits are:

* Lateral — no end stop, free to accumulate past 360°, capped at 180° per step
* CranioCaudal — ±45° cumulative
* InPlane — ±60° cumulative
* Gantry, GantryLR — ±20 cm cumulative
* ColumnHeight — asymmetric, −8 cm to +20 cm (the floor is lower than the ceiling)

\---

## The scorer

`OrbitScorer.cs` is the objective function. Everything else exists to feed it or
read its output. Given 48 parameters it does the following:

**1. Build the cumulative pose table.** The 48 deltas are accumulated into
absolute joint positions at each of the 9 step boundaries (start plus 8 steps).

**2. Walk the orbit.** The orbit is sampled at `projections` evenly spaced points
(300 by default). At each one, the fraction along the orbit is mapped to a step
index and a fraction within that step, and every joint is interpolated between
its two bracketing waypoints. Poses are set *absolutely* from the cached home
pose every time — nothing is integrated frame to frame, so there's no drift and
no dependence on how many frames have elapsed.

**3. Check for collisions.** `Physics.ComputePenetration` is run on every
arm-collider × obstacle-collider pair at the current pose. Arm colliders are
collected from the roots listed in `Arm Roots`; obstacle colliders are found by
the `Obstacle` tag. The penalty for a projection with any overlap is:

```
binaryCollisionWeight                           always, for any contact at all
+ lightCollisionWeight × depth                  if depth < 5 cm
+ harshCollisionWeight × depth²                 if depth ≥ 5 cm
```

The flat binary term exists so that a very shallow but persistent graze still
costs something — without it, sub-millimetre contact across the whole orbit was
rounding to nearly zero.

**4. Cast the beam.** A lattice of rays is fired from the focal spot to a grid of
points across the detector face, and each ray is marched through a voxel grid
with a DDA traversal, incrementing a counter in every voxel it passes through.
The grid is a cube centred on the `head` transform, sized `roiRadiusM × 2`, and
only voxels inside the ROI sphere are counted toward the score.

**5. Score it.**

```
sampled   = ROI voxels whose counter reached samplingThreshold
fitness   = samplingWeight × sampled − collisionPenalty
```

### Things the scorer does not do

**There is no centering term.** There used to be one — it measured the distance
from the head to the central beam ray and penalized it. That turns out to be
exactly backwards for orbital imaging: a beam rotating *around* a point is
tangential to it, so a full 360° orbit scored worse than not moving at all. It
was removed. The sampling reward handles centering implicitly, since an orbit
that wanders away from the head stops hitting ROI voxels.

**Self-collision is not evaluated.** The loop is arm × obstacle only. The arm
hitting itself is never scored. (`Physics.IgnoreCollision` calls in the setup
affect the simulated physics and trigger events, not `ComputePenetration`, which
is a direct geometric query — the exclusion comes from the loop structure, not
from those calls.)

**Motion between sampled poses is invisible.** Only the discrete projections are
tested. A contact that begins and ends between two of them is never seen. At 300
projections over a 360° sweep that's about 1.2° apart.

**Anything untagged is invisible.** Only colliders tagged `Obstacle` are checked,
and `FindGameObjectsWithTag` doesn't search children — each object holding a
collider needs the tag itself. **The floor is currently untagged and has no
collider, so orbits can clip through it and still score zero collision.** That's
a known gap, not a bug in the scoring.

### CranioCaudal motion profile

`OrbitScorer` has a `Cranio Caudal Interp` setting that changes how CrCd moves
*between* waypoints while leaving the waypoints themselves alone:

* **Linear** — constant velocity with sharp reversals. This is a **sawtooth**, and
it matches what the real pheno does (see the velocity traces in Reynolds et
al. 2024, which are near-square waves).
* **Sinusoidal** — Catmull-Rom through the same waypoints. Velocity is continuous,
so peaks round off while zero-crossings stay fast. This is a genuine **sine**,
and it's the sine-on-sphere orbit.
* **SmoothStep** — eases in and out of *every* waypoint including the
zero-crossings, so it flattens where a real sine is fastest. Included for
completeness; it isn't a good model of either.

Lateral is always linear regardless of this setting.

This matters enormously when comparing builds. **Two builds with different
settings here will disagree about whether the same orbit collides**, because the
arm physically travels a different path. If you're comparing numbers across
builds, confirm both were built with the same profile.

\---

## Using the interactive app

**Camera** — right-click and drag to orbit, scroll to zoom.

**Manual joint control** — keyboard bindings set up in `KeyedJointRotator` and
`LinearAxisMover`, one per joint.

**Orbit sequences** — the OnGUI panel (`DOFSequenceBuilder`, in `SequenceUI.cs`)
has a row per DOF with a value, a duration, a wait-for-previous toggle, and a
smooth toggle. Rows can be dragged to reorder. Sequences can be looped. There's
an **Import Orbit CSV** button that reads an orbit CSV out of `Assets/` and turns
it into a sequence — that's how you get an optimizer result back into the app to
look at.

**Beam visualisation** — `ConeSmear` accumulates semi-transparent ghost meshes of
the beam cone as the arm moves, so you can see the swept volume build up. View
modes are Normal / Trace Only / No Trace.

**Collision highlight** — `ArmCollision` turns parts red on trigger contact. Be
aware this is **completely independent of the scorer**. It uses Unity's trigger
system on the interactive arm; the scorer uses `ComputePenetration` on a separate
solid-collider duplicate. They watch different objects and can legitimately
disagree. Red in the viewport does not mean the scorer saw a collision, and vice
versa.

**Playing back a scored orbit** — `OrbitPlayback` loads an orbit CSV and plays it
using the *exact same pose math and penetration checks as the scorer*. If you
want to see what the scorer actually simulated, use this rather than the sequence
builder, because the two take different paths through the joint hierarchy. It has
a scrub slider, a live penetration readout, and a "Go to worst" button that jumps
to the deepest contact in the orbit.

\---

## Using the headless build

It does nothing without arguments. With them:

```
CBCT.exe -batchmode -nographics ^
  -logFile  path\\to\\unity.log ^
  -in       path\\to\\candidates.json ^
  -out      path\\to\\results.json ^
  -trajdir  path\\to\\trajectories
```

`-trajdir` is optional and only exists in later builds. When present, it writes
one CSV per candidate containing the focal spot and detector pose at every
projection — that's the input for the Tuy analysis.

Input:

```json
{"candidates": \[{"p": \[45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0, ...]}]}
```

Output, one entry per candidate, in the same order:

```json
{"results": \[{"fitness": 7080, "sampling": 7080, "collisionPenalty": 0,
              "sampledVoxels": 708, "roiVoxels": 14328,
              "maxDepthCm": 0.0, "touchingProjections": 0}]}
```

### Figuring out which build you're looking at

Inspector values are **baked into the build at compile time**. Two builds from
different days will score identically-defined orbits differently, and there's no
way to tell them apart from the outside. The startup log is the record — run any
build once and read the first few `\[OrbitScorer]` lines:

```
\[OrbitScorer] grid 31^3 at 1.0cm, ROI = 14328 voxels within 15cm of head
\[OrbitScorer] 5 arm colliders, 4 obstacle colliders
\[OrbitScorer]   arm: .../Virtual Module/Cube — BoxCollider
\[OrbitScorer]   obstacle: wide body table clamp/Cube — BoxCollider
\[OrbitScorer] CranioCaudal motion profile: Linear (lateral is always linear)
```

That tells you the grid resolution, the ROI voxel count, exactly which colliders
are being tested, and the motion profile. If two builds disagree on any of these,
their scores are not comparable and you shouldn't try to compare them.

A quick way to compare several builds at once is to run one reference orbit
through each and diff those lines — `manual\_score.py` supports a list of builds
and prints this block per build.

Also worth knowing: a MeshCollider must have **Convex** ticked or
`ComputePenetration` silently returns false for it, and that collider will never
register a collision. The startup log flags non-convex mesh colliders.

\---

## The Python side

**`orbit\_optimizer.py`** — CMA-ES. Each generation it writes 20 candidates to
`batch/candidates.json`, launches the headless exe, reads `batch/results.json`,
and feeds the fitnesses back. No sockets, no ML-Agents, nothing to desynchronise:
row *i* in equals row *i* out, and both files sit on disk where you can open
them. Key knobs at the top of the file are `WARM\_START` (the 48-value starting
point), `SIGMA\_PER\_DOF` (how far it explores in each DOF — set one near zero to
effectively freeze that axis), and the CMA-ES options. On completion it writes
`orbit\_results.json` and `orbit\_results\_orbit.csv`.

Note that CMA-ES never evaluates its own starting point — `ask()` returns
perturbed samples around the mean, not the mean itself. If the warm start is
already good, the optimizer can and will return something worse than what you
gave it unless you explicitly score the seed.

**`manual\_score.py`** — scores orbits without any optimization. Orbits come from
a dict you type in at the top of the file and/or from every CSV in a folder you
point it at. Everything scores in a single launch. This is the tool for
troubleshooting: when a number looks wrong, score a known orbit (pure circular,
zero motion) and see whether the baseline matches what it used to be.

**`orbit\_analysis.py`** — scoring plus Tuy completeness in one pass, producing a
comparison table across a folder of orbits.

### Tuy completeness

Tuy's condition says exact reconstruction at a point requires that every plane
through that point be crossed by the source trajectory. The analysis samples
\~2000 plane orientations per evaluation point (Fibonacci sphere), checks which
are crossed, and reports two different numbers:

* **`tuy\_mean`** — the average completeness fraction over all ROI points. Partial
credit. "On average a point sees 95% of the planes it needs."
* **`tuy\_effective\_fov`** — each point is first thresholded at 0.99, then this
reports the percentage that pass. Pass/fail. "62% of the ROI is exactly
reconstructable."

These can diverge sharply. An orbit sitting at 0.97 everywhere has a high mean
and an effective FOV of zero. `tuy\_effective\_fov` is the one that corresponds to
"effective field of view" in the sense of the volume that actually receives
complete angular coverage.

A planar trajectory can only satisfy Tuy within its own plane, so a circular
orbit — and any orbit built from coplanar circles — has a hard ceiling no amount
of extra in-plane sampling will raise. Out-of-plane motion (CranioCaudal tilt) is
what moves this number.

The trajectory CSVs are in Unity world coordinates, which are **left-handed with
Y up**. The analysis remaps them to a right-handed Z-up CT frame via
`UNITY\_TO\_CT\_AXES` and scales to mm via `UNITY\_UNITS\_TO\_MM`. If you ever get a
plausible-looking but wrong completeness number, check those two constants first.

\---

## Typical workflow

1. Set up the orbit you want to try, either in the `ORBITS` dict in
`manual\_score.py` or as a CSV in an orbits folder.
2. Score it against the headless build and compare against the pure circular and
zero-motion baselines, which are always worth including for scale.
3. If it looks promising, use it as `WARM\_START` in `orbit\_optimizer.py` and let
CMA-ES refine it. Keep sigma small on the DOF you want to preserve.
4. Copy the resulting `orbit\_results\_orbit.csv` into `Assets/` and play it back
in the interactive app with `OrbitPlayback` to see what it actually does.
5. Run `orbit\_analysis.py` for the Tuy numbers once you have a set worth
comparing.

\---

## Known gaps

Collected in one place so they don't have to be rediscovered:

* The floor has no collider and no `Obstacle` tag, so floor collisions are not
scored at all.
* Only colliders under `Arm Roots` are tested. If that array points only at the
detector module, the column and gantry base are invisible to collision
scoring even though they move.
* Trigger colliders on the arm side are skipped by the collector.
* Contacts occurring between sampled projections are missed.
* `ArmCollision`'s red highlight and the scorer's collision number measure
different things on different objects and will not always agree.
* Scores from builds with different Inspector values, or different CranioCaudal
motion profiles, are not comparable.

