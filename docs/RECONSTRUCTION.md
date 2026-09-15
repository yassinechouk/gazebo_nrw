# What was wrong with the export, and what this package does about it

The source package (`../../urdf`, `../../meshes`) is a SolidWorks
`sw_urdf_exporter` output. It is a valid *description* of the kinematics but it
cannot be simulated as shipped. This document records every difference between
the export and the model in `src/nrw_cell_sim/urdf/`, so nothing here is a
silent change.

---

## 1. Every link's STL contained the entire assembly

This is the big one.

| source mesh | triangles | size |
|---|---|---|
| `base_link.STL` | 1 219 656 | 61 MB |
| `convoyeur.STL` | 1 265 292 | 63 MB |
| `guidage.STL`   | 1 265 780 | 63 MB |
| `axe x.STL`     | 1 219 668 | 61 MB |
| `axe y.STL`     | 1 220 024 | 61 MB |

All five have the same bounding-box extents up to an axis permutation
(7.709 x 6.189 x 4.642 m), and mapping `axe x.STL` through the `prismatic x`
joint transform onto `base_link.STL` reproduces its bounding box to 1e-4 m with
89 % of vertices identical. They are the same assembly, written five times in
five different link frames.

Loading the export into Gazebo therefore spawns **five overlapping copies of the
whole cell**, each with a 1.2 M-triangle *collision* mesh. That is roughly
310 MB of geometry and a physics scene that will not step.

### What was done

`tools/build_meshes.py` performs a shared-vertex connected-component
segmentation of all five meshes, transforms every component into the
`base_link` frame, de-duplicates, and recovers **271 distinct physical parts**:

| group | parts | note |
|---|---|---|
| chain links | 220 | 1 198 340 of the 1 265 768 triangles — 95 % of the budget |
| rollers | 24 | Ø0.12 x 1.0 m, pitch 0.25 m |
| chain drums | 4 | Ø1.06 m sprockets, two per tower |
| tower frames + guide rails | 6 | 6.0 m and 6.1 m verticals |
| drive assembly | 10 | motor / gearbox at the top of the left tower |
| floor slab + table legs | 1 | 50 mm pad, 7.709 x 4.642 m |
| **platform + pushings** | 1 | -> `carriage` |
| **pusher blades** | 2 | -> `pusher` |
| **comptoir + tray** | 2 | -> `shelf` |

Each part is decimated (vertex clustering, binary-searched grid) to a budget
proportional to its size, and written into per-material STLs.

**Result: 1 265 768 -> 40 123 triangles, 310 MB -> 2.0 MB.**

### How the moving parts were identified

The meshes are unreliable, but the SolidWorks **mass properties** in
`../../urdf/Assemforchouk.csv` are per-part and correct. Inverting the inertia
tensor for equivalent box dimensions and transforming each link's centre of mass
into the base frame gives a target that matches exactly one component:

| link | COM in base frame (from CSV) | matching component | miss |
|---|---|---|---|
| `convoyeur` | (-2.163, -5.721, -1.464) | 45 268 tri, ext 0.50 x 0.90 x 0.20 | 27 mm |
| `guidage` | (-2.163, -6.065, -1.402) | two 244-tri blades, midpoint identical | ~0 |
| `axe y` | (-1.804, -3.793, -1.521) | 244 tri, ext 4.10 x 0.80 x 0.21 | see below |

The `axe y` (shelf) match is on dimensions and on Y/Z; the 0.32 m offset in X is
the four `box-*` components that the CSV lists in the same link pulling its
centre of mass sideways.

Note that `base_link.STL` is the only source mesh that does **not** contain the
moving parts — the exporter removed them from the static link but left the
static structure in every moving link.

---

## 2. Names that are illegal downstream

`axe x`, `axe y` and `prismatic x` contain spaces. SDFormat, `ros2_control` and
the controller YAML all choke on those.

| export | this package | what it is |
|---|---|---|
| `base_link` | `base_link` | cell structure: floor, both towers, roller table |
| `convoyeur` / joint `slider` | `carriage` / `carriage_slide` | platform running along the roller table |
| `guidage` / joint `pusher` | `pusher` / `pusher_extend` | two transfer blades on the carriage |
| `axe x` / joint `prismatic x` | `lift_carriage` / `lift` | chain-lift carriage (vertical axis) |
| `axe y` / joint `shelf` | `shelf` / `shelf_extend` | comptoir + tray carried by the lift |

Joint origins, `rpy` and axis vectors are copied **verbatim** from the export.
Forward kinematics through them gives:

| joint | axis in world | travel direction |
|---|---|---|
| `carriage_slide` | +X | along the roller table |
| `pusher_extend` | +X | blade stroke |
| `lift` | **+Z** | the vertical chain lift |
| `shelf_extend` | +Y | shelf in/out toward the conveyor |

which is consistent with the recovered geometry (rollers spaced along X, chain
loops running vertically between drums at z = -2.96 and z = +2.04).

---

## 3. Joint limits

The export gives every joint `lower="-50" upper="50" effort="1" velocity="1"` —
50 m of travel on a 7 m machine, and 1 N of effort, which cannot hold up a
105 kg shelf. Replaced with limits derived from the recovered geometry:

| joint | limits (m) | derivation |
|---|---|---|
| `carriage_slide` | -1.90 … 3.40 | platform (0.5 m) kept on the 5.88 m roller table |
| `pusher_extend` | 0.00 … 0.35 | blade stroke |
| `lift` | -0.80 … 2.80 | shelf kept between the drums at z = -2.96 and +2.04 |
| `shelf_extend` | -1.15 … 0.15 | shelf edge reaches the conveyor at y = -5.72 |

Effort and velocity are sized from the moving masses (e.g. `lift` carries
105 + 25 kg -> 1.3 kN, set to 8 kN with margin). Damping and friction were added
to every joint; the export had none.

---

## 4. Inertia of `axe x`

The export gives the dummy carriage `mass="1E-06"` with `ixx=1.6667E-13`. A
near-massless link between two joints makes the constraint solver ill-conditioned.
Replaced with a plausible welded carriage: 25 kg, box inertia for 1.0 x 0.4 x 0.3 m.

Every other link keeps its SolidWorks mass and inertia tensor unchanged. One
caveat: `base_link`'s exported centre of mass is at z = +4.65, which is outside
its own geometry (the structure spans z = -3.59 … +2.60). The geometric centroid
is used instead. `base_link` is fixed to the world, so this is inert either way.

---

## 5. Things the export did not have at all

* **No world attachment.** The export has a free-floating `base_link`, so the
  cell falls under gravity. A `world` link and fixed joint were added, with a
  +3.592 m Z offset so the floor slab rests on the ground plane.
* **No collision primitives.** Collision reused the 1.2 M-triangle visual mesh.
  Replaced with 10 boxes (floor pad, roller bed, two tower frames, platform, two
  blades, shelf, tray).
* **No `<gazebo>` tags, no transmissions, no controllers.** Added
  `gz_ros2_control` with a `JointTrajectoryController` over all four axes.
* `launch/gazebo.launch.py` in the export targets ROS 1 (`gazebo_ros`
  `empty_world.launch`, `rostopic pub /calibrated`). Rewritten for ROS 2 Jazzy
  and Gazebo Harmonic.
* The package name `Assemforchouk` is invalid in ROS 2, which requires
  `^[a-z][a-z0-9_]*$`. Renamed `nrw_cell_sim`.

---

## 6. Transfer mechanism — changed from the CAD on request

As exported, the cell had no working transfer: the shelf stopped 130 mm short of
a load on the carriage, the two decks overlapped vertically by 184 mm, and the
pusher's axis ran **along** the conveyor (+X) rather than across it.

| what | as exported | now |
|---|---|---|
| `pusher_extend` axis | +X (along the conveyor) | **+Y** (across it, toward the shelf) |
| `pusher_extend` stroke | 0.35 m | **1.05 m** |
| `lift` range | -0.80 … 2.80 | **-2.05 … 4.10** (full chain wrap) |
| `shelf_extend` sense | positive moved *away* from the conveyor | **positive extends toward it** |
| `shelf_extend` limits | -1.15 … 0.15 | **-0.15 … 1.15** |
| compartment tray | 1.28 m of the 4.10 m comptoir | **repeated x3, 3.84 m** |
| carriage collision | one solid 0.5 x 0.9 x 0.2 block | **open channel**: deck floor + two side walls |
| shelf collision | two slabs | **deck floor + 2 end walls + 11 dividers** |

The pusher axis in the `guidage` link frame is now `-1 0 0` and the shelf axis in
`axe y` is `0 0 1`; both were verified by forward kinematics.

### Slot geometry

The dividers were measured off the mesh rather than estimated: **11 dividers,
30 mm wide on a 320 mm pitch**, leaving **twelve 290 mm slots** centred at

```
-3.193 -2.873 -2.553 -2.233 -1.913 -1.593 -1.273 -0.953 -0.633 -0.313 0.007 0.324
```

An earlier pass modelled every divider as 98 mm wide. That was wrong — the real
ones alternate 30 mm — and the oversized boxes ate 24 mm into each slot, so a
200 mm package caught on a divider edge by 2.5 mm and the push jammed dead at the
seam. `SLOT_SLIDE` in `cycle_demo.py` and `SHELF_SLOTS` in `spawn_payload.py` are
both derived from the measured values and must stay in step with the collision
boxes in the xacro.

### Why the stroke is 1.05 m

At `shelf_extend = 1.055` the shelf deck's near edge sits at y = -5.273, exactly
the carriage deck's front edge. The blades start at y = -6.065, so:

| stroke | package ends at | result |
|---|---|---|
| 0.80 m | centre -5.115, 8 mm onto the shelf | balanced on the seam; the shelf slides out from under it on retract and it falls |
| **1.05 m** | centre -4.865 | lands mid-deck, seats in the slot at z = 2.099 dead level |

Measured with the 1.05 m stroke: package seats at z = 2.099 with rpy (0, 0, 0);
shelf retracts and it rides +1.055 m with it; lift +2.400 m and it rises to
z = 4.499, i.e. exactly +2.400.

The blades finish 258 mm over the shelf, passing 16 mm above the divider tops.

### The lift is a loop, not a hoist

The export models the lift as a single prismatic axis, but the machine is a
paternoster: the carrier goes up one side of the chain, over the top sprocket,
down the other side and under the bottom one. `lift` and `shelf_extend` are
really the two coordinates of that loop, and the CAD confirms it — the shelf's
rest position (y = -3.818) sits on the far-side chain run (y = -3.780), and the
transfer position (y = -4.873) sits on the conveyor-side run (y = -4.860).

To let the carrier reach the top and bottom of the chain wrap, `lift` was widened
from -0.80 … 2.80 to **-2.05 … 4.10**. Loop geometry measured off the meshes:

| feature | value |
|---|---|
| conveyor-side run | `shelf_extend` = 1.042 |
| far-side run | `shelf_extend` = -0.038 |
| top sprocket centre | `lift` = 3.495 |
| bottom sprocket centre | `lift` = -1.499 |
| wrap radius | 0.527 in `lift`, 0.540 in `shelf_extend` |

The bottom wrap is flattened to 0.460 so the deck clears the floor slab by 27 mm
instead of cutting 47 mm into it. Verified: a full circulation stays inside both
joint limits, and seven packages stayed seated through it.

### Loading and timing

`spawn_payload.py` builds the whole batch first and fires every `create` call
from a thread pool, so a pre-loaded shelf appears complete in one instant — the
five packages land within about 1 ms of each other instead of one every half
second.

It also reads `/joint_states` before placing anything, so packages land where the
machine actually is rather than where it was at t=0. And `cycle_demo.py` holds
still for `start_delay` (7 s) before its first move. Without both of those the
carrier would start sliding toward the receive pose while the packages were still
spawning at the home pose, and the deck would slip out from under them: measured
spread across the deck was 0.330 m before, 0.022 m after.

### One part deleted

The CAD has a loose 0.32 x 0.70 x 0.10 plate lying on the roller table at
(-2.172, -5.710, -1.417). It belongs to no link, so it sat there statically in
the middle of the carriage's path. It is now listed in `EXCLUDE` in
`tools/build_meshes.py` and dropped at mesh-build time.

### What is still not right

The decks are not level: the shelf deck floor is at z = -1.553 and the carriage
deck floor at z = -1.470, an 83 mm step down. The package drops that step rather
than sliding across flat. Levelling them means moving geometry that does come
from the CAD, so it was left alone.

---

## 7. What is synthesised rather than recovered

Stated plainly, so it is not mistaken for CAD data:

* Triangle counts are reduced; the decimated meshes are visually faithful but are
  **not** dimensionally exact at sub-centimetre scale. Do not measure off them —
  use the original STLs for that.
* All collision geometry is box primitives fitted to part bounding boxes.
* `lift_carriage` mass/inertia (section 4).
* Joint limits, effort, velocity, damping, friction (section 3).
* Materials/colours are chosen for legibility; the export only carried two
  colours (light blue-grey for everything, yellow for `guidage`).
* The chain links are static geometry. The chains do **not** animate — the lift
  is modelled as a prismatic axis, which is the correct abstraction for control
  but means you will not see the chain move.
