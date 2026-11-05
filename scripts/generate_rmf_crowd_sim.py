#!/usr/bin/env python3
"""
Generate complete RMF crowd simulation files:
- maps/airport_cropped.building.yaml (clean human_lanes, lanes & vertices)
- L1_navmesh.nav (NavMesh for ORCA)
- behavior_file.xml (Menge FSM transitions)
- scene_file.xml (Menge agent profiles & goals)
- Inserts crowd_simulation plugin into Gazebo world
"""

import copy
import os
import sys
import yaml

SCALE = 0.0821218733149518

X_MIN_WORLD = 117.0
X_MAX_WORLD = 158.0
Y_MIN_WORLD = -62.5
Y_MAX_WORLD = -25.0

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORIG_YAML = "/Users/hojun/codings/simulating/sicnavproject/cutting_world/airport_terminal.building.yaml"

if not os.path.exists(ORIG_YAML):
    print(f"Original building yaml not found at: {ORIG_YAML}")
    sys.exit(1)

with open(ORIG_YAML, "r") as f:
    orig = yaml.safe_load(f)

l1 = orig["levels"]["L1"]
verts = l1["vertices"]
walls = l1.get("walls", [])
lanes = l1.get("lanes", [])
human_lanes = l1.get("human_lanes", [])

needed_verts = set()

# Keep walls inside boundary
kept_walls = []
for w in walls:
    i1, i2 = w[0], w[1]
    v1, v2 = verts[i1], verts[i2]
    w1x, w1y = v1[0] * SCALE, -v1[1] * SCALE
    w2x, w2y = v2[0] * SCALE, -v2[1] * SCALE
    in1 = (X_MIN_WORLD <= w1x <= X_MAX_WORLD and Y_MIN_WORLD <= w1y <= Y_MAX_WORLD)
    in2 = (X_MIN_WORLD <= w2x <= X_MAX_WORLD and Y_MIN_WORLD <= w2y <= Y_MAX_WORLD)
    if in1 or in2:
        kept_walls.append(w)
        needed_verts.add(i1)
        needed_verts.add(i2)

# Keep robot lanes inside boundary
kept_lanes = []
for lane in lanes:
    i1, i2 = lane[0], lane[1]
    v1, v2 = verts[i1], verts[i2]
    l1x, l1y = v1[0] * SCALE, -v1[1] * SCALE
    l2x, l2y = v2[0] * SCALE, -v2[1] * SCALE
    in1 = (X_MIN_WORLD <= l1x <= X_MAX_WORLD and Y_MIN_WORLD <= l1y <= Y_MAX_WORLD)
    in2 = (X_MIN_WORLD <= l2x <= X_MAX_WORLD and Y_MIN_WORLD <= l2y <= Y_MAX_WORLD)
    if in1 or in2:
        kept_lanes.append(lane)
        needed_verts.add(i1)
        needed_verts.add(i2)

# Keep human lanes (Goal paths)
kept_human_lanes = []
for hlane in human_lanes:
    i1, i2 = hlane[0], hlane[1]
    kept_human_lanes.append(hlane)
    needed_verts.add(i1)
    needed_verts.add(i2)

# Also keep all goal vertices explicitly (idx 1290..1296)
for idx, v in enumerate(verts):
    if len(v) > 4 and isinstance(v[4], dict) and "human_goal_set_name" in v[4]:
        needed_verts.add(idx)

sorted_needed = sorted(list(needed_verts))
vmap = {old: new for new, old in enumerate(sorted_needed)}

new_verts = [copy.deepcopy(verts[i]) for i in sorted_needed]

new_walls = []
for w in kept_walls:
    nw = copy.deepcopy(w)
    nw[0] = vmap[w[0]]
    nw[1] = vmap[w[1]]
    new_walls.append(nw)

new_lanes = []
for lane in kept_lanes:
    nl = copy.deepcopy(lane)
    nl[0] = vmap[lane[0]]
    nl[1] = vmap[lane[1]]
    new_lanes.append(nl)

new_human_lanes = []
for hlane in kept_human_lanes:
    nhl = copy.deepcopy(hlane)
    nhl[0] = vmap[hlane[0]]
    nhl[1] = vmap[hlane[1]]
    new_human_lanes.append(nhl)

# Add perimeter boundary walls
corners_world = [
    (X_MIN_WORLD, Y_MAX_WORLD),
    (X_MAX_WORLD, Y_MAX_WORLD),
    (X_MAX_WORLD, Y_MIN_WORLD),
    (X_MIN_WORLD, Y_MIN_WORLD),
]
corner_indices = []
for wx, wy in corners_world:
    c_idx = len(new_verts)
    new_verts.append([wx / SCALE, -wy / SCALE, 0, f"corner_{len(corner_indices)}"])
    corner_indices.append(c_idx)

boundary_wall_params = {"alpha": [3, 1.0], "texture_name": [1, "default"]}
for i in range(4):
    new_walls.append([corner_indices[i], corner_indices[(i + 1) % 4], boundary_wall_params])

ref_v0 = len(new_verts)
ref_v1 = len(new_verts) + 1
new_verts.append([130.0 / SCALE, 40.0 / SCALE, 0, "meas_0"])
new_verts.append([130.0 / SCALE, 40.0 / SCALE + (3.0 / SCALE), 0, "meas_1"])
new_meas = [[ref_v0, ref_v1, {"distance": [3, 3.0]}]]

cropped = copy.deepcopy(orig)
cropped["name"] = "airport_cropped_world"
cl1 = cropped["levels"]["L1"]
cl1["vertices"] = new_verts
cl1["walls"] = new_walls
cl1["lanes"] = new_lanes
cl1["human_lanes"] = new_human_lanes
cl1["measurements"] = new_meas
cl1["models"] = []
cl1["doors"] = []
cl1["floors"] = []
cl1["drawing"]["filename"] = "airport_cropped_world.png"
cropped["lifts"] = {}

out_yaml = os.path.join(BASE_DIR, "maps", "airport_cropped.building.yaml")
with open(out_yaml, "w") as f:
    yaml.dump(cropped, f, sort_keys=False)

print(f"Generated clean {out_yaml}: {len(new_verts)} verts, {len(new_walls)} walls, {len(new_human_lanes)} human_lanes")
