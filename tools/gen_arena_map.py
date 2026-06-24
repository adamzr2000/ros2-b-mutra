#!/usr/bin/env python3
"""
gen_arena_map.py — generate the occupancy grid for swarm_arena.world.

The arena is a 16 m x 16 m walled empty room: interior free space spans
x,y in [-8, +8], with boundary walls at +/-8 m. This script emits a Nav2
map_server-compatible PGM + YAML pair describing exactly that, so the map is
reproducible and auditable (no offline SLAM scan needed for a known rectangle).

Encoding (matches map.yaml: negate=0, occupied_thresh=0.65, free_thresh=0.196):
  pixel 254 -> p~0.00 -> FREE      (interior)
  pixel   0 -> p=1.00 -> OCCUPIED  (walls)
  pixel 205 -> p~0.20 -> UNKNOWN   (outside the room)

Geometry constants mirror swarm_arena.world. Run from the repo root:
  python3 tools/gen_arena_map.py
Outputs into dockerfiles/.../turtlebot3_navigation2/map/swarm_arena.{pgm,yaml}.
"""
import os

# ── Map / arena geometry (keep in sync with swarm_arena.world) ──────────────
RES = 0.05                 # m per pixel
ORIGIN = (-10.0, -10.0)    # world coord of the image's bottom-left corner
SIZE_PX = 400              # 400 * 0.05 = 20 m span (comfortably covers +/-8 m)
HALF = 8.0                 # interior half-extent: free space is [-8, +8]
WALL_HALF = 0.12           # half-thickness drawn for walls (~0.24 m -> solid)

FREE, OCC, UNK = 254, 0, 205

OUT_DIR = os.path.join(
    "dockerfiles", "turtlebot3", "src", "turtlebot3_navigation2", "map")


def world_xy(col, row_from_top):
    """Map pixel (col, row-from-top) -> world (x, y). Image is read bottom-up."""
    x = ORIGIN[0] + (col + 0.5) * RES
    row_from_bottom = (SIZE_PX - 1 - row_from_top)
    y = ORIGIN[1] + (row_from_bottom + 0.5) * RES
    return x, y


def classify(x, y):
    on_vert = (abs(abs(x) - HALF) <= WALL_HALF) and (abs(y) <= HALF + WALL_HALF)
    on_horz = (abs(abs(y) - HALF) <= WALL_HALF) and (abs(x) <= HALF + WALL_HALF)
    if on_vert or on_horz:
        return OCC
    if -HALF < x < HALF and -HALF < y < HALF:
        return FREE
    return UNK


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    pgm_path = os.path.join(OUT_DIR, "swarm_arena.pgm")
    yaml_path = os.path.join(OUT_DIR, "swarm_arena.yaml")

    pixels = bytearray(SIZE_PX * SIZE_PX)
    free = occ = 0
    for row in range(SIZE_PX):
        for col in range(SIZE_PX):
            x, y = world_xy(col, row)
            v = classify(x, y)
            pixels[row * SIZE_PX + col] = v
            if v == FREE:
                free += 1
            elif v == OCC:
                occ += 1

    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# swarm_arena 16x16 walled empty room (generated)\n")
        f.write(f"{SIZE_PX} {SIZE_PX}\n255\n".encode())
        f.write(bytes(pixels))

    with open(yaml_path, "w") as f:
        f.write(
            "image: swarm_arena.pgm\n"
            f"resolution: {RES:.6f}\n"
            f"origin: [{ORIGIN[0]:.6f}, {ORIGIN[1]:.6f}, 0.000000]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n")

    print(f"wrote {pgm_path} ({SIZE_PX}x{SIZE_PX}, free={free}px, occ={occ}px)")
    print(f"wrote {yaml_path}")


if __name__ == "__main__":
    main()
