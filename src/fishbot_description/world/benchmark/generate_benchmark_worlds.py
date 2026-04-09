#!/usr/bin/env python3
"""Generate Gazebo world files from benchmark occupancy maps.

This script converts occupied map pixels into merged axis-aligned boxes and writes
one `.world` file per map for map-world paired experiments.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import yaml


Rectangle = Tuple[int, int, int, int]  # x, y, w, h in image pixels (top-left origin)


def read_pgm(path: Path) -> Tuple[int, int, bytes]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Unsupported PGM format in {path}, expected P5")

        line = f.readline().strip()
        while line.startswith(b"#"):
            line = f.readline().strip()

        width, height = map(int, line.split())
        _ = int(f.readline().strip())  # max value (typically 255)
        data = f.read()

    expected = width * height
    if len(data) != expected:
        raise ValueError(f"PGM size mismatch in {path}: got {len(data)}, expected {expected}")
    return width, height, data


def occupancy_mask(
    pixels: bytes,
    occupied_thresh: float,
    negate: int,
) -> List[bool]:
    mask: List[bool] = []
    for p in pixels:
        if negate:
            occ_prob = p / 255.0
        else:
            occ_prob = (255 - p) / 255.0
        mask.append(occ_prob >= occupied_thresh)
    return mask


def merge_rectangles(mask: Sequence[bool], width: int, height: int) -> List[Rectangle]:
    visited = [False] * (width * height)
    rects: List[Rectangle] = []

    for y in range(height):
        row_base = y * width
        x = 0
        while x < width:
            idx = row_base + x
            if (not mask[idx]) or visited[idx]:
                x += 1
                continue

            rect_w = 0
            while x + rect_w < width:
                i = row_base + x + rect_w
                if (not mask[i]) or visited[i]:
                    break
                rect_w += 1

            rect_h = 1
            while y + rect_h < height:
                next_row = (y + rect_h) * width
                ok = True
                for xx in range(x, x + rect_w):
                    i = next_row + xx
                    if (not mask[i]) or visited[i]:
                        ok = False
                        break
                if not ok:
                    break
                rect_h += 1

            for yy in range(y, y + rect_h):
                row = yy * width
                for xx in range(x, x + rect_w):
                    visited[row + xx] = True

            rects.append((x, y, rect_w, rect_h))
            x += rect_w

    return rects


def rect_to_world_pose(
    rect: Rectangle,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    obstacle_height: float,
) -> Tuple[float, float, float, float, float, float]:
    x, y, w, h = rect

    size_x = w * resolution
    size_y = h * resolution
    size_z = obstacle_height

    cx = origin_x + (x + w / 2.0) * resolution
    cy = origin_y + (height - y - h / 2.0) * resolution
    cz = obstacle_height / 2.0
    return cx, cy, cz, size_x, size_y, size_z


def build_world_text(world_name: str, rect_poses: Iterable[Tuple[float, float, float, float, float, float]]) -> str:
    lines: List[str] = []
    lines.append("<sdf version='1.7'>")
    lines.append(f"  <world name='{world_name}'>")
    lines.append("    <include><uri>model://sun</uri></include>")
    lines.append("    <include><uri>model://ground_plane</uri></include>")
    lines.append("    <model name='benchmark_obstacles'>")
    lines.append("      <static>true</static>")
    lines.append("      <link name='obstacles_link'>")

    for i, (cx, cy, cz, sx, sy, sz) in enumerate(rect_poses, start=1):
        name = f"obs_{i:04d}"
        pose = f"{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0"
        size = f"{sx:.3f} {sy:.3f} {sz:.3f}"

        lines.append(f"        <collision name='{name}_collision'>")
        lines.append(f"          <pose>{pose}</pose>")
        lines.append("          <geometry><box><size>" + size + "</size></box></geometry>")
        lines.append("          <surface><contact><collide_without_contact>false</collide_without_contact></contact></surface>")
        lines.append("        </collision>")

        lines.append(f"        <visual name='{name}_visual'>")
        lines.append(f"          <pose>{pose}</pose>")
        lines.append("          <geometry><box><size>" + size + "</size></box></geometry>")
        lines.append("          <material>")
        lines.append("            <ambient>0.25 0.25 0.25 1</ambient>")
        lines.append("            <diffuse>0.45 0.45 0.45 1</diffuse>")
        lines.append("            <specular>0.05 0.05 0.05 1</specular>")
        lines.append("          </material>")
        lines.append("        </visual>")

    lines.append("      </link>")
    lines.append("    </model>")
    lines.append("  </world>")
    lines.append("</sdf>")
    return "\n".join(lines) + "\n"


def generate_world(map_yaml: Path, out_dir: Path, obstacle_height: float) -> Tuple[Path, int]:
    cfg = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = cfg["image"]
    resolution = float(cfg["resolution"])
    origin_x, origin_y, _ = cfg["origin"]
    negate = int(cfg.get("negate", 0))
    occupied_thresh = float(cfg.get("occupied_thresh", 0.65))

    image_path = (map_yaml.parent / image).resolve()
    width, height, pixels = read_pgm(image_path)

    mask = occupancy_mask(pixels, occupied_thresh=occupied_thresh, negate=negate)
    rects = merge_rectangles(mask, width, height)

    rect_poses = [
        rect_to_world_pose(
            r,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            obstacle_height=obstacle_height,
        )
        for r in rects
    ]

    world_text = build_world_text(map_yaml.stem, rect_poses)
    out_path = out_dir / f"{map_yaml.stem}.world"
    out_path.write_text(world_text, encoding="utf-8")
    return out_path, len(rects)


def default_map_dir(script_path: Path) -> Path:
    src_root = script_path.parents[3]
    return src_root / "fishbot_navigation2" / "maps" / "benchmark"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate benchmark Gazebo world files from map YAMLs")
    parser.add_argument(
        "--map-dir",
        default="",
        help="Directory containing map YAML files (default: fishbot_navigation2/maps/benchmark)",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory for .world files (default: script directory)",
    )
    parser.add_argument(
        "--obstacle-height",
        type=float,
        default=1.0,
        help="Height of generated wall obstacles in meters (default: 1.0)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    script_path = Path(__file__).resolve()

    map_dir = Path(args.map_dir).resolve() if args.map_dir else default_map_dir(script_path)
    out_dir = Path(args.out_dir).resolve() if args.out_dir else script_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    map_yamls = sorted(p for p in map_dir.glob("*.yaml") if p.is_file())
    if not map_yamls:
        raise FileNotFoundError(f"No map YAML found in {map_dir}")

    for map_yaml in map_yamls:
        out_path, rect_count = generate_world(
            map_yaml=map_yaml,
            out_dir=out_dir,
            obstacle_height=args.obstacle_height,
        )
        print(f"Generated {out_path} with {rect_count} obstacle boxes")


if __name__ == "__main__":
    main()
