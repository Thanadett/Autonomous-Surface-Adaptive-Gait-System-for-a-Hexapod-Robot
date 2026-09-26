#!/usr/bin/env python3
"""Regenerate hexapod.canonical.urdf from the untouched SolidWorks export.

This reproduces (and extends) 49x_Hexapod_test/scripts/adapt_cad_urdf.py:
  1. Renames CAD link/joint names onto the workspace's stable naming
     convention (front|middle|rear_left|right_{coxa,femur,tibia}).
  2. Points every visual mesh at meshes/visual/<file>.STL (full-resolution,
     unchanged geometry).
  3. Replaces every <collision><mesh> with a <collision><box> sized to that
     link's own mesh bounding box, computed directly from the binary STL
     (no external mesh library). This cuts collision complexity from
     ~173,000 triangles to 19 boxes -- see hexapod_description/meshes/README.md.

Run with --check to verify the generated file is current (used by
validate_urdf.sh); otherwise it (re)writes the output.
"""
from __future__ import annotations

import argparse
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

LEG_POSITIONS = {"Leg1": "front", "Leg2": "middle", "Leg3": "rear"}
SIDES = {"Left": "left", "Right": "right"}
SEGMENTS = {"Coxa": "coxa", "Femur": "femur", "Tibia": "tibia"}

HEADER = (
    "<?xml version=\"1.0\"?>\n"
    "<!--\n"
    "  GENERATED FILE. Do not hand-edit.\n"
    "  Produced by scripts/generate_cad_urdf.py from the untouched SolidWorks\n"
    "  draft-1 export (hexapod_description/urdf/cad/hexapod.urdf). Only names\n"
    "  and collision geometry are changed; every visual mesh, inertial value,\n"
    "  joint origin, axis, and limit is preserved byte for byte from the CAD\n"
    "  export. Re-run scripts/build.sh (or generate_cad_urdf.py directly) to\n"
    "  regenerate after touching the export or the adapter.\n"
    "-->\n"
)


def name_mapping() -> dict[str, str]:
    mapping = {"base_link": "body_link"}
    for cad_side, side in SIDES.items():
        for cad_leg, position in LEG_POSITIONS.items():
            prefix = f"{position}_{side}"
            for cad_segment, segment in SEGMENTS.items():
                mapping[f"{cad_side}_{cad_segment}_{cad_leg}_Link"] = (
                    f"{prefix}_{segment}_link"
                )
                mapping[f"{cad_side}_{cad_segment}_{cad_leg}_Joint"] = (
                    f"{prefix}_{segment}_joint"
                )
    return mapping


def mesh_bbox(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (center, size) of a binary STL's axis-aligned bounding box."""
    with open(path, "rb") as f:
        f.read(80)
        (n_tri,) = struct.unpack("<I", f.read(4))
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for _ in range(n_tri):
            data = f.read(50)
            floats = struct.unpack("<12f", data[:48])
            for i in range(3, 12, 3):
                xs.append(floats[i])
                ys.append(floats[i + 1])
                zs.append(floats[i + 2])
    lo = (min(xs), min(ys), min(zs))
    hi = (max(xs), max(ys), max(zs))
    center = tuple((hi[i] + lo[i]) / 2 for i in range(3))
    size = tuple(hi[i] - lo[i] for i in range(3))
    return center, size


def build(source: Path, meshes_dir: Path) -> ET.Element:
    tree = ET.parse(source)
    root = tree.getroot()
    mapping = name_mapping()

    for link in root.findall("link"):
        original_name = link.attrib["name"]
        if original_name in mapping:
            link.attrib["name"] = mapping[original_name]

        mesh_filename = None
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                mesh_filename = mesh.attrib["filename"].rsplit("/", 1)[-1]
                mesh.attrib["filename"] = (
                    f"package://hexapod_description/meshes/visual/{mesh_filename}"
                )

        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None or mesh_filename is None:
                continue
            stl_path = meshes_dir / mesh_filename
            if not stl_path.is_file():
                raise FileNotFoundError(f"missing mesh for bbox: {stl_path}")
            center, size = mesh_bbox(stl_path)
            geometry.remove(mesh)
            box = ET.SubElement(geometry, "box")
            box.attrib["size"] = " ".join(f"{v:.6f}" for v in size)
            origin = collision.find("origin")
            if origin is None:
                origin = ET.Element("origin")
                collision.insert(0, origin)
            base_xyz = tuple(
                float(v) for v in origin.attrib.get("xyz", "0 0 0").split()
            )
            origin.attrib["xyz"] = " ".join(
                f"{base_xyz[i] + center[i]:.6f}" for i in range(3)
            )
            origin.attrib.setdefault("rpy", "0 0 0")

    for joint in root.findall("joint"):
        for tag in ("parent", "child"):
            element = joint.find(tag)
            if element is not None and element.attrib["link"] in mapping:
                element.attrib["link"] = mapping[element.attrib["link"]]
        if joint.attrib.get("name") in mapping:
            joint.attrib["name"] = mapping[joint.attrib["name"]]

    return root


def serialize(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    return HEADER + body + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("meshes_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    root = build(args.source, args.meshes_dir)
    expected = serialize(root)

    if args.check:
        if not args.output.is_file() or args.output.read_text(encoding="utf-8") != expected:
            raise SystemExit(f"{args.output} is stale; regenerate it from {args.source}")
        print(f"validated generated CAD model: {args.output}")
        return 0

    args.output.write_text(expected, encoding="utf-8")
    print(f"generated {args.output} from {args.source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
