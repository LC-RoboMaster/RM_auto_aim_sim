#!/usr/bin/env python3
"""Convert expanded URDF to MJCF while retaining named bodies for gimbal/TF binding."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("urdf", type=Path, help="Expanded URDF; resolve package:// mesh paths first")
parser.add_argument("output", type=Path)
args = parser.parse_args()
root = ET.parse(args.urdf).getroot()
if root.tag != "robot":
    raise SystemExit("Expected expanded URDF with a <robot> root")
for mesh in root.iter("mesh"):
    filename = mesh.get("filename", "")
    if filename.startswith("package://"):
        raise SystemExit(f"Resolve package:// to an absolute mesh path first: {filename}")
    if filename and not Path(filename).is_absolute():
        mesh.set("filename", str((args.urdf.resolve().parent / filename).resolve()))
mujoco_element = root.find("mujoco")
if mujoco_element is None:
    mujoco_element = ET.SubElement(root, "mujoco")
compiler = mujoco_element.find("compiler")
if compiler is None:
    compiler = ET.SubElement(mujoco_element, "compiler")
compiler.set("fusestatic", "false")
compiler.set("discardvisual", "false")
spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
spec.compiler.fusestatic = False
spec.compiler.discardvisual = False
spec.compile()  # Fail early for missing meshes, inertias or invalid joints.
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(spec.to_xml(), encoding="utf-8")
print(f"Saved {args.output.resolve()}")
print("Next: adapt the named base/yaw/pitch/camera tree, add gimbal_camera and muzzle; see docs/MODELS.md.")
