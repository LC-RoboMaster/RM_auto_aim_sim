#!/usr/bin/env python3
"""将 ling37luo/rmuc2026-mujoco 的已验证场地包导出给 C++ 模拟器。

不下载或重新分发官方 CAD。XML 保留原场地包的绝对资产路径，浮点高度
单独导出，避免绕过上游加载器后退回 PNG 量化碰撞。需要 NumPy、PyYAML
和已安装的 rmuc2026_mujoco；输出只供本机使用，不提交到 Git。
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from rmuc2026_mujoco import FieldAsset
from rmuc2026_mujoco.mjcf import HFIELD_NAME, load_model


def export_field(asset: FieldAsset, output: Path, profile: str) -> Path:
    """Compile through upstream, then preserve exact normalized hfield samples."""
    entrypoint = asset.entrypoint_for(profile)
    model, _ = load_model(asset, profile=profile)
    tree = ET.parse(entrypoint)
    compiler = tree.getroot().find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    texturedir = compiler.get("texturedir", "") if compiler is not None else ""
    # Resolved paths remain local to the user, independent of ROS install location.
    for element in tree.getroot().iter():
        filename = element.get("file")
        if filename:
            base = meshdir if element.tag == "mesh" else texturedir if element.tag == "texture" else ""
            element.set("file", str((entrypoint.parent / base / filename).resolve()))
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)
    hfield = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, HFIELD_NAME)
    if hfield < 0:
        raise ValueError("Upstream field has no expected collision hfield")
    rows, cols = int(model.hfield_nrow[hfield]), int(model.hfield_ncol[hfield])
    adr = int(model.hfield_adr[hfield])
    samples = np.asarray(model.hfield_data[adr:adr + rows*cols], dtype="<f4")
    output.mkdir(parents=True, exist_ok=False)
    xml = output / "field.xml"
    ET.indent(tree)
    tree.write(xml, encoding="utf-8", xml_declaration=True)
    sidecar = xml.with_suffix(".xml.heights")
    sidecar.write_bytes(struct.pack("<ii", rows, cols) + samples.tobytes())
    provenance = {
        "source_repository": "https://github.com/ling37luo/rmuc2026-mujoco",
        "pack": str(asset.root), "manifest_sha256": asset.manifest_sha256,
        "profile": profile, "heightfield_shape": [rows, cols],
        "xml_sha256": hashlib.sha256(xml.read_bytes()).hexdigest(),
        "heights_sha256": hashlib.sha256(sidecar.read_bytes()).hexdigest(),
        "physical_scope": "Upstream heightfield proxy; not a multilevel solid field",
    }
    (output / "source.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return xml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pack", type=Path, help="rmuc2026-field setup 生成的目录")
    parser.add_argument("output", type=Path, help="新输出目录，例如 assets/rmuc")
    parser.add_argument("--profile", choices=["full", "collision_only", "interactive_lite"], default="full")
    parser.add_argument("--config", type=Path, help="源 sim.yaml；默认使用工程内的配置")
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    if not (package / "scenes/robot.xml").is_file():
        from ament_index_python.packages import get_package_share_directory
        package = Path(get_package_share_directory("rm_auto_aim_sim"))
    template = (args.config or package / "config/sim.yaml").resolve()
    settings = yaml.safe_load(template.read_text())
    config = settings["rm_auto_aim_sim"]["ros__parameters"]
    asset = FieldAsset.open(args.pack, verify=True)
    xml = export_field(asset, args.output.resolve(), args.profile)
    config["scene_path"] = str(package / "scenes/robot.xml")
    config["field_xml"] = str(xml)
    # Field coordinates remain exactly those of upstream. Edit target/base settings
    # in the generated config or viewer; no inferred official spawn/target placement.
    output_config = args.output.resolve() / "sim_rmuc.yaml"
    output_config.write_text(yaml.safe_dump(settings, sort_keys=False))
    print(f"RMUC 配置: {output_config}")
    print(f"ros2 launch rm_auto_aim_sim sim.launch.py config:={output_config}")


if __name__ == "__main__":
    main()
