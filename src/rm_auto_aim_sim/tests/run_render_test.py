#!/usr/bin/env python3
"""Make EGL current and run the actual C++ camera renderer (no viewer window needed)."""
import argparse
import ctypes
import os
from pathlib import Path
import struct
import zlib

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco

parser = argparse.ArgumentParser()
parser.add_argument("--library", required=True)
parser.add_argument("--config", default=str(Path(__file__).with_name("test_config.yaml")))
parser.add_argument("--output", default="camera_test.ppm")
args = parser.parse_args()
context = mujoco.GLContext(640, 480)
context.make_current()
library = ctypes.CDLL(os.path.abspath(args.library))
library.RunRenderProbe.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
library.RunRenderProbe.restype = ctypes.c_int
result = library.RunRenderProbe(os.fsencode(args.config), os.fsencode(args.output))
if result == 0:
    library.RunLabRenderProbe.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    library.RunLabRenderProbe.restype = ctypes.c_int
    result = library.RunLabRenderProbe(os.fsencode(args.config), os.fsencode(args.output + ".lab.ppm"))
if result == 0:
    library.RunDarkRenderProbe.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    library.RunDarkRenderProbe.restype = ctypes.c_int
    result = library.RunDarkRenderProbe(os.fsencode(args.config), os.fsencode(args.output + ".dark"))
if result == 0:
    library.RunShaderRenderProbe.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    library.RunShaderRenderProbe.restype = ctypes.c_int
    result = library.RunShaderRenderProbe(os.fsencode(args.config), os.fsencode(args.output + ".shader.ppm"))
if result == 0:
    library.RunAsyncQueueProbe.argtypes = [ctypes.c_char_p]
    library.RunAsyncQueueProbe.restype = ctypes.c_int
    result = library.RunAsyncQueueProbe(os.fsencode(args.config))
if result == 0:
    library.RunOutpostRenderProbe.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    library.RunOutpostRenderProbe.restype = ctypes.c_int
    result = library.RunOutpostRenderProbe(os.fsencode(args.config), os.fsencode(args.output + ".outpost.ppm"))
# Preview images for a browser without adding a runtime image dependency.
for source in Path(args.output).parent.glob(Path(args.output).name + "*.ppm"):
    magic, dimensions, maximum, pixels = source.read_bytes().split(b"\n", 3)
    width, height = (int(value) for value in dimensions.split())
    if magic != b"P6" or maximum != b"255" or len(pixels) != width * height * 3:
        raise ValueError(f"Unexpected PPM probe image: {source}")

    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))

    stride = width * 3
    scanlines = b"".join(b"\0" + pixels[row * stride:(row + 1) * stride] for row in range(height))
    png = (b"\x89PNG\r\n\x1a\n" +
           chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(scanlines, 6)) + chunk(b"IEND", b""))
    source.with_suffix(source.suffix + ".png").write_bytes(png)
context.free()
raise SystemExit(result)
