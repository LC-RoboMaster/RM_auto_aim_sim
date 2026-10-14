#!/usr/bin/env python3
"""Make EGL current and run the actual C++ camera renderer (no viewer window needed)."""
import argparse
import ctypes
import os

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco

parser = argparse.ArgumentParser()
parser.add_argument("--library", required=True)
parser.add_argument("--config", required=True)
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
context.free()
raise SystemExit(result)
