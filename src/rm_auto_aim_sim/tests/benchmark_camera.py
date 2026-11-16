#!/usr/bin/env python3
"""Measure the real C++ camera throughput ceiling in EGL, without viewer or DDS."""
import argparse
import ctypes
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--library", required=True)
parser.add_argument("--config", required=True)
parser.add_argument("--warmup", type=float, default=1.0)
parser.add_argument("--seconds", type=float, default=5.0)
args = parser.parse_args()
if not 0 <= args.warmup <= 60 or not 0 < args.seconds <= 300:
    parser.error("warmup must be in [0,60], seconds in (0,300]")
context = mujoco.GLContext(32, 32)
context.make_current()
try:
    library = ctypes.CDLL(str(Path(args.library).resolve()))
    library.RunCameraBenchmark.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double]
    library.RunCameraBenchmark.restype = ctypes.c_int
    print("Render ceiling only: static scene; no viewer, DDS or camera scheduling.", flush=True)
    result = library.RunCameraBenchmark(os.fsencode(args.config), args.warmup, args.seconds)
finally:
    context.free()
raise SystemExit(result)
