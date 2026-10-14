# Third-party sources

The integration reference was read from `Spaaaace-yyj/RM_Dron_mujoco_sim`, commit
`7536db08450ad63586081d622386fdb3c4bd186f` (main, accessed 2026-10-13).

`src/rm_auto_aim_sim/third_party/simulate` contains the MuJoCo 3.3.7 official
simulate viewer sources distributed in that reference. Copyright and Apache-2.0
headers are preserved; the license is in `third_party/MUJOCO_LICENSE`.

Application changes remove the drone-specific hooks, add the Auto Aim menu and
input hooks, route paused single-step through Engine, prevent live model replacement,
and expose the GLFW window for application key polling. New simulation, camera,
projectile and ROS2 modules are separate from the viewer.

`third_party/lodepng` is the reference's vendored LodePNG encoder/decoder. Its zlib
license and attribution are embedded in lodepng.cpp/lodepng.h; those files are unmodified.

MuJoCo, GLFW, yaml-cpp and ROS2 are linked dependencies installed by the user;
their binaries are not included in this source archive.
