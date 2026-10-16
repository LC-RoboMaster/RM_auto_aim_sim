# 参数完整性与 UI 对照（2026-10-16）

- `sim.yaml` 显式列出全部 95 个配置字段，每项都有中文注释。与上一版本比对，
  原有参数值全部保留；补列三个统计话题，新增启动视角与两种图表开关。
- `configuration_contract` 检查声明/YAML 键集合一致、无重复键、逐项有注释；
  验证实际启动的环境、内参与目标参数，以及初始视角校验。
- 删除字段、删除注释、重复键、非法视角四个负例均按预期失败。
- Release 独立版与 ROS2 Humble 版编译通过，CTest 3/3、完整 EGL 成像/队列
  检查通过。成像环境与下节相同，为 Mesa llvmpipe 软件渲染。
- UI 初始视角/图表开关与倾角提示已接入；本次未进行桌面交互验收。

---

# 异步相机暂停与时间回退修复（2026-10-16）

环境：Ubuntu 22.04、MuJoCo 3.3.7、GCC 11.4、ROS2 Humble；EGL 使用 Mesa
llvmpipe（LLVM 15.0.7）软件渲染。

- Release 独立版与 ROS2 Humble 版编译通过，ROS2 可执行文件 `--help` 正常。
- CTest 2/2 通过；原有 RGB 投影、红蓝灯条、暗背景、HDR Shader、异步方向与
  采集元数据成像检查全部通过。
- 新增真实 EGL 队列回归：单次提交后不再渲染也能收帧；重复轮询不重复出帧；
  重置与非零时刻回退清理旧图像；循环复用 PBO，保留各帧时间与内参。
- 将新回归运行在旧队列逻辑上（仅补轮询入口）会报
  `Reset published a stale frame or calibration`；修复后通过。
- 暂停期间 FPS、CPU/RSS 的墙钟更新已移出“仿真时间变化”分支。
- CI 的 push 检查覆盖 `main`、`feat/**`、`fix/**`，不再只检查已经合并的
  `feat/hdr-shader-telemetry` 分支。

本次未运行 GLFW viewer 的暂停交互或 ROS2 DDS 接收联调：当前执行环境无法
连接桌面显示。队列行为通过 EGL 实测，完整应用编译通过；RTX 4060 和
1280×1024 / 120 FPS 的端到端性能仍需目标机验证。未修改运行配置 `sim.yaml`。

---

# 暗环境补充验证（2026-10-14）

- 保留用户 `ff5d909` 提交的 1280×1024、真实 K、odom 和图像话题配置。
- 独立版编译、CTest 2/2，通过；CTest 使用固定 `tests/test_config.yaml`。
- C++ EGL 真实 RGB：黑天空/黑地板采样 <=2/255，红蓝灯条均存在灰度 >150
  且红蓝通道差 >60 的像素；同一个相机 GL 上下文切回原天空正常。
- 环境颜色/高光和灯条材质通过复制同步到独立相机模型，碰撞位掩码不变。
- 回归探针继续检查原彩色画面投影、灯条/子弹 emission、统计与几何。

检测算法本身尚未接入此验证；该预设不保证其面积、长宽比、颜色或配对条件全部通过。

---

# v0.2 验证记录（2026-10-14）

环境：MuJoCo 3.3.7，C++17，独立版 `ENABLE_ROS2=OFF`，EGL 软件渲染，Xvfb/X11。
没有 ROS2 Humble，也没有用户的 NVIDIA GPU 和完整官方 RMUC 场地资产。

已通过：

- 独立版 CMake 编译（core、camera、viewer、测试可执行文件）。
- CTest 2/2：保留原云台/弹丸生命周期回归；新增四板半径、高度、外法线 15°、
  正反转、颜色、自定义内参、环境灯光、真实弹道装甲命中、重复接触、未命中、
  有在飞子弹时重置统计、重置后新弹命中。
- C++ EGL RGB 渲染：fx≠fy、非居中主点的蓝色目标像素投影与光学 TF 一致；
  红/蓝发光网格在暗环境中可见；模型复制后颜色、半径、灯光、相机修改可见；
  绿色发光子弹可见。软件渲染探针约 60 FPS，仅用于验证图像，不代表实机帧率。
- 上游 `3db8efe6...` 合成场地夹具：38 个合成视觉网格、可验证 manifest 与浮点
  高度。上游验证 → 导入脚本 → C++ 场地 attach → 物理测试通过；所有高度样本
  与上游注入后的浮点值逐项完全一致。该夹具不代表官方场地精度。
- 实际 X11 viewer：现代面板、两张有界图表、F 切换、W 平移、方向键转向、
  按住左键连射、统计重置按钮、退出流程正常。XYZ 输入（含 Backspace）和红蓝下拉切换已实测。
  隐藏原生面板不会覆盖目标面板。
- `bash -n build.sh`、Python 脚本语法、`git diff --check`。

仍需用户本机验证：ROS2 Humble 编译/话题/CameraInfo 联调，完整 RMUC 场地包
导入后的外观与所需碰撞位置，RTX 4060 上 100–150 FPS 的接收端表现。
相机输出是无畸变针孔图；场地沿用上游 heightfield 物理范围限制。

---

以下保留 v0.1 的验证历史：

# 本次交付验证记录

日期：2026-10-13。环境：Ubuntu 24.04 容器、GCC 13.3、Python 3.12、pip MuJoCo 3.3.7；没有 ROS2 Humble，也没有用户的 NVIDIA GPU。

| 检查 | 结果 | 范围 |
|---|---|---|
| C++ Release 编译 | 通过 | 完整官方 viewer、云台/物理、对象池、相机线程、ROS stub 和测试目标 |
| 物理 CTest | 通过 | 手动平移/转向，自动角度跟踪，限幅，NaN 拒绝，手动/自动命令隔离 |
| 弹道与火控 | 通过 | 22 m/s 水平初速，重力抛物线，环境接触事件，15 Hz 连射，false 停止 |
| 生命周期 | 通过 | 飞行/碰撞后回收、模式切换清缓存、viewer reset、固定槽池耗尽与复用 |
| EGL 实际 C++ 渲染 | 通过 | 640×480 RGB 数据、红蓝靶标可见、标准 optical 坐标与针孔投影一致、绿色自发光弹丸可见 |
| 官方 viewer 启动/退出 | 通过 | Xvfb 下实际运行，独立相机线程创建、发布回调、停止/回收上下文 |
| 实际键鼠操作 | 通过 | F 切第一人称、W 平移、左方向键转向、按住左键发射；退出记录 yaw=0.63 rad、shots=3 |
| 菜单视觉检查 | 通过 | 模式/视角选项可见，角度/发射计数显示，短说明在侧栏内完整显示 |
| URDF 转换工具 | 通过 | 最小 URDF 的两个 visual geom 和固定 camera_link 都保留 |
| 配置与脚本 | 通过 | 62 个配置字段与 C++ 参数定义逐一匹配，XML/Python/shell 语法检查 |
| ROS2 Humble 编译/运行 | **未执行** | 当前环境缺少 Humble；包含 ROS2 接口实现、依赖、launch、控制示例和接收检查脚本 |
| 用户硬件 100–150 FPS | **未验证** | 需要在 Ubuntu 22.04 + NVIDIA GPU 上实测 |

软件渲染容器的三帧离屏读回检查约为 2.9 FPS；这个环境不能用来推断 RTX 4060 的帧率。相机线程已按 120 FPS 调度并独立于 viewer 的垂直同步，仍需要目标机验证完整渲染 + DDS 的接收速率。容器中的 viewer 也明显慢于实时，按住左键的墙钟时间不等于弹丸调度的仿真时间；15 Hz 精确连射由独立物理测试验证。

## 用户机器上的检查

```bash
conda activate rm_auto_aim_sim
source /opt/ros/humble/setup.bash
bash build.sh
source install/setup.bash
ros2 launch rm_auto_aim_sim sim.launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rm_auto_aim_sim check_ros.py --seconds 10 --min-fps 100
```

检查脚本使用墙钟测量真实接收 FPS，并验证 RGB 编码/长度、同 stamp 的 CameraInfo、JointState 和完整 TF 链。调整了话题名或 TF 名称后，同步传入脚本的相关参数。自动控制示例需要先在右侧菜单切到 AUTO；相关命令与单位见 README。

示例场景当前只有静态几何靶标；未实现旋转装甲板、比赛得分、机器人受击判定或完整事件回放。
