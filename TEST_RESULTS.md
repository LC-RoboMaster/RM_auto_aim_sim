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
