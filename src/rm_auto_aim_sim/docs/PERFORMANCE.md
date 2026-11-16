# 150 FPS 图像发布与性能定位

目标是保留 1280×1024 RGB8、每秒收到约 150 张不同时间戳的新图像。默认请求值
`camera_fps: 150.0`，viewer 用 `viewer_max_fps: 30.0` 给传感器让出渲染资源。
150 FPS 的每帧预算约 6.67 ms；仅提高请求频率不能突破渲染或 DDS 的处理上限。

## 先测真实接收频率

合并更新并执行 `bash build.sh` 后，在已 source ROS2 与 install/setup.bash 的终端：

```bash
ros2 run rm_auto_aim_sim camera_rate_probe --topic /image_raw --seconds 10 --min-fps 145
```

此 C++ 接收器使用 SensorDataQoS / best effort / depth=2，只统计消息元数据和长度，
不逐像素处理。它按完整的墙钟测量窗口计数，包含停顿，不会把短暂的高频突发
当成持续帧率；重复/倒退时间戳、无效 RGB 布局会使验收失败。测量时不要暂停或
重置仿真，也不要启用低于实时的播放速度。145 是约 150 FPS 的可调验收下限。

`ros2 topic hz` 或 Python 接收器本身也会消耗资源；请先用这个轻量接收器复测。
TF/CameraInfo 的完整接口检查仍可使用 `check_ros.py`，它与吞吐测速的目的不同。

## UI 应该看哪些数值

| 指标 | 含义 | 低于目标时检查 |
|---|---|---|
| Capture FPS | 完成 GPU 读回的新图像数/墙钟秒 | GPU、场景、采样调度和物理快照 |
| Publish FPS | ROS 发布回调完成次数/墙钟秒 | 发布耗时、传输和发布队列丢弃 |
| C++ Received FPS | 接收端实际收到的图像数/墙钟秒 | DDS、订阅 QoS、接收端负载 |
| GPU render / GPU transfer | GPU 绘制和读回传输耗时 | 绘制、光源数量、阴影和像素负载 |
| ROS publish | 独立发布线程完成一帧回调的耗时 | 图像序列化、DDS 写入和订阅者 |
| GPU drops | GPU 队列忙或时间回退丢弃的采集 | 显卡负载和重置 |
| Publish drops | 一张待发图像被更新帧替换，或回退时清除 | 发布端跟不上采集 |

Publish FPS 表示调用完成，不等于 DDS 对端已接收；必须一起看 Received FPS。
相机与发布线程分开后可以重叠工作，但如果任意阶段持续慢于 6.67 ms，仍不能
承诺同分辨率下稳定 150 FPS。队列不会补发同一帧，也不会累积越来越旧的图像。

## 单独检查 ROS 传输能力

1280×1024 RGB8 每帧 3.75 MiB，150 FPS 的净像素数据约 562.5 MiB/s，尚未计入
序列化和传输开销。先关闭仿真器和图像显示工具，两个终端使用相同 ROS 环境，
运行独立的合成传输测试（它不代表真实渲染帧率）：

终端 A：

```bash
ros2 run rm_auto_aim_sim camera_rate_probe --publish --fps 150 --seconds 15 --min-fps 145
```

终端 B（发布开始后尽快运行）：

```bash
ros2 run rm_auto_aim_sim camera_rate_probe --topic /sim/benchmark/image_raw --seconds 10 --min-fps 145
```

`--publish` 明确是合成数据，默认使用单独的 `/sim/benchmark/image_raw` 话题，
不会向真实 `/image_raw` 注入假图像。未发现订阅者时发布测试失败；发布端计数
和接收端计数需要分别检查。若这个测试也低于 145，先处理传输/订阅端问题。

### 同机 Fast DDS 大图配置

仓库提供 `config/fastdds_camera.xml`，显式创建每 participant 16 MiB 的 SHM 段，
同时保留 UDP 发现和远端通信。它不改变应用的 best effort QoS，不是零拷贝，
也不保证某台机器达到特定频率。用于当前约 4 MiB 图像；更大分辨率需调整容量。

在发布端、接收端及算法节点启动前的各个终端，从仓库根目录设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/rm_auto_aim_sim/config/fastdds_camera.xml"
```

随后重新启动双方进程，再比较接收频率。其他 RMW 不使用这个配置。容器通信
还要共享可用的 IPC/SHM 资源；该配置不替你修改主机或容器设置。

参考：[Fast DDS SHM 文档](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/shared_memory/shared_memory.html)、
[XML transport 配置](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/xml_configuration/transports.html)。
保持同步 DDS 写入模式，应用已有独立发布线程；不叠加不可见的异步发送队列。

## 单独检查渲染上限

在包含 MuJoCo 3.3.7 的 Python 环境，独立版构建后运行：

```bash
MUJOCO_GL=egl python src/rm_auto_aim_sim/tests/benchmark_camera.py \
  --library build/standalone/libaim_render_probe.so \
  --config src/rm_auto_aim_sim/config/sim.yaml --seconds 5
```

colcon 构建将 library 路径换成 `build/rm_auto_aim_sim/libaim_render_probe.so`。
这个静态场景极限基准调用实际 C++ Shader/PBO，排除 viewer、DDS 和采样限频；
输出物理步进、快照复制、完整 forward 的对照耗时，以及 GPU/CPU 阶段与渲染 FPS。
它不向 ROS 发布图像，不能用于宣称端到端接收帧率。

如果渲染上限足够、Publish FPS 足够，而接收低，应继续看 DDS/订阅负载；如果
渲染上限本身低于 150，则需要结合 GPU render/transfer 决定下一步优化。
`viewer_max_fps: 0` 可取消额外预览限速；增大预览频率会增加竞争。

## 本轮实现与验证范围

- 独立发布线程：一张正在发布、一张待发；过载只替换待发帧，计数丢弃。
- 回收 ROS Image 的 RGB vector 并在读回/发布队列循环复用，减少多 MiB 分配。
- 物理步前只更新枪口运动学，保留步后完整 forward；渲染使用已计算的快照。
- Shader 排除远在可视范围之外的停放弹丸光源，保留场景灯条和近处弹丸。
- CPU 不等待 GPU fence；重置同时清理待发布队列；退出和发布异常有回归测试。

本地 Mesa llvmpipe 对照中，1280×1024、用户当前光照设置的静态渲染约从
27～29 FPS 到 40～41 FPS；物理步进约从 0.083～0.088 ms 到 0.069～0.070 ms。
这组软件渲染结果只证明该环境中的开销下降，不能外推 NVIDIA 性能。

本地执行环境无可用 NVIDIA 驱动，DDS 网络接口/套接字也受限制，无法验收目标
机器的实际 ROS 接收 150 FPS。GitHub CI 的同尺寸 ROS 传输测试是功能冒烟，
请求 150 Hz、最低只要求 10 Hz，以免把共享 runner 当性能验收硬件。目标机请使用
上面的 145 Hz 门槛；当前尚未宣称其完整链路达到该门槛。
