# 参数配置与 UI 对照

集中配置文件是 [`../config/sim.yaml`](../config/sim.yaml)，已按场景/UI、相机标定、
光照、目标、控制、弹丸、模型、TF、话题分组。每个参数都有中文注释；所有
`config_fields.def` 中声明的参数都在该文件中显式列出。

`config_fields.def` 的默认值仅在 YAML 未提供该字段时作为缺省值。示例 YAML
保留当前相机标定和黑背景预设，部分值与缺省值不同；UI 使用实际加载的配置初始化。

## 选择实际加载的配置

从仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_auto_aim_sim sim.launch.py config:="$PWD/src/rm_auto_aim_sim/config/sim.yaml"
```

独立版可以使用：

```bash
build/standalone/rm_auto_aim_sim --config "$PWD/src/rm_auto_aim_sim/config/sim.yaml"
```

`scene_path` 和非空的 `field_xml` 相对所选 YAML 的目录解析。ROS2 launch 不指定
`config` 时读取当前所 source 的安装目录中的配置；普通安装后修改源码 YAML
需要重新构建/安装，显式传上述路径能避免读到旧副本。

配置在启动时读取并校验，修改磁盘文件后需要重启。直接使用可执行文件时，
`--ros-args -p camera_exposure:=1.5` 等 ROS 参数覆盖随后生效。参数声明为
read-only，不支持用 `ros2 param set` 在运行中改值。

UI 控件会实时改变当前场景或相机线程，**不会写回 YAML，也不会同步修改 ROS
参数服务器显示的启动值**。`Restore YAML look` 恢复本次启动时读取的 9 个外观值
（包含 ROS 参数覆盖），不会重新读取磁盘，也不会恢复内参或目标位置。

## CAMERA / LIGHT 的对应关系

下表范围来自 C++ 校验；曝光的 UI 滑条范围是 0.1～8，YAML 允许任意大于 0 且
不超过 8 的有限值。倍率均为仿真渲染参数，不是经过标定的物理照度或相机曝光时间。

| UI 名称 | YAML 参数 | 范围 / 单位 | 效果与生效条件 |
|---|---|---|---|
| fx / fy [px] | `camera_fx`、`camera_fy` | >0，像素 | 启动时需 `camera_use_intrinsics: true`；UI 修改后点 Apply intrinsics |
| cx / cy [px] | `camera_cx`、`camera_cy` | [0,width)、[0,height)，像素 | 与实际投影、CameraInfo 同步；像素原点在左上 |
| Scene light | `lighting_brightness` | [0,3] | 缩放场景基础灯光和头灯，不关闭灯条自发光 |
| Black sky | `dark_sky` | bool | true 关闭天空盒和雾，背景变黑 |
| Floor / scene color | `environment_albedo` | [0,1] | 静态环境表面颜色和高光倍率，0 为黑色，1 恢复原色 |
| LED white mix | `target_light_whiteness` | [0,1] | 将红蓝灯条颜色混向白色；增大后灰度更高，但色差更小 |
| Shader camera | `camera_shader` | bool | 控制独立 RGB 的 HDR 补光、光晕、曝光和色调映射；要求 `camera_msaa: 0` |
| Shadows | `camera_shadows` | bool | 独立 RGB 的基础光源阴影开关；原生/Shader 两种模式均可用 |
| Exposure | `camera_exposure` | (0,8] | 整幅 RGB 固定曝光倍率，仅 Shader 模式生效 |
| Halo strength | `camera_bloom` | [0,2] | HDR 光晕叠加强度，0 关闭叠加，仅 Shader 模式生效 |
| LED HDR brightness | `camera_lamp_intensity` | [0,20] | 附加灯条补光和发光强度；0 仍保留 MuJoCo 原生灯条自发光，仅 Shader 模式生效 |

Scene light、Black sky、Floor / scene color、LED white mix 同时作用于原生 viewer
和独立相机。Shader、Shadows、Exposure、Halo、LED HDR brightness 只改变独立
相机输出，原生 viewer 的 Gimbal 视角不会显示这套后处理。查看算法实际输入请
订阅 YAML 的 `topic_image`，当前为 `/image_raw`。

阴影开关控制 MuJoCo 基础光源的阴影；Shader 附加的 LED 补光没有独立的阴影贴图。
灯条为白色混合 1 时红蓝色差消失。背景表面颜色为 0 时，增加场景灯光也不能
恢复表面原色；需要一起提高 `environment_albedo`。

当前文件保留黑背景预设：Scene light=0.03、Black sky=true、Floor / scene color=0、
LED white mix=0.65、Shader=true、Exposure=1、Halo=0.9、LED HDR brightness=8。

## 其他 UI 控件

| UI 控件 | YAML 参数 | 说明 |
|---|---|---|
| Control mode | `initial_mode` | `manual` / `auto`，初始控制模式 |
| Camera view | `initial_view` | `orbit` / `gimbal`，初始观察视角；不决定 RGB 相机位姿 |
| Center XYZ [m] | `target_x/y/z` | 旋转中心世界坐标，m |
| r1 / r2 [m] | `target_r1/r2` | 0/2、1/3 号相对板的两组半径，m |
| h1 / h2 [m] | `target_h1/h2` | 相对中心 Z 的两组装甲中心高度，m |
| Color | `target_color` | `red` / `blue` |
| Speed [rad/s] + Direction | `target_angular_speed` | 正数为绕 +Z 逆时针，负数为顺时针；UI 将绝对值与方向分开显示 |
| Show charts | `show_charts` | 云台角度/命中率图的初始显示状态 |
| Show perf plots | `show_perf_plots` | 性能图的初始显示状态 |

`target_tilt_deg` 控制装甲外法线上倾角，UI 的 Geometry 显示实际配置值。
`target_initial_angle` 控制初始朝向；Reset heading 是运行时将朝向置零的动作。
Apply intrinsics、Restore YAML look、Reset statistics 等按钮属于动作，不对应持续数值。
FPS、CPU、RSS、命中数等文本是实测输出，不是可调参数。

## 参数完整性检查

`configuration_contract` 随 CTest 运行：检查示例 YAML 与声明字段完全对应，拒绝
重复/遗漏项，调用实际配置加载/校验，并检查内参、目标及环境参数进入 Engine。
新增参数时需要同时补充 YAML 的数值与注释。参数拼写错误、非法范围和错误的
启动视角会在启动时明确报错。
