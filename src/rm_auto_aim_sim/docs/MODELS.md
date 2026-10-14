# 替换几何体、导入 URDF 与扩展目标

## 当前模型契约

运行时通过 YAML 的 `model_*` 名称查找对象，启动时检查结构。底座是 world 的固定 mocap 子体；yaw/pitch 是两个 hinge 关节。相机本体是 pitch 的固定子体，`gimbal_camera` 挂在相机本体，`muzzle` site 挂在 pitch。

| 默认对象 | 条件 |
|---|---|
| `base_link` body | `mocap="true"`，无自由关节，world 的直接子体 |
| `yaw_link` body + `yaw_joint` | base 的直接子体，hinge，默认轴 +z |
| `pitch_link` body + `pitch_joint` | yaw 的直接子体，hinge，默认轴 −y |
| `camera_link` body | pitch 的直接固定子体，保留真实相机外参 |
| `gimbal_camera` camera | 挂在 camera_link，朝向与实际光轴一致 |
| `muzzle` site | 挂在 pitch，site +x 是发射方向 |

你可以把这些对象改名，然后修改 YAML 绑定。TF 名称独立配置。角度正方向由 URDF/MJCF 关节轴决定；替换后应维持 +yaw 向左、+pitch 抬头，或同步修改算法端约定。固定 camera/muzzle 外参从 MuJoCo 真实位姿推导，不在 ROS 发布代码中写死。

最简单的替换是保留 `scenes/gimbal.xml` 的关节树和相机/site，用 mesh geom 替换 box/cylinder。资产写在 arena 的 `<asset>` 中；相对 mesh 路径相对于模型文件位置，mesh 必须携带到工程中。这样控制/TF/火控逻辑无需修改。

## URDF 转换

MuJoCo 能加载 URDF，但 ROS 的 `package://` 资源解析、xacro 和复杂材质并不会自动替你处理。先展开 xacro，解析 mesh 资源为实际路径：

```bash
source /opt/ros/humble/setup.bash
ros2 run xacro xacro your_gimbal.urdf.xacro > /tmp/gimbal.urdf

# 在含 MuJoCo 的 Conda 环境运行转换工具
python src/rm_auto_aim_sim/scripts/convert_urdf.py \
  /tmp/gimbal.urdf /tmp/gimbal_converted.xml
```

转换工具保留固定 body、保留 visual geom，并提前检查编译。转换完成后不是直接“无配置替换”：需要按上表整理 yaw/pitch/camera 命名与父子结构，把 base 改成 mocap 固定体，补充 MuJoCo camera 和 muzzle site。MuJoCo 可能把固定 link 优化合并，转换工具禁用 `fusestatic` 以便保留命名。

把转换后的资产与云台 body 子树合并到 `gimbal.xml`，保留 arena 的环境、灯光与默认物理选项；不要把两个完整 `<mujoco>` 模型直接拼接。可以建立独立 include 文件，把可视 mesh 与测试场景分开。

## 碰撞设置

本版本的场景 geoms 默认 `contype="1" conaffinity="2"`，弹丸为 `contype=2 / conaffinity=1`，禁用弹丸相互碰撞。云台外观 geoms 是 0/0，避免炮口弹丸撞到云台自身；底座按用户指定位置运动，不支持底盘撞墙。新靶标需要设置与环境相同的碰撞位，才能接收弹丸接触。

旋转装甲板、机器人实体可以在 arena 的 worldbody include 新 XML。用于接触统计的碰撞外形尽量使用 box/cylinder 等简单几何体；用于识别画面的 mesh 可以设置不碰撞。视觉和碰撞 geom 可以共用同一个 body。

## 建议检查

换模型后先启动，确认名称和父子结构检查通过；分别输入正 yaw 和正 pitch，检查方向；查看 `world -> camera_optical_frame` TF 与相机图像；用 ROS CameraInfo 的 K 进行目标点投影。枪口方向和相机光轴可以有真实偏差，需要在算法端做外参、视差和弹道补偿。

导入模型附带其他可动关节时，当前 TF 发布器只覆盖本项目定义的二轴链，额外关节需要扩展绑定与发布。相机当前使用垂直 FOV 理想针孔模型；修改 imported camera 时保持透视相机和 centered FOV 模式，不要混用额外 sensor/focal 参数。
