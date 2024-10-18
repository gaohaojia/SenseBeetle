# SenseBeetle

高颢嘉

# 下载代码

```bash
cd ~/
git clone --recursive https://github.com/gaohaojia/SenseBeetle
```

# 安装驱动

以下驱动请根据实际情况按需安装，Mid 360 为必要驱动，即使不使用 Mid 360，也需要安装相关驱动。

## Mid 360 驱动（必要）

```bash
cd ~/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

## Realsense 驱动

Realsense 驱动当前不支持 Jetson orin AGX。

```bash
sudo apt update
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-* -y
```

## 宇树 Unilidar 驱动

```bash
cd ~/SenseBeetle/src
git clone https://github.com/gaohaojia/Unitree_lidar_ros2
```

## 北科天绘 Dom 60 驱动

当前还存在部分 Bug。

# 安装额外功能

## Tare Planner 自主探索

该功能有待进一步调试，当前还存在不少 Bug。

```bash
cd ~/SenseBeetle/src
git clone https://github.com/gaohaojia/Tare_planner
```

## STD 回环检测

调试中

TODO

# 配置环境

```bash
cd ~/SenseBeetle
sudo apt update
sudo apt install ros-humble-desktop-full python3-rosdep libgflags-dev libgoogle-glog-dev ros-humble-pcl* -y
sudo rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

# 参数配置

~/SenseBeetle/src/bringup/launch/real_robot.launch.py 中的参数配置。

| 参数名称 | 可选参数 | 参数描述 | 默认参数 |
| --- | --- | --- | --- |
| planner_mode | none、tare_planner | planner 算法。只有安装对应功能后，设置此参数才有效。 | none |
| lidar_type | mid360、unilidar、dom60 | 雷达类型。只有安装对应雷达驱动后，设置此参数才有效。 | mid360 |
| robot_type | simulated、v4 | 机器人类型，即对应的预先设定好的参数值。 | simulated |

修改 ~/SenseBeetle/src/livox_ros_driver2/src/config/MID360_config.json 中的各个 IP 为雷达有线网口 IP。

## 调参（非必须）

**若使用的机器人不为 robot_type 参数所包含的可选项，则需要按照以下步骤调参，否则可跳过此步骤。**

1. 更改 ~/SenseBeetle/src/local_planner/paths/path_generator.m 第 114 行，将搜索半径设置为小车的半径（m）。
2. 运行修改后的 ~/SenseBeetle/src/local_planner/paths/path_generator.m 代码，将结果替换到 ~/SenseBeetle/src/local_planner/paths 路径里的文件。
3. 修改 ~/SenseBeetle/src/local_planner/config/local_planner_simulated.yaml 的以下参数。

| 参数名称 | 描述 |
| --- | --- |
| vehicleLength  | 车长（m） |
| vehicleWidth | 车宽（m） |
| obstacleHeightThre | 障碍物高度阈值（m） |
| minRelZ | （m） |
| maxRelZ | （m） |
| sensorOffsetX | 雷达相较于小车中心的 X 轴偏置（m） |
| sensorOffsetY | 雷达相较于小车中心的 Y 轴偏置（m） |
| cameraOffsetZ | 雷达相较于小车中心的 Z 轴偏置（m） |
| stopYawRateGain | 小车停止时Yaw轴增益率（个人理解） |
| pathScale | 路径大小（m） |
| minPathScale | 路径大小最小值（m） |
| pathScaleStep  | 路径大小步长（m） |

## 全向轮/差速地盘切换

修改 ~/SenseBeetle/src/local_planner/src/CMakeLists.txt 第 31 行和第 32 行。

### 全向轮

```bash
add_executable(localPlanner src/localPlanner_omnidirectional.cpp)
add_executable(pathFollower src/pathFollower_omnidirectional.cpp)
```

### 差速

```bash
add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)
```

## 雷达角度调整

若雷达并非水平向上安装，则需进行此调整。

修改 ~/SenseBeetle/src/lidar_transform/config/lidar_transform_params_simulated.yaml 中带有 lidar 前缀的六个参数，此六个参数表示雷达相较于小车中心位置的偏移。

## 配置服务器

修改 ~/SenseBeetle/src/robot_communication/config/robot_communication_params.yaml 的 network_ip 为服务器 ip，network_port 为服务器端口（详情请查看 [SenseServer 文档](https://www.notion.so/SenseServer-66ec20dd852645f99df47974ee3cc9cb?pvs=21) ）。

若同时启动多台机器人，请为 id 不为 0 的机器人设置偏移坐标实现全局点云配准。修改上述配置文件中带有 multi 前缀的六个参数，此六个参数表示为该机器人相对于 robot_0 机器人的偏移。

## 编译

本代码实现对 x86 和 arm 的自适应，会自动根据不同架构编译不同代码。

```bash
bash ~/SenseBeetle/toBuild.sh
```

## 启动

将 [robot_id] 替换为当前机器人的 id（当前支持的范围为 0-4），若单机启动请输入 0。

```bash
bash ~/SenseBeetle/run.sh [robot_id]
```

# 保存地图

先建立保存文件夹。

```bash
cd ~/SenseBeetle/
mkdir save
```

之后运行代码，待扫描全部完成后，按下 ctrl+c 终止代码将自动保存地图。