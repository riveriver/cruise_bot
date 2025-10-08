# 巡检项目流程运行手册

## 定位导航

### 设备

1. **livox mid360 激光雷达**
    - requirement: livox_ros_driver(ros package)
    - github:(https://github.com/Livox-SDK/livox_ros_driver/releases)

## 核心代码
### Ros_Package
#### FAST_LIO_LOCALIZATION-ROS-NOETIC
1. **localization.launch**
**导航运行逻辑的核心节点，包括全局定位里程计、位姿融合、初始位姿发布和地图发布。**
```
<!-- loalization-->
    <node pkg="fast_lio_localization" type="global_localization.py" name="global_localization" output="screen" />

    <!-- transform  fusion-->
    <node pkg="fast_lio_localization" type="transform_fusion.py" name="transform_fusion" output="screen" />

    <!-- inital pose-->
    <node pkg="fast_lio_localization" type="publish_initial_pose.py" name="publish_inital" output="screen" />

    <!-- global map-->
    <node pkg="pcl_ros" type="pcd_to_pointcloud" name="map_publishe" output="screen"
          args="$(arg map) 5 _frame_id:=map cloud_pcd:=/bim" />

```
- global_localization.py：
    - 全局定位里程计模块
- transform_fusion.py：

- publish_initial_pose.py：
    - 借助aruco标记初始化定位，发布初始位姿
- pcd_to_pointcloud：
发布全局bim地图,用于激光雷达扫描配准

2. **fast_lio.launch**
**导航启动的核心模块**
- lasermapping.py：
激光雷达扫描配准模块,包含雷达建图，配准，pcd保存等功能。
运行逻辑：从/start_lio话题接收定位开始信号，从/livox_points话题接收激光雷达点云数据，从/map话题接收全局地图数据。
开始导航需要命令`rostopic pub /start_lio std_msgs/Bool "data: true"`
决定是否保存pcd文件：`rostopic pub /save_pcd std_msgs/Bool "data: true/false (true为保存)"`