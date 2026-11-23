# 第三方组件与许可

本仓库是系统集成与实验配置项目，不宣称 NDT、GICP、BBS 或差速里程计为本项目原创算法。两个定位依赖通过 `dependencies.repos` 拉取，不在本仓库复制其源码。

| 组件 | 来源 | 许可说明 |
|---|---|---|
| lidar_localization_ros2 | https://github.com/rsasaki0109/lidar_localization_ros2 | 固定 commit 的 LICENSE 为 BSD 2-Clause，版权属于原作者 |
| ndt_omp_ros2 | https://github.com/rsasaki0109/ndt_omp_ros2 | BSD-2-Clause，保留原项目作者与许可声明 |
| ros2_control / ros2_controllers | https://github.com/ros-controls | 各包按其自身 LICENSE 和 package.xml 使用 |
| topic_based_ros2_control | https://github.com/PickNikRobotics/topic_based_ros2_control | BSD-3-Clause，参见所安装版本 |
| PCL | https://pointclouds.org/ | 配准和点云基础库，按上游许可使用 |
| NVIDIA Isaac Sim | https://github.com/isaac-sim/IsaacSim | 按实际版本及包含组件的许可使用；本仓库不分发安装包 |

本仓库中的场景覆盖层是在原始场景之上保存的参数与图接口修改，不包含原场景几何和贴图。原始场景、模型、材质、纹理和 PCD 不随本仓库打包或分发，运行者自行准备本地资源；这些素材不能自动套用本项目未来选定的代码许可证。

本项目自有代码/配置的开源许可证尚未指定；组织决定后应增加 `LICENSE`。这不更改任何第三方组件的许可。
