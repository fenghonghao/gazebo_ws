使用catkin build编译

roslaunch simlaunch map.launch 启动建图

simluanch launch相关

mapandnav 建图相关

carstate 坐标发布相关


实际上建图的实车根你仿真的实车根本不一样，投机取巧甚至可以从实车轨迹拉一条路径出来

改动：car_description 车辆建模描述
state_converter  车辆位姿数据转换为CarState消息
perception/src/lidar_processor.cpp   将 Gazebo 激光雷达点云转换为ConeDetections消息（供 mapandnav 节点使用）
simlaunch/worlds/race_track.world   定义仿真环境（赛道与锥桶）
新建controller包，使用纯追踪算法进行控制
新增 launch 文件（simlaunch/launch/control.launch）用于启动控制器节点

##新增的代码很乱，待优化 ，见谅