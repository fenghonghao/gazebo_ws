改动：car_description 车辆建模描述
state_converter  车辆位姿数据转换为CarState消息
perception/src/lidar_processor.cpp   将 Gazebo 激光雷达点云转换为ConeDetections消息（供 mapandnav 节点使用）
simlaunch/worlds/race_track.world   定义仿真环境（赛道与锥桶）
新建controller包，使用纯追踪算法进行控制
新增 launch 文件（simlaunch/launch/control.launch）用于启动控制器节点

##新增的代码很乱，待优化 ，见谅

剩下村跟踪需要调整