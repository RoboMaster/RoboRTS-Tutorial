# 整体架构

## 架构与模块介绍
整体系统以 `机器人传感器->感知->决策->规划->控制->执行器` 的环路进行架构，不同模块具体以ROS Package的形式维护，模块和其数据流如下图所示
![](https://rm-static.djicdn.com/documents/20758/f42d65d85d97c1547553106539783606.png)

### 传感器、控制器与执行器

- 中心模块集成传感器模块（雷达、相机、IMU等）、嵌入式控制平台（执行实时任务，如闭环控制和数据采集与预处理）与执行器（电机等），负责sensing和control两大任务，具体ROS Package为包含嵌入式SDK的[roborts_base](sdk_docs/roborts_base)，相机的[roborts_camera](sdk_docs/roborts_camera)以及相关传感器驱动包

### 感知部分

感知部分包括机器人定位、地图的维护和抽象、目标识别与追踪等

- localization模块负责机器人定位，详见[roborts_localization](sdk_docs/roborts_localization)

- map模块负责机器人地图维护，目前采用ROS开源Package [map_server](http://wiki.ros.org/map_server)

- costmap模块负责代价地图维护，集成了静态地图层，障碍物层和膨胀层，主要用于运动规划部分,详见roborts_costmap，后续将会计划更新为feature_map模块，不单纯针对规划使用。

- detection模块负责目标识别和追踪，详见[roborts_detection](sdk_docs/roborts_detection)，当前主要集成了敌人装甲板的识别和发射弹丸的控制器，由于帧率需求比较高，当前识别和控制是耦合的，后续将会解耦，将控制跟随部分放到gimbal_executor中

### 任务调度与决策部分

任务调度与决策部分包括调度感知输入模块和调度规划执行输出模块的接口，以及决策的核心框架。
- decision模块为机器人决策框架，官方提供行为树（BehaviorTree）的决策框架。blackboard模块调度各种模块的感知任务获取信息和裁判系统的比赛信息，behavior模块集成了离散动作空间的各种动作或行为，详见[roborts_decision](sdk_docs/roborts_decision)

- executor模块是behavior模块的依赖，其包含底盘和云台内不同模块内不同抽象程度的机器人任务委托接口（例如调度底盘运动规划执行），详见roborts_decision/executor

### 运动规划部分

运动规划部分是运动规划功能模块，由决策部分中chassis_executor模块来调度完成导航，详见roborts_planning

- global_planner模块负责机器人的全局路径规划，详见[roborts_planning/global_planner](sdk_docs/roborts_planning_global_planner)，依赖roborts_costmap

- local_planner模块负责机器人的局部轨迹规划模块，详见[roborts_planning/local_planner](sdk_docs/roborts_planning_local_planner)，依赖roborts_costmap


## ROS Package介绍

| Package               |  功能           | 内部依赖     | 
| :--:                  | :------------: | :------: |
|  roborts              |  Meta-package  |   - |    
|  roborts_base         | 嵌入式通信接口   | roborts_msgs | 
|  roborts_camera       | 相机驱动包 | roborts_common |    
|  roborts_common       |   通用依赖包   |    -    |          
|  roborts_decision     |  机器人决策包  | roborts_common<br/>roborts_msgs <br/>roborts_costmap | 
|  roborts_detection    |  视觉识别算法包  | roborts_msgs<br/>  roborts_common<br/>  roborts_camera |  
|  roborts_localization |  机器人定位算法包  |    -    | 
|  roborts_costmap          |  代价地图相关支持包  | roborts_common |    
|  roborts_msgs         |  自定义消息类型包  |    -    | 
|  roborts_planning     |  运动规划算法包  | roborts_common<br/>  roborts_msgs<br/>roborts_costmap | 
|  roborts_bringup        |  启动包  | roborts_base<br/>  roborts_common<br/>  roborts_localization<br/>  roborts_costmap<br/>  roborts_msgs<br/>  roborts_planning<br/>  |    
|  roborts_tracking     |  视觉追踪算法包  | roborts_msgs | 
