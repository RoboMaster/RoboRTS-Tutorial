# 全局路径规划

## 模块介绍

全局路径规划（简称全局规划）是导航系统中运动规划的第一个步骤，在给定目标位置后，根据感知的全局代价地图搜索得到一条无碰撞的最短路径（即一系列离散的坐标点），然后作为输入传递给局部轨迹规划模块控制机器人的具体运动。

全局路径规划模块位于`roborts_planner`包中，依赖`roborts_msgs`包中的Action消息与`roborts_common`包中的抽象工厂模式和参数读取，模块文件目录如下所示

```bash
└── global_planner/
    ├── CMakeLists.txt
    ├── global_planner_algorithms.h #包含实现具体算法类的头文件
    ├── global_planner_base.h       #全局规划算法的抽象类
    ├── global_planner_test.cpp     #全局规划的测试节点
    ├── global_planner_node.cpp     #全局规划核心功能执行的节点和Main函数
    ├── global_planner_node.h       
    ├── a_star_planner              #Astar全局规划算法实现
    │   └── ...
    ├── config
    │   └── global_planner_config.prototxt # 全局规划参数配置文件
    └── proto  
        ├── global_planner_config.pb.cc
        ├── global_planner_config.pb.h
        └── global_planner_config.proto    # 全局规划参数定义文件
 ```

全局路径规划的相关算法参考[A Star算法](sdk_docs/roborts_planning_global_planner?id=a) 

global_planner_node为核心规划节点，ROS节点图示为

![](https://rm-static.djicdn.com/documents/20758/63b3d8db24ce71547553505842780076.png)

节点的输入输出如下

### 输入

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  (必须) 通过`TransformListener`监听到的base_link->map的变换，由`roborts_localization`包提供

* global_costmap对象 (`roborts_costmap/CostmapInterface`)

  (必须) 可以表述静态层和障碍物层（可选）的全局栅格代价地图（global_costmap），依赖话题/tf, /map(静态层)和/scan（障碍物层可选)，由`roborts_costmap`包提供

* /global_planner_node_action/goal ([roborts_msgs/GlobalPlannerGoal]())

  (必须) 由`Actionlib Client`向`Actionlib Server`输入全局规划的目标点，接口由Actionlib封装，具体消息类型为([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

* /global_planner_node_action/cancel ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

  由`Actionlib Client`向`Actionlib Server`申请取消正在运行的全局规划任务，接口由Actionlib封装

### 输出

* /global_planner_node_action/feedback ([roborts_msgs/GlobalPlannerFeedback]())

  `Actionlib Server`实时反馈的规划路径，接口由Actionlib封装，具体消息类型为 [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)

* /global_planner_node_action/result ([roborts_msgs/GlobalPlannerResult]())

  `Actionlib Server`反馈规划的结果，即是否判断到达目标点，接口由Actionlib封装

* /global_planner_node_action/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))  

  `Actionlib Server`实时反馈的规划状态，接口由Actionlib封装

* /global_planner_node/path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  

  用于可视化显示的规划路径

* /global_costmap/global_costmap/costmap ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))  

  用于可视化显示的全局代价栅格地图

### 相关参数

参数的定义见`proto/global_planner_config.proto`，参数的配置见`config/global_planner_config.prototxt`

* selected_algorithm(`string`, 默认值: "a_star_planner")

    选择全局规划算法的名称

* frequency(`int`, 默认值: 3)

    全局规划算法的频率

* max_retries(`int`, 默认值: 5)

    允许全局规划失败重试的次数

* goal_distance_tolerance(`double`, 默认值: 0.15)

    全局规划到达目标的欧式距离误差

* goal_angle_tolerance(`double`, 默认值: 0.1)

    全局规划到达目标的角度（单位弧度制）误差

### 执行流程

核心规划节点`global_planner_node_action`的执行流程如下

- 初始化： 
  - 初始化Actionlib Server，构建可视化的publisher
  - 读取参数
  - 创建tf listener，global_costmap对象
  - 创建具体算法的planner对象

- 初始化成功后，开启规划线程，ROS回调队列在主线程开始回调，同时线程Actionlib Server Callback也开始回调
  - 规划线程执行流程图
    ![](https://rm-static.djicdn.com/documents/20758/057c895762b7d1547553536324774678.png)
  - Actionlib Server Callback线程执行流程图
    ![](https://rm-static.djicdn.com/documents/20758/b93803f9be2aa1547553557409469215.png)


## 编译与运行

### 编译

在ROS的工作区内编译

```shell
catkin_make -j4 global_planner_node global_planner_test
```

### 运行

在仿真环境中测试

```shell
roslaunch roborts_bringup roborts_stage.launch
```

启动全局路径规划测试节点

```shell
rosrun roborts_planning global_planner_test
```
开启Rviz，显示了模块所需的输入输出，如下图所示

![](https://rm-static.djicdn.com/documents/20758/844f7b73e9a091547553578138533412.png)

- Pose: 红色箭头为规划目标点
- LaserScan: 黄色点集为激光雷达扫描数据
- TF: 参见TF的坐标系
- Map: 灰黑部分为静态地图
- Map: 不同颜色（紫\红\青\蓝等）显示不同代价的全局代价地图
- Path: 绿色路径为规划路径

## A*
### 算法介绍

A*（A-Star)算法是一种静态路网中求解最短路最有效的方法。公式表示为：

$$ f(n) = g(n) + h(n) $$

其中$f(n)$是节点n从初始点到目标点的估价函数，$g(n)$是在状态空间中从初始节点到n节点的实际代价，$h(n)$是从n到目标节点最佳路径的估计代价。在实际的全局路径规划中，静态网路由全局代价地图提供。

### 相关参数
* inaccessible_cost(`int`, 默认值: 253)

    不可通过（即视为致命障碍物）的地图栅格代价值

* heuristic_factor(`double`, 默认值: 1.0)

    启发因子，见启发函数`h(n) = heuristic_factor * manhattan_distance`

* goal_search_tolerance(`double`, 默认值: 0.45)

    寻找新目标点的容忍距离（单位米），如果发送的目标点不可通过，则在此允许距离周围内找一个可以通过的新目标点


