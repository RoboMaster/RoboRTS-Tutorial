# 局部轨迹规划

## 模块介绍

局部路径规划模块根据机器人的里程计信息,雷达的感知信息,结合全局路径规划给出的最优路径,计算出机器人当前能够避免与障碍物碰撞的最优速度.

局部路径规划模块位于`roborts_planner`包中，依赖`roborts_msgs`包中的action与msgs消息与`roborts_common`中的抽象工厂模式和参数读取。
 
局部路径规划层级简略如下:

```bash
local_planner
├── CMakeLists.txt
├── config
│   └── local_planner.prototxt          #局部路径规划算法配置文件
├── include
│   └── local_planner
│       ├── proto 
│       │   ├── local_planner.pb.cc
│       │   ├── local_planner.pb.h
│       │   └── local_planner.proto     # 局部路径规划参数生成文件
│       ├── data_base.h                 # 局部路径规划数据结构,用于g2o构建边以及顶点
│       ├── data_converter.h            # 数据转换,将数据转换为data_base数据结构
│       ├── distance_calculation.h      # 计算二维点、线、几何图形之间的距离
│       ├── line_iterator.h             # 连续线段在离散栅格中的坐标计算
│       ├── local_planner_algorithms.h  # 局部路径规划算法头文件（所有算法头文件都应在此文件中引入）
│       ├── local_planner_base.h        # 局部路径规划算法父类
│       ├── local_planner_node.h        # 局部路径规划入口节点
│       ├── local_visualization.h       # 可视化局部路径规划结果
│       ├── obstacle.h                  # 二维障碍物信息
│       ├── odom_info.h                 # 里程计信息
│       ├── optimal_base.h              # 优化相关算法父类
│       ├── robot_footprint_model.h     # 机器人外形描述
│       ├── robot_position_cost.h       # 机器人所在位置代价计算，用于判断路径是否可行
│       └── utility_tool.h              # 通用函数
├── src
│   ├── local_planner_node.cpp          # ros node文件，负责局部路径规划内的逻辑调度
│   ├── vel_converter.cpp               # ros node文件, 仿真时将局部路径规划速度转换为“cmd_vel”发布
│   ├── teb_test.cpp                    # 算法timed elastic band 测试文件
│   └── ...
└── timed_elastic_band
    ├── CMakeLists.txt
    ├── config
    │   └── timed_elastic_band.prototxt # timed elastic band 配置文件
    ├── include
    │   └── timed_elastic_band
    │       ├── proto
    │       │   ├── timed_elastic_band.pb.cc
    │       │   ├── timed_elastic_band.pb.h
    │       │   └── timed_elastic_band.proto
    │       ├── teb_local_planner.h     #算法timed elastic band实现，继承local_planner_base.h
    │       ├── teb_optimal.h           # g2o优化逻辑，继承optimal_base.h
    │       └── ...                     # 其他g2o的边以及顶点相关文件。
    └── src
        ├── teb_local_planner.cpp       # 算法timed elastic band实现
        ├── teb_optimal.cpp             #g2o优化逻辑
        └── ...

```

局部路径规划的相关算法参考 [Time Elastic Band](/sdk_docs/roborts_planning_local_planner?id=timed-elastic-band)

local_planner_node为核心规划节点，ROS节点图示为:

![](https://rm-static.djicdn.com/documents/20758/708f059fc647c1547553602751260545.png)

节点的输入输出如下

### 输入

- /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  (必须) 通过`TransformListener`监听到的map->odom的变换，在roborts框架内由roborts_localization Package提供

- local_costmap对象 (`roborts_costmap/CostmapInterface`)

  (必须) 可以表述障碍物层的局部栅格代价地图（local_costmap），依赖话题/tf, /scan（障碍物层)，在roborts框架内由roborts_costmap Package提供

- /local_planner_node_action/goal (roborts_msgs/LocalPlannerGoal)

  (必须) 由`Actionlib Client`向`Actionlib Server`输入由全局路径规划得到的path，接口由Actionlib封装，具体消息类型为([nav_msgs/Path](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html))

- /locall_planner_node_action/cancel ([actionlib_msgs/GoalID](http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalID.html))

  由`Actionlib Client`向`Actionlib Server`申请取消正在运行的局部规划任务，接口由Actionlib封装

### 输出

- /local_planner_node_action/feedback (roborts_msgs/LocalPlannerFeedback)

  `Actionlib Server`实时反馈的局部路径规划的错误码,错误信息，接口由Actionlib封装.

- /local_planner_node_action/result (roborts_msgs/LocalPlannerResult)

  暂未使用

- /local_planner_node_action/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatusArray.html))  

  `Actionlib Server`实时反馈的规划状态，接口由Actionlib封装

- /local_planner_node/trajectory([nav_msgs/Path](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html))  

  用于可视化显示局部路径

- /local_costmap/local_costmap/costmap ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))  

  用于可视化显示的局部代价栅格地图



## 编译与运行

### 编译

在编译之前, 确保已经安装所有依赖。在 ros 工程目录下运行一下指令

```shell
catkin_make local_planner_node vel_converter teb_test
```

### 运行

```shell
# 启动局部路径规划节点
rosrun roborts_planning local_planner_node
# 启动vel_converter
rosrun roborts_planning vel_converter
# 启动timed elastic band测试节点
rosrun roborts_planning teb_test
```

或从 launch 文件启动

```xml
<!-- Run the local planner Node -->
<node pkg="roborts_planning" type="local_planner_node" name="local_planner_node" respawn="false" />
<!-- Run the vel converter Node -->
<node pkg="roborts_planning" type="vel_converter" name="vel_converter_node" respawn="false" />
```

## Timed Elastic Band
### 算法介绍

算法建立了轨迹执行时间的目标方程，在满足与障碍物保持安全距离, 运动时遵守动力学约束的前提下,优化机器人实际运行轨迹,最终求出机器人可执行的速度。具体理论可参考论文:

* C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.

### 相关参数

#### 轨迹相关

* teb_autosize 
  
    是否重新计算路径

* dt_ref 
  
    初始时间常数

* dt_hysteresis 
  
    时间补偿常数

* global_plan_overwrite_orientation 
  
    是否重写路径规划点的朝向 

* allow_init_with_backwards_motion 
  
    是否允许后退运动 

* global_plan_viapoint_sep 
  
    选取必须通过的全局路径规划点

* via_points_ordered 
  
    按顺序通过必经路径点

* max_global_plan_lookahead_dist 
  
    局部路径规划截取全局路径规划距离

* exact_arc_length 
  
    两个位置之间以弧线连接

* force_reinit_new_goal_dist 
  
    重新规划局部路径

* feasibility_check_no_poses 
  
    计算路径可行性点的个数

* publish_feedback 
  
    无用参数

* min_samples 
  
    路径最小点的个数

* max_samples 
  
    路径最多点的个数


#### 运动学参数


* max_vel_x 

    最大前进运动速度

* max_vel_x_backwards 

    最大后退速度

* max_vel_y 

    最大平移速度

* max_vel_theta

    最大角速度

* acc_lim_x

    最大纵向加速度

* acc_lim_y 

    最大平移加速度

* acc_lim_theta 

    最大角加速度

* min_turning_radius

    最小转弯半径

* wheelbase

     无用参数

* cmd_angle_instead_rotvel

    无用参数

#### 容忍度参数

*   xy_goal_tolerance

    允许距离误差

*   yaw_goal_tolerance

    允许角度误差

*   free_goal_vel

    终点零速


#### 障碍物参数

* min_obstacle_dist

    障碍物最小距离
    
* inflation_dist

    障碍物膨胀大小

* include_costmap_obstacles

    是否考虑代价地图中的障碍物,无用参数

* costmap_obstacles_behind_robot_dist

    考虑车后障碍物距离阈值

* obstacle_poses_affected

    障碍物影响路径点的数目,仅在legacy_obstacle_association为true有用

* legacy_obstacle_association

    选择以离障碍物最近的路径点优化,还是离路径点最近的障碍物优化

* obstacle_association_cutoff_factor

    不考虑的障碍物因子,legacy_obstacle_association为false有用

* obstacle_association_force_inclusion_factor

    强制考虑障碍物因子,工作条件同上

#### 优化参数

* no_inner_iterations 

    每一次调整轨迹后,求解器迭代次数

* no_outer_iterations 

    自动调整轨迹次数

* optimization_activate
    是否开始优化

* optimization_verbose

    是否输出调试信息

* penalty_epsilon

    约束惩罚函数补偿项

* weight_max_vel_x

    最大x速度权重

* weight_max_vel_y

    最大y速度权重

* weight_max_vel_theta

    最大角速度权重

* weight_acc_lim_x

    最大x加速度权重

* weight_acc_lim_y

    最大y加速度权重

* weight_acc_lim_theta

    最大角速度权重

* weight_kinematics_nh

    非完整运动学约束权重

* weight_kinematics_forward_drive

    正向运动权重

* weight_kinematics_turning_radius

    最小化转弯半径权重,car like model

* weight_optimaltime

    优化轨迹时间的权重

* weight_obstacle

    满足与障碍物最小距离的权重

* weight_inflation

    膨胀后权重

* weight_dynamic_obstacle

    动态障碍物权重

* weight_viapoint

    最小与必经点距离权重

* weight_adapt_factor

    权重自曾参数,现在只有obstacle使用

* weight_prefer_rotdir

    转向权重












