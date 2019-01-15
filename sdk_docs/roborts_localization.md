# 定位模块

## 模块介绍

定位模块是导航系统中强依赖的节点，其主要通过获得一系列的传感器信息，通过特定算法的处理，最终得到机器人坐标系到地图坐标系的变换关系，也就是机器人在地图中的位姿。在`roborts_localization`中，默认的定位算法是AMCL算法，关于AMCL算法的介绍和参数说明可以参考[AMCL](sdk_docs/roborts_localization?id=amcl算法)章节。

数据流示意图如下：

![localization](https://rm-static.djicdn.com/documents/20758/0ba7ea27fc531547553277041255026.png)



文件目录如下：

```bash
├── CMakeLists.txt
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindGlog.cmake
├── config
│   └── localization.yaml  #Localization参数配置文件，需要在launch file中load
├── localization_config.h  #Localization参数类
├── localization_math.cpp  
├── localization_math.h    #模块内通用数学函数
├── localization_node.cpp  #主节点和Main函数
├── localization_node.h    
├── log.h                  #Glog Wrapper
├── package.xml
├── types.h                #Type define
├── amcl                   #AMCL算法代码目录
│   ├── amcl_config.h      #AMCL参数类
│   ├── amcl.cpp           #AMCL主要逻辑代码
│   ├── amcl.h
│   ├── CMakeLists.txt    
│   ├── config
│   │   └── amcl.yaml      #AMCL参数配置文件，需要在launch file中load
│   ├── map
│   │   ├── amcl_map.cpp   
│   │   └── amcl_map.h     #AMCL地图运算相关代码
│   ├── particle_filter    #粒子滤波器相关代码
│   │   ├── particle_filter.cpp
│   │   ├── particle_filter_gaussian_pdf.cpp
│   │   ├── particle_filter_gaussian_pdf.h
│   │   ├── particle_filter.h
│   │   ├── particle_filter_kdtree.cpp
│   │   ├── particle_filter_kdtree.h
│   │   └── particle_filter_sample.h
│   └── sensors            #里程计模型与激光雷达传感器模型
│       ├── sensor_laser.cpp
│       ├── sensor_laser.h
│       ├── sensor_odom.cpp
│       └── sensor_odom.h
```



定位节点可以从命令行独立启动

```bash
# 编译roborts_localization
catkin_make -DCMAKE_BUILD_TYPE=Release localization_node

rosrun roborts_localization localization_node
```

或从launch文件启动

```xml
<!-- Load parameters -->
<rosparam command="load" file="$(find roborts_localization)/config/localization.yaml" />
<rosparam command="load" file="$(find roborts_localization)/amcl/config/amcl.yaml" />

<!-- Run the Localization Node -->
<node pkg="roborts_localization" type="localization_node" name="localization_node" respawn="false" />
```

RViz中的测试效果

![localization_rviz](https://rm-static.djicdn.com/documents/20758/f172f1f29aa2f1547553303273821911.png)



## 模块输入

* /map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

  (必须) 订阅静态地图。

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  (必须) 获取坐标系间转换关系。（odom->bask_link）

* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))

  (必须) 订阅激光雷达数据。

* /initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

  (可选) 估计位姿，以此均值和方差初始化粒子滤波器。对应Rviz中的2D Pose Estimate。

* /uwb ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html))

  (可选) UWB数据，用于校正全局定位信息。

## 模块输出

* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

  通过AMCL算法估计的位姿。

* /particlecloud ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

  粒子滤波器中粒子的位姿。

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  发布`odom`系到`map`系的转换关系。



> [!NOTE]
> 在运行`roborts_localization`模块前需检查roborts_base和激光雷达相关驱动是否已正确运行。



## 相关参数

* odom_frame_id (`string`, 默认值: "odom")

  odom的坐标系。

* base_frame_id (`string`, 默认值: "base_link")

  机体坐标系。

* global_frame_id (`string`, 默认值: "map")

  地图坐标系。

* laser_topic_name (`string`, 默认值: "scan")

  激光雷达发送的topic名。

* map_topic_name (`string`, 默认值: "map")

  静态地图发送的topic名。

* init_pose_topic_name (`string`, 默认值: "initialpose")

  指定估计位姿发送的topic名。

* transform_tolerance (`double`, 默认值: 0.1)

  tf发布间隔。

* initial_pose_x (`double`, 默认值: 1)

  初始估计位姿的x轴位置。

* initial_pose_y (`double`, 默认值: 1)

  初始估计位姿的y轴位置。

* initial_pose_a (`double`, 默认值: 0)

  初始估计位姿的yaw角度。

* initial_cov_xx (`double`, 默认值: 0.1)

  初始估计位姿的xx协方差。

* initial_cov_yy (`double`, 默认值: 0.1)

  初始估计位姿的yy协方差。

* initial_cov_aa (`double`, 默认值: 0.1)

  初始估计位姿的aa协方差。

* enable_uwb (`bool`, 默认值: true)

  是否启用UWB校正。

* uwb_frame_id (`string`, 默认值: "uwb")

  UWB坐标系。

* uwb_topic_name (`string`, 默认值: "uwb")

  UWB发布的topic名。

* use_sim_uwb (`bool`, 默认值: false）

  是否使用stage模拟器生成fake uwb信息。

* uwb_correction_frequency (`int`, 默认值: 20)

  UWB校正的频率。


## AMCL算法

### 算法介绍

![autonomous mobile robot](https://rm-static.djicdn.com/documents/20758/6b0ea21b6d57a1547553332830529202.png)

自适应蒙特卡洛定位( AMCL, Adaptive Monte Carlo Localization)是一套适用于二维机器人运动的定位算法。其主要的算法原理源于《Probabilistic Robotics》一书8.3中描述的**MCL**, **Augmented_MCL**, 和**KLD_Sampling_MCL**算法的结合与实现。同时书中第5章描述的Motion Model和第6章描述的Sensor Model，用于AMCL中运动的预测和粒子权重的更新。

在`roborts_localization`内部集成的AMCL，功能上添加了随机角度、随机位置和角度的初始化支持，以便于竞技比赛中的快速部署。



算法基本流程如下图![AMCL_flowchart](https://rm-static.djicdn.com/documents/20758/e75d3533fc03c1547553357220164263.png)



### 相关参数

* min_particles (int, 默认值:  50)

  粒子滤波器的最小粒子数。

* max_particles (int, 默认值:  2000)

  粒子滤波器的最大粒子数。

* use_global_localization (bool, 默认值:  false)

  是否初始随机初始定位。

* random_heading (bool, 默认值:  false)

  是否初始随机角度的初始定位。

* update_min_d (double, 默认值:  0.1)

  滤波器更新的位移阈值。

* update_min_a (double, 默认值:  0.5)

  滤波器更新的旋转阈值。

* odom_model (ENUM, ODOM_MODEL_OMNI)

  机器人运动模型。目前只支持全向轮里程计模型。

* odom_alpha1 (double, 默认值:  0.005)

  里程计模型的误差参数。

* odom_alpha2 (double, 默认值:  0.005)

  里程计模型的误差参数。

* odom_alpha3 (double, 默认值:  0.01)

  里程计模型的误差参数。

* odom_alpha4 (double, 默认值:  0.005)

  里程计模型的误差参数。

* odom_alpha5 (double, 默认值:  0.003)

  里程计模型的误差参数。

* laser_min_range (double, 默认值:  0.15)

  激光雷达的最小有效测距距离。

* laser_max_range (double, 默认值:  8.0)

  激光雷达的最大有效测距距离。

* laser_max_beams (int, 默认值:  30)

  激光雷达的波束数。

* laser_model (ENUM, 默认值:  LASER_MODEL_LIKELIHOOD_FIELD_PROB)

  激光雷达测距仪模型。目前只支持似然域改进模型。

* z_hit (double, 默认值:  0.5)

  似然域模型中的$z_{hit}$参数。

* z_rand (double, 默认值:  0.5)

  似然域模型中的$z_{rand}$参数。

* sigma_hit (double, 默认值:  0.2)

  似然域模型中的$\sigma_{hit}$参数。

* laser_likelihood_max_dist (double, 默认值:  2.0)

  似然域模型中的测距点与障碍物间最大距离。

* do_beamskip (bool, 默认值:  true)

  Position Tracking阶段忽略部分激光波束，以避免不可预料的误差，比如动态物体等等。

* beam_skip_distance (double, 默认值:  0.5)

  忽略波束的障碍物距离阈值。

* beam_skip_threshold (double, 默认值:  0.3)

  波束忽略的阈值。

* beam_skip_error_threshold (double, 默认值:  0.9)

  波束忽略的错误阈值。

* resample_interval (int, 默认值:  2)

  重采样周期。

* recovery_alpha_slow (double, 默认值:  0.001)

  **Augmented_MCL**中的$\alpha_{slow}$参数。

* recovery_alpha_fast (double, 默认值:  0.1)

  **Augmented_MCL**中的$\alpha_{fast}$参数。

* kld_err (double, 默认值:  0.05)

  **KLD_Sampling_MCL**中的$\epsilon$。

* kld_z (double, 默认值:  0.99)

  **KLD_Sampling_MCL**中的$(1-\delta)$。

* laser_filter_weight (double, 默认值:  0.4)

  权重阈值，用于筛选出权重较低的激光雷达测量值。

* max_uwb_particles (int, 默认值:  10)

  重采样阶段，以UWB为均值的最大重采样数。

* uwb_cov_x (double, 默认值:  0.06)

  重采样阶段，以UWB为均值的高斯分布的方差x

* uwb_cov_y (double, 默认值:  0.06)

  重采样阶段，以UWB为均值的高斯分布的方差y

* resample_uwb_factor (double, 默认值:  4.0)

  重采样因子，用于判断对称定位。





