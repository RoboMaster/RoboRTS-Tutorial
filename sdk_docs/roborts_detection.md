# Detection模块

## 模块介绍

Detection模块为检测ICRA Robomaster 2019人工智能挑战赛机器人装甲模块, 同时也包含了对弹丸模型的简单分析(参见PDF文档[弹丸模型分析](https://raw.githubusercontent.com/RoboMaster/RoboRTS-Tutorial/master/pdf/projectile_model.pdf)).

模块位于`roborts_detection`包中, 依赖`roborts_common`包中抽象工场模式和参数读取, 模块文件目录如下所示.


```bash
roborts_detection
├── package.xml
├── CMakeLists.txt
├── armor_detection                  # 装甲识别算法
│   ├── CMakeLists.txt
│   ├── config
│   │   └── armor_detection.prototxt # 装甲识别参数配置文件
│   ├── armor_detection_algorithms.h # 装甲识别算法头文件（所有算法头文件都应在此文件中引入）
│   ├── armor_detection_base.h       # 装甲识别父类
│   ├── armor_detection_client.cpp   # 装甲识别actionlib中的client，在调试中使用。
│   ├── armor_detection_node.cpp     # ros node，负责装甲识别内部逻辑调度
│   ├── armor_detection_node.h       # 装甲识别入口文件
│   ├── gimbal_control.cpp           # 云台控制文件，以弹丸模型建立，得出云台控制的pitch以及yaw
│   ├── gimbal_control.h
│   └── proto
│       ├── armor_detection.pb.cc
│       ├── armor_detection.pb.h
│       └── armor_detection.proto    # 装甲识别参数生成文件
│   ├── constraint_set               # 装甲识别算法，使用装甲特征识别装甲板
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── constraint_set.prototxt # 装甲识别可调参数
│   │   ├── constraint_set.cpp
│   │   ├── constraint_set.h
│   │   └── proto
│   │       ├── constraint_set.pb.cc
│   │       ├── constraint_set.pb.h
│   │       └── constraint_set.proto    # 装甲识别参数生成文件
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindProtoBuf.cmake
└── util
    ├── CMakeLists.txt
    └── cv_toolbox.h # 负责订阅图片以整个detection使用，同时也包含通用图像处理函数
```

armor_detection_node 的ROS节点图示为

![](https://rm-static.djicdn.com/documents/20758/01dfe2ff9684a1547553225209707914.png)

节点的输入输出如下

### 输入

- /camera_name/image_raw ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))

  (必须) 通过订阅roborts_camera得到, 用于装甲识别

- /camera_name/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html))

  (必须) 通过订阅roborts_camera得到, 用于PnP结算, 得出目标三维坐标

### 输出

- /armor_detection_node_action/feedback (roborts_msgs/action/ArmorDetection)

  `Actionlib Server`实时反馈识别到装甲的位置信息，接口由Actionlib封装

- /armor_detection_node_action/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatusArray.html))  

  `Actionlib Server`实时反馈装甲识别节点运行状态，接口由Actionlib封装

- /cmd_gimbal_angle (roborts_msgs/msgs/GimbalAngle)  

  发布云台控制信息

### 相关参数

参数的定义见`proto/armor_detection.proto`，参数的配置见`config/armor_detection.prototxt`

- name(`string`)

  装甲板识别算法的名称

- selected_algorithm(`string`)

  选择装甲板识别算法的名称

- undetected_armor_delay(`uint32`)

  未检测到装甲板时, 继续发布上一帧数据的次数 

- camera_name(`string`)

  相机名称, 应该和roborts_camera配置文件中的camera_name相同

- camera_gimbal_transform(`CameraGimbalTransform`) # 复合数据

  相机与云台的变换矩阵包括了一下参数

  - offset_x(`float`) 

    相机与云台x方向偏移

  - offset_y

    相机与云台y方向偏移

  - offset_z

    相机与云台z方向偏移

  - offset_pitch

    相机与云台pitch角偏移

  - offset_yaw

    相机与云台yaw角偏移

- projectile_model_info(`ProjectileModelInfo`) # 复合数据

  弹丸初始参数, 用于弹丸模型分析, 包括一下参数

  - init_v(`float`)

    弹丸出射速度

  - init_k(`float`)

    空气阻力常数, 具体参考projectile_model.pdf

## 编译与运行

### 编译

在ROS的工作区内编译

```shell
catkin_make -j4 armor_detection_client armor_detection_node
```

### 运行

启动装甲板识别节点

```shell
rosrun roborts_detection armor_detection_node
```

实际调试过程中,需启动armor_detection_client节点

```shell
rosrun roborts_detection armor_detection_client
```





