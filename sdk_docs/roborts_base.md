# 机器人驱动模块

## 模块介绍

机器人驱动模块是连接底层嵌入式控制板与上层机载电脑的桥梁，通过虚拟串口，基于一套通用的协议来定义双向传输的数据种类，进而完成数据互传的任务。

在机载端，`roborts_base`提供了ROS接口，接收底层嵌入式控制板发送的数据并控制其完成对机器人的模式切换和运动控制。

`roborts_base`依赖`roborts_msgs`中定义的相关消息类型。

模块文件目录如下所示
```bash
roborts_base
├── CMakeLists.txt
├── cmake_module
│   └── FindGlog.cmake
├── chassis                          #底盘模块ROS接口封装类
│   ├── chassis.cpp
│   └── chassis.h
├── gimbal                           #云台模块ROS接口封装类
│   ├── gimbal.cpp
│   └── gimbal.h
├── config
│   └── roborts_base_parameter.yaml  #参数配置文件
├── roborts_base_config.h            #参数读取类
├── roborts_base_node.cpp            #核心节点Main函数
├── ros_dep.h                        #包括所有协议对应ROS消息的头文件
├── roborts_sdk                     
│   ├── ...
└── package.xml
```
其中roborts_sdk文件内原生协议SDK，无任何ROS的依赖，详见
```bash
    ├── roborts_sdk
    │   ├── hardware                 #硬件层完成对协议数据的收发
    │   │   ├── hardware_interface.h #硬件层基类
    │   │   ├── serial_device.cpp    #串口设备实现类
    │   │   └── serial_device.h
    │   ├── protocol                 #协议层完成对协议的解包与打包
    │   │   ├── protocol.cpp         #协议层类
    │   │   ├── protocol_define.h    #协议具体定义消息类型的头文件
    │   │   └── protocol.h
    │   ├── dispatch                 #分发层完成对消息的分发
    │   │   ├── dispatch.h           #分发层类
    │   │   ├── execution.cpp        #分发层执行类
    │   │   ├── execution.h
    │   │   ├── handle.cpp           #roborts_sdk三层的对外接口类
    │   │   └── handle.h
    │   ├── sdk.h                    #roborts_sdk对外头文件
    │   ├── test
    │   │   └── sdk_test.cpp         #协议测试文件
    │   └── utilities
    │       ├── circular_buffer.h    #环形缓冲池
    │       ├── crc.h                #crc校验文件
    │       ├── log.h                #日志记录文件
    │       └── memory_pool.h        #内存池
```

>[!Note]
>协议详细信息请参考`RoboMaster AI机器人通信协议文档（TODO）`。


在该模块的核心运行节点`roborts_base_node`中，创建所需模块的对象（如底盘、云台）并初始化后，即可正常执行通信任务。

其ROS节点图示如下：

![](https://rm-static.djicdn.com/documents/20758/002d528eb36ad1550474043463957284.png)

### 底盘模块

#### 输入

* /cmd_vel ([geomtry_msgs/Twist]())

  底盘速度的控制量，即在下一个控制周期内，底盘做匀速运动

* /cmd_vel_acc ([roborts_msgs/TwistAccel]())

  底盘速度与加速度的控制量，即在下一个控制周期内，底盘以给定的速度做匀加速度运动


#### 输出

* /odom ([nav_msgs/Odometry]())

  底盘里程计信息

* /uwb ([geometry_msgs/PoseStamped]())

  底盘在UWB坐标系中的位姿信息

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  从base_link->odom的变换


### 云台模块

#### 输入

* /cmd_gimbal_angle ([roborts_msgs/GimbalAngle]())

  云台角度的控制量，根据相对角度标志位判断是绝对角度控制还是相对角度控制

* /cmd_gimbal_rate ([roborts_msgs/GimbalRate]())

  [**未启用**]云台角速度的控制量

* /set_gimbal_mode ([roborts_msgs/GimbalMode]())

  设定云台的模式

* /cmd_fric_wheel ([roborts_msgs/FricWhl]())

  开启与关闭摩擦轮（待开放发射速度控制）

* /cmd_shoot ([roborts_msgs/ShootCmd]())

  弹丸发射的控制指令（包括发射模式，频率与个数）

#### 输出

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))
  
  从base_link->gimbal的变换

### 相关参数

* serial_path(`string`, 默认值: "/dev/serial_sdk")

    串口的端口路径名称，默认值为udev rules设置的"/dev/serial_sdk"

## 编译与运行

### 编译

在ROS的工作区内编译

```shell
catkin_make -j4 roborts_base_node
```

### 运行

执行roborts_base_node节点

```shell
rosrun roborts_base roborts_base_node
```

或者启动相关launch文件

```shell
roslaunch roborts_bringup base.launch
```


