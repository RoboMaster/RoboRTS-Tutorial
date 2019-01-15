# 相机驱动模块

## 模块介绍

相机驱动模块封装了常见相机驱动，并通过ROS的`image_transport`发布图像数据和相机参数。

相机模块位于roborts_camera Package中，依赖roborts_common Package中的抽象工厂模式和参数读取，模块文件目录如下所示

```bash
roborts_camera
├── CMakeLists.txt
├── package.xml
├── cmake_module
│   └── FindProtoBuf.cmake
├── camera_base.h             #相机的抽象类
├── camera_node.cpp           #相机核心运行节点和Main函数
├── camera_node.h
├── camera_param.cpp          #相机参数读取类
├── camera_param.h
├── config
│   └── camera_param.prototxt #相机参数配置文件
├── proto
│   ├── camera_param.pb.cc
│   ├── camera_param.pb.h
│   └── camera_param.proto    #相机参数定义文件
├── test
│   └── image_capture.cpp     #相机数据和参数接收测试节点
└── uvc                       
    ├── CMakeLists.txt
    ├── uvc_driver.cpp        #uvc相机类
    └── uvc_driver.h
```

相机运行节点`roborts_camera_node`，通过读取一个或多个相机的配置参数，可自动调度相机来发布图像数据。

### 相关参数
参数的定义见`proto/camera_param.proto`，参数的配置见`config/camera_param.prototxt`，接受多个相机配置，单个相机参数包括

* camera_name (`string`)
  
  相机名称，用于发布相机消息的namespace

* camera_type (`string`)
  
  相机类型，用于抽象工厂模式中注册和实例化相机对象的键值,例如uvc相机为"uvc"

* camera_path (`string`)
  
  相机路径，用于打开相机的端口路径名称

* camera_matrix (`double[]`)

  相机内参矩阵，一般为3X3

* camera_distortion (`double[]`)

  相机畸变矩阵

* fps (`uint32`)

  相机帧率（单位frame per second）

* resolution

    * width (`uint32`) 

      分辨率宽度（单位像素）

    * height (`uint32`)

      分辨率长度（单位像素）

    * width_offset (`uint32`)

      分辨率宽度偏移值（单位像素），用于硬件采集图像帧时做的Crop偏移

    * height_offset (`uint32`)

      分辨率长度偏移值（单位像素），用于硬件采集图像帧时做的Crop偏移

* auto_exposure (`bool`)

  自动曝光

* exposure_value (`uint32`)

  曝光值

* exposure_time (`uint32`)

  曝光时间（单位us）

* auto_white_balance (`bool`)

  自动白平衡

* white_balance_value (`uint32`)

  白平衡值

* auto_gain (`bool`)

  自动增益

* gain_value (`uint32`)

  增益值

* contrast (`uint32`)

  对比度

### 输出

/camera_name/camera_info ([sensor_msgs/CameraInfo]())

 相机参数信息

/camera_name/image_raw ([sensor_msgs/Image]())

 相机原生图像数据


## 编译与运行

### 编译

在ROS的工作区内编译

```shell
catkin_make -j4 roborts_camera_node
```

### 运行

启动相机节点

```shell
rosrun roborts_camera roborts_camera_node
```

开启rqt测试发布图像

```shell
rqt_image_view
```
