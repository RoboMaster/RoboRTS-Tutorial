# Detection Module

## Module Introduction

Decision module provides tools to detect robot armor in ICRA Robomaster 2019 AI Challenge, and it also includes a simple analysis of the projectile model. (for more information, see the PDF documentation [projectile model analysis](https://raw.githubusercontent.com/RoboMaster/RoboRTS-Tutorial/master/pdf/projectile_model.pdf))

This module is a submodule of `roborts_detection` and depends on the abstract factory pattern and parameter loading in module `roborts_common`. The module file directory is shown as below.

```bash
roborts_detection
├── package.xml
├── CMakeLists.txt
├── armor_detection                  # Algorithms for armor detection
│   ├── CMakeLists.txt
│   ├── config
│   │   └── armor_detection.prototxt # Config file for armor detecion parameters
│   ├── armor_detection_algorithms.h # Header file for armor detection algorithms (all algorithm's header file should be included here)
│   ├── armor_detection_base.h       # Base class of the armor detection class
│   ├── armor_detection_client.cpp   # Client of actionlib for armor detection, for development usages
│   ├── armor_detection_node.cpp     # ROS node for internal logic of armor detection
│   ├── armor_detection_node.h       # Header/Entry file for the armor detection node
│   ├── gimbal_control.cpp           # Calculate gimbal's pitch and yaw according to projectile model
│   ├── gimbal_control.h
│   └── proto
│       ├── armor_detection.pb.cc
│       ├── armor_detection.pb.h
│       └── armor_detection.proto    # Structure description file for parameters used by armor detection node
│   ├── constraint_set               # Armor detection algorithm, identifies armor using armor characteristics
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── constraint_set.prototxt # Adjustable armor detection parameters
│   │   ├── constraint_set.cpp
│   │   ├── constraint_set.h
│   │   └── proto
│   │       ├── constraint_set.pb.cc
│   │       ├── constraint_set.pb.h
│   │       └── constraint_set.proto    # Structure description file for parameters used by constraint set
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindProtoBuf.cmake
└── util
    ├── CMakeLists.txt
    └── cv_toolbox.h # Image data subscriber used by the detection node. It acts as a tool box for common image processing functions.
```

The ROS node graph of armor_detection_node can be shown as follows:

![](https://rm-static.djicdn.com/documents/20758/01dfe2ff9684a1547553225209707914.png)

Inputs and outputs of the node are as follows:

### Input

- /camera_name/image_raw ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))

  (requried) Obtained by subscribing to `roborts_camera`, used for armor detection.

- /camera_name/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html))

  (requried) Obtained by subscribing to `roborts_camera`, used to calculate PnP and get 3D coodinates of the target.

### Output

- /armor_detection_node_action/feedback (roborts_msgs/action/ArmorDetection)

  `Actionlib Server` responds in real-time with location data of detected armor; the interface is wrapped by Actionlib.

- /armor_detection_node_action/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatusArray.html))  

  `Actionlib Server` responds in real-time with states of the detection node;  the interface is wrapped by Actionlib.


- /cmd_gimbal_angle (roborts_msgs/msgs/GimbalAngle)  

  Publish gimbal control messages.

### Related Parameters

For definitions of parameters see `proto/armor_detection.proto`，for parameter config data see `config/armor_detection.prototxt`.

- name(`string`)

  Name of the armor detection algorithm

- selected_algorithm(`string`)

  Selected detection algorithm name

- undetected_armor_delay(`uint32`)

  Counter variable that keeps track of times when no armor is detected and data of the previous frame is published

- camera_name(`string`)

  Camera name, should be identical to camera_name in the config file under roborts_camera

- camera_gimbal_transform(`CameraGimbalTransform`) # composite data

  Transformation matrix for camera and gimbal, including the following parameters

  - offset_x(`float`) 

    Offset of camera and gimbal in x direction

  - offset_y

    Offset of camera and gimbal in y direction

  - offset_z

    offset of camera and gimbal in z direction

  - offset_pitch

    Offset of pitch angle of camera and gimbal

  - offset_yaw

    Offset of yaw angle of camera and gimbal

- projectile_model_info(`ProjectileModelInfo`) # composite data

  Projectile initial parameters, used for model analysis, including the following parameters

  - init_v(`float`)

    Initial velocity of the projectile

  - init_k(`float`)

    Air friction constant, for more details see projectile_model.pdf

## Compile and Run

### Compile

Compile inside a ros workspace

```shell
catkin_make -j4 armor_detection_client armor_detection_node
```

### Run

Run the armor detection node

```shell
rosrun roborts_detection armor_detection_node
```

In the actual debugging process, you may need to run the armor_detection_client node

```shell
rosrun roborts_detection armor_detection_client
```





