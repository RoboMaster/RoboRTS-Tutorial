# Robot Driver Module

## Module introduction

The robot Driver Module is a bridge between the underlying embedded control board to the upper onboard computer，through the virtual serial port, the data type of bidirectional transmission is defined based on a common set of protocols, thereby completing the task of data mutual transmission.

In  computing device，`roborts_base `has a ROS interface which receives data sent by the underlying embedded control board and sends data to complete mode switching and motion control of the robot.

`roborts_base`depends on the relevant message types defined in`roborts_msgs`.

The module file directory is as follows
```bash
roborts_base
├── CMakeLists.txt
├── cmake_module
│   └── FindGlog.cmake
├── chassis                          #Chassis module ROS interface package
│   ├── chassis.cpp
│   └── chassis.h
├── gimbal                           #Gimbal module ROS interface package
│   ├── gimbal.cpp
│   └── gimbal.h
├── config
│   └── roborts_base_parameter.yaml  #Parameter configuration file
├── roborts_base_config.h            #Parameter reading class
├── roborts_base_node.cpp            #Core node Main function
├── ros_dep.h                        #Includes header files for all protocols corresponding to ROS messages
├── roborts_sdk                     
│   ├── ...
└── package.xml
```
The Original protocol  SDK in the roborts_sdk file，without any ROS dependencies, as follows
```bash
    ├── roborts_sdk
    │   ├── hardware                 #Hardware layer to raw data transmission
    │   │   ├── hardware_interface.h #Hardware layer base class
    │   │   ├── serial_device.cpp    #Serial device implementation
    │   │   └── serial_device.h
    │   ├── protocol                 #Protocol layer to unpack and pack the message 
    │   │   ├── protocol.cpp         #Protocol layer
    │   │   ├── protocol_define.h    #Protocol message type definition header file  
    │   │   └── protocol.h
    │   ├── dispatch                 #The dispatch layer to dispatch message
    │   │   ├── dispatch.h           #Dispatch layer
    │   │   ├── execution.cpp        #Dispatch layer execution
    │   │   ├── execution.h
    │   │   ├── handle.cpp           #roborts_sdk three-layer external interface
    │   │   └── handle.h
    │   ├── sdk.h                    #roborts_sdk external document
    │   ├── test
    │   │   └── sdk_test.cpp         #Protocol test file
    │   └── utilities
    │       ├── circular_buffer.h    #Ring buffer pool
    │       ├── crc.h                #crc check file
    │       ├── log.h                #Log file
    │       └── memory_pool.h        #Memory pool
```

>[!Note]
>For details of the agreement, please refer to `RoboMaster AI Robot communication protocol document（TODO）`.


In the core running node`roborts_base_node` of the module，After the objects of the required modules (such as the chassis and the gimbal) are created and initialized, the communication task can be performed normally.

Its ROS node is shown as follows：

![](https://rm-static.djicdn.com/documents/20758/002d528eb36ad1550474043463957284.png)

### Chassis Module

#### Input

* /cmd_vel ([geomtry_msgs/Twist]())

  Chassis speed control, In the next control cycle, the chassis is moving at a constant speed.

* /cmd_vel_acc ([roborts_msgs/TwistAccel]())

  Chassis speed and acceleration control，During the next control cycle, the chassis performs a uniform acceleration motion at an initial given speed.


#### Output

* /odom ([nav_msgs/Odometry]())

  Chassis odometry information

* /uwb ([geometry_msgs/PoseStamped]())

  Pose information of the chassis in the UWB coordinate system

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  From base_link->odom


### Gimbal Module

#### Input

* /cmd_gimbal_angle ([roborts_msgs/GimbalAngle]())

  Control of the angle of the gimba. Judging whether it is absolute angle control or relative angle control according to the relative angle mark

* /cmd_gimbal_rate ([roborts_msgs/GimbalRate]())

  [**deprecated**] Gimbal rate control 

* /set_gimbal_mode ([roborts_msgs/GimbalMode]())

  Set the mode of the gimbal

* /cmd_fric_wheel ([roborts_msgs/FricWhl]())

  Open and close the friction wheel (to be opened to launch speed control)

* /cmd_shoot ([roborts_msgs/ShootCmd]())

  Control instructions for projectile launch (including launch mode, frequency and number)

#### Output

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))
  
  From base_link->gimbal

### Related Parameters

* serial_path(`string`, Defaults: "/dev/serial_sdk")

    Serial port path name，The default value is "/dev/serial_sdk" set by udev rules

## Compile and Run

### Compile

Compile in the ROS workspace

```shell
catkin_make -j4 roborts_base_node
```

### Run

Execute the roborts_base_node 

```shell
rosrun roborts_base roborts_base_node
```

Or start the relevant launch file

```shell
roslaunch roborts_bringup base.launch
```


