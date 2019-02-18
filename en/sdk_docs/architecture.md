# Overall Architecture

## Architecture and Module Introduction
The overall system is structured with the loop of `sensor->perception->decision->planning->control->executor`, and the different modules are maintained in the form of ROS Package. The module and its data flow are shown in the figure below.
![](https://rm-static.djicdn.com/documents/20758/f42d65d85d97c1547553106539783606.png)

### Sensors, Controllers and Executers

- The central module integrates sensor modules (lidar, camera, IMU, etc.), embedded control platform (executing real-time tasks such as closed-loop control and data acquisition and pre-processing) and actuators (motors, etc.), and is responsible for sensing and control. The ROS Package for the embedded SDK is [roborts_base] (en/sdk_docs/roborts_base); for the camera is [roborts_camera](en/sdk_docs/roborts_camera). The system also includes the ROS packages for the related sensor drivers.

### Perception

Perception includes robot localization, map maintenance and representation, target detection and tracking, etc.

- localization module is responsible for robot localization. See [roborts_localization](en/sdk_docs/roborts_localization) for details.

- The map module is responsible for robot map maintenance, currently using ROS open source package [map_server](http://wiki.ros.org/map_server)

- The costmap module is responsible for the representation of the costmap. It integrates the static map layer, the obstacle layer and the inflation layer. It is mainly used in the motion planning part. Please see roborts_costmap for details. In the subsequent plans, this module will be updated to feature map module, not only for planning.

- The detection module is responsible for target detection and tracking. For details, see [roborts_detection](en/sdk_docs/roborts_detection). Currently, this module integrates the enemy detection and the projectile controller. Since the frame rate requirement is relatively high, the current detection and control are coupled, which will be decoupled later, putting the gimbal planner for tracking the target into the gimbal_executor.

### Task Scheduling and Decision Making

The task scheduling and decision making part includes the interface of the perception scheduler as input and the plan execution scheduler as output , and the core framework of the decision-making.
- The decision module is a robotic decision-making framework, and the Behavior Tree is officially provided. The blackboard module schedules the perception task acquisition information of various modules and the game information of the referee system. The behavior module integrates various actions or behaviors of the discrete action space. For details, see [roborts_decision](en/sdk_docs/roborts_decision)

- The executor module is a dependency of the behavior module, which contains the delegation interface of the chassis and the gimbal of different abstraction levels (e.g. scheduling the chassis to execute the result of the motion planning). See roborts_decision/executor for details.

### Motion Planning

The motion planning part is the motion planning function module, which is scheduled and delegated by the chassis_executor module in the decision part, completed in the roborts_planning part. See roborts_planning for details.

- The global_planner module is responsible for the global path planning of the robot. See [roborts_planning/global_planner](en/sdk_docs/roborts_planning_global_planner) for details. It depends on roborts_costmap

- The local_planner module is responsible for the local trajectory planning of the robot. See [roborts_planning/local_planner](en/sdk_docs/roborts_planning_local_planner) for details. It depends on roborts_costmap


## ROS Package Introduction

| Package               |  function           | internal dependency     |
| :--:                  | :------------: | :------: |
|  roborts              |  Meta-package  |   - |
|  roborts_base         | Embedded communication interface   | roborts_msgs |
|  roborts_camera       | camera driver | roborts_common |
|  roborts_common       |   common dependency   |    -    |
|  roborts_decision     |  robot decision package  | roborts_common<br/>roborts_msgs <br/>roborts_costmap |
|  roborts_detection    |  computer vision detection algorithm package  | roborts_msgs<br/>  roborts_common<br/>  roborts_camera |
|  roborts_localization |  robot localization algorithm package  |    -    |
|  roborts_costmap          |  costmap-related package  | roborts_common |
|  roborts_msgs         |  custom message definition package  |    -    |
|  roborts_planning     |  motion planning algorithm package  | roborts_common<br/>  roborts_msgs<br/>roborts_costmap |
|  roborts_bringup        |  launch package  | roborts_base<br/>  roborts_common<br/>  roborts_localization<br/>  roborts_costmap<br/>  roborts_msgs<br/>  roborts_planning<br/>  |
|  roborts_tracking     |  computer vision tracking algorithm package  | roborts_msgs |
