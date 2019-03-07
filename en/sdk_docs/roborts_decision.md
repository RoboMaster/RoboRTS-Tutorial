# Task Scheduling and Decision Module

## Module Introduction 

Task scheduling and decision module , provided perceptual input scheduling modules , planning the interface of executing the output scheduling module，and the core frame of the decision.

The module is located in `roborts_decision`，and depends on the module of parameter reading in  `roborts_common` , and the objects of cost maps in the `roborts_costmap` (selectable) as well as the related information types in the `roborts_msgs`. 

The module file directory is as follows:

```bash
roborts_decision/
├── behavior_test.cpp              #Test nodes of the behavior example
├── behavior_tree                  #Example of decision frame,behavior tree
│   ├── behavior_node.h            #Definiton of behavior tree's nodes
│   ├── behavior_state.h           #Definition of behavior tree's state 
│   └── behavior_tree.h            #Definition of behavior tree's runnig
├── blackboard                             
│   └── blackboard.h               #Definition of the blackboard(the input of decision                                     frame)
├── CMakeLists.txt         
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindProtoBuf.cmake
├── config                         
│   └── decision.prototxt          #Parameter configuration file of the behavior                                           example
├── example_behavior               #Behavior examples(the output of decision frame) 
│   ├── back_boot_area_behavior.h  #Definition of behavior of returning to the startup 									   area 
│   ├── chase_behavior.h           #Definition of behavior of Chasing the enemy
│   ├── escape_behavior.h          #Definition of executing escape behavior when 										obseving the enemy
│   ├── goal_behavior.h            #Definition of behavior of assigning target 											navigation
│   ├── line_iterator.h        
│   ├── patrol_behavior.h          #Definition of fixed patrol behavior
│   └── search_behavior.h          #Definition of searching behavior in the local 										disappearance area
├── executor                       #Scheduling of task executor(Mission commission of 									  different modules)
│   ├── chassis_executor.cpp      
│   ├── chassis_executor.h         #Definition of chassis task scheduling
│   ├── gimbal_executor.cpp
│   └── gimbal_executor.h          #Definition of gimbal task scheduling
├── package.xml
└── proto                          
    ├── decision.pb.cc
    ├── decision.pb.h
    └── decision.proto             #Parameter configuration file of the behavior                                           example

```
There is two core parts including decision and task scheduling：

### Decision Modules

Decision modules include several parts：

- Decision frame

    The decision-making framework takes the information of the observation as input and the action as the output to assist the robot in making decisions. The current official example framework is the behavior tree , more information please refer to:`roborts_decision/behavior_tree`

- Blackboard

    The blackboard is similar to the concept of Blackboard in game design. As an input to the observation in the current decision system, it is used to dispatch a series of perceptual tasks and obtain perceptual information. More details please refer to:`roborts_decision/blackboard/blackboard.h`, the user can complete the modification and improvement of the blackboard class according to the type of information acquired by the user.

- Behavior

    The specific behavior of the robot can be used as an action in the current decision system after different levels of abstraction. The framework provide a series of specific behavior examples. More details please refer to:`roborts_decision/example_behavior`，users can custom behavior according to samples.

### Task Scheduling Modules

The behavior-dependent task scheduling module is mainly responsible for the task delegation of each module, and the function execution module is scheduled to complete the specific task.

For each scheduling module, the core is:

- Task execution(specific task input)

- Updating of the task state(real-time feedback of the task)

- Cancel of task(State reset and related recycling after interruption)

Three call interfaces can basically perform scheduling for different tasks.

Task state including:

-  IDLE

-  RUNNING

-  SUCCESS

-  FAILURE

According to the main part of the robot module , it can be divided into :

- chassis scheduling modules

-  gimbal scheduling modules

#### Chassis Scheduling Modules

The chassis scheduling modules includes a task scheduling interface with different levels of abstraction for the chassis. The operation block diagram is as follows:

![](https://rm-static.djicdn.com/documents/20758/ae091269db41b1547553446982826082.png)

It includes three task modules:

- Motion planning control
  
    Input the target  goal ([geometry_msgs/PoseStamped]()) , delegate path plannning task and trajectory plannning task for robot chassis motion planning control

- Speed control

    Input the target speed twist  ([geometry_msgs/Twist]()) to control the robot chassis directly  moving at a constant speed.

- Speed and acceleration control

   Input target speed and acceleration  twist_accel ([roborts_msgs/TwistAccel]()) to control the robot chassis directly  performing a uniform acceleration motion at an initial given speed.

##### Examples of Navigation Task

As to the navigation task of motion planning control, it is actually a complex task in which multi-module nodes cooperate with each other. The chassis scheduling module delegates the planning task to the global planning module and the local planning module, and finally outputs the speed and acceleration control amount to the underlying main control board. The planning module also relies on real-time updated odometer information, location information and cost maps.

The block diagram of the navigation system in the actual scene and virtual environment is as follows:

![](https://rm-static.djicdn.com/documents/20758/1a8702f75a2361547553400516274462.png)

#### Gimbal scheduling Modules

The gimbal scheduling modules includes a task scheduling interface with different levels of abstraction for the gimbal. The operation block diagram is as follows:

![](https://rm-static.djicdn.com/documents/20758/5a50d15ba49371547553470292508751.png)

It includes two task modes:

- Angle control

    Input target angle ([roborts_msgs/GimbalAngle]())to control the angle of gimbal of the robot directly

- Rate control

    [**deprecated**] Input target rate ([roborts_msgs/GimbalRate]())to control the rate of gimbal of the robot directly




## Compile and Run

### Compile

Compile in the workspace of ROS

```shell
catkin_make -j4 behavior_test_node
```

###  Run


Test in the simulation environment

```shell
roslaunch roborts_bringup roborts_stage.launch
```

Start the testing nodes of behavior

```shell
rosrun roborts_decision behavior_test_node
```

Input different digital command like 1/2/3/4/5/6 and you can switch to perform different behaviors






