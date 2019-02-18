# Global Path Planning

## Module Introduction

Global path planning (Referred to as global planning) is the first step of the motion planning of the Navigation System , after you set a target location, it will search a shortest path (a series of discrete coordinate points) without collision by perceptual global cost map , and then passed the path to the local trajectory planning module as a input to control the specific motion of the robot.



The module of global path planning is located in `roborts_planner`,  relevant action and mags are defined in the`roborts_msgs`and the  abstract factory pattern and parameter in the`roborts_common`. The directory of module files are as follows:

```bash
└── global_planner/
    ├── CMakeLists.txt
    ├── global_planner_algorithms.h #Contains header files for specific algorithms
    ├── global_planner_base.h       #The abstract class of global planning algorithm.
    ├── global_planner_test.cpp     #The test node of global planning.
    ├── global_planner_node.cpp     #The planner executive node with Main functions 
    ├── global_planner_node.h       
    ├── a_star_planner              #Astar global planning algorithm. 
    │   └── ...
    ├── config
    │   └── global_planner_config.prototxt # Parameter configuration file
    └── proto  
        ├── global_planner_config.pb.cc
        ├── global_planner_config.pb.h
        └── global_planner_config.proto    # Global planning parameter definition file.
```

Related algorithm of global path planning refers to [A Star Algorithm](en/sdk_docs/roborts_planning_global_planner?id=a) 

global_planner_node is the core planning node，ROS node graph is as follow:

![](https://rm-static.djicdn.com/documents/20758/63b3d8db24ce71547553505842780076.png)

The input and output of the nodes are as follows:

### Input

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  (Must)It is a base_link->map transformation that is listened to by `TransformListener`, provided by the `roborts_localization` Package within the roborts framework.

* global_costmap Object (`roborts_costmap/CostmapInterface`)

  It can represent the global  cost map (global_costmap) including the static and obstacle layers (optional), depending on the topic /tf, /map (static layer) and /scan (obstacle layer), which is provided by the `roborts_costmap` package.

* /global_planner_node_action/goal ([roborts_msgs/GlobalPlannerGoal]())

  (Must)Enter the target of the global planning from `Actionlib Client` to `Actionlib Server`. The interface is encapsulated by Actionlib. The specific message type is([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

* /global_planner_node_action/cancel ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

  Apply`Actionlib Client`to`Actionlib Server`to cancel the running task of the global planning. The interface is encapsulated by Actionlib.

### Output

* /global_planner_node_action/feedback ([roborts_msgs/GlobalPlannerFeedback]())

  `Actionlib Server`Real-time feedback of the planning path. The interface is encapsulated by Actionlib. Concrete message type is:  [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)

* /global_planner_node_action/result ([roborts_msgs/GlobalPlannerResult]())

  `Actionlib Server`Feedback of the planning results, which is used to judge whether it reaches the target or failed to get the feasible plan. The interface is encapsulated by Actionlib.

* /global_planner_node_action/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))  

  `Actionlib Server`Real-time feedback of the planning status. The interface is encapsulated by Actionlib.

* /global_planner_node/path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  

  It is used to visually display paths.

* /global_costmap/global_costmap/costmap ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))  

  It is used to visually display the global cost map.

### Related Parameters

Definition of the parameters refer to `proto/global_planner_config.proto`. Configuration of the parameters refer to `config/global_planner_config.prototxt`

* selected_algorithm(`string`, default: "a_star_planner")

    Select the name of global planning algorithm.

* frequency(`int`, default: 3)

    Frequency of global planning execution.

* max_retries(`int`, default: 5)

    max allowed times of retry after the global plan failed.   

* goal_distance_tolerance(`double`, default: 0.15)

    Euclidean distance tolerance on the goal point for the global planner.

* goal_angle_tolerance(`double`, default: 0.1)

    Angle (Unit radian) tolerance on the goal point for the global planner.

### Executing Process

In the core planning nodes`global_planner_node_action`, executing process is as follows:

- Initialization： 
  - Initialize Actionlib Server，create a visible publisher
  - Read the parameter.
  - Instantiate tf listener，global_costmap objects.
  - Instantiate specific algorithm planner object.

- After the initialization is finished, start the planning thread, ROS callback queue starts to callback in the main thread, at the meantime, Actionlib Server Callback also start to callback.
  - The flow diagram of the planning thread when it is executed:
    ![](https://rm-static.djicdn.com/documents/20758/057c895762b7d1547553536324774678.png)
  -  The flow diagram of Actionlib Server Callback thread:
    ![](https://rm-static.djicdn.com/documents/20758/b93803f9be2aa1547553557409469215.png)


## Compile and Run

### Compile

Compile in the working area of ROS:

```shell
catkin_make -j4 global_planner_node global_planner_test
```

### Run

Test in the simulation environment : 

```shell
roslaunch roborts_bringup roborts_stage.launch
```

Start the testing nodes of global path planning.

```shell
rosrun roborts_planning global_planner_test
```
Start Rviz，it display the input and output of modules that needed，as shown in the following figure:

![](https://rm-static.djicdn.com/documents/20758/844f7b73e9a091547553578138533412.png)

- Pose: Red arrow is the planning destination.
- LaserScan: Yellow point set is the data of laser radar scanning.
- TF: Refer to coordinate system of TF.
- Map: Gray-black part is the static map.
- Map: Different colors (purple, red, blue, blue, etc.) show global cost maps at different costs.
- Path: Green path is the planning path.

## A Star
### Algorithm Introduction

The A*(A-Star) algorithm is the most effective method for solving the shortest path in a static road network.The formula is expressed as：

$$ f(n) = g(n) + h(n) $$

$f(n)$ is the value function of node n from the initial point to the target point, $g(n)$ is the actual cost from the initial node to the n node in the state space, $h(n)$ is the cost from n to the target node of the best path . In the actual global path planning, the static network routing  is provided by the global cost map.

### Related Parameters
* inaccessible_cost(`int`, default: 253)

    The cost of map grid which is impassable(deadly obstacle).

* heuristic_factor(`double`, default: 1.0)

    Heuristic factor，refers to heuristic function: h(n) = heuristic_factor * manhattan_distance`

* goal_search_tolerance(`double`, default: 0.45)

    Search tolerating distance of the new destination(unit:meter) , if the destination you send is impassable , then you can find a new passable destination in the given allowing distance range.  


