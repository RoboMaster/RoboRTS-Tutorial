# Quick Test

> [!Warning]
>
>Before testing, Refer to [Software Dependency Configuration](en/quick_start/setup_on_manifold2?id=software-dependency-configuration) and [Roborts Download and Compile](en/quick_start/setup_on_manifold2?id=roborts-download-and-compile). Make sure software dependencies are installed, roborts packages are compiled successfully according to the guidelines before testing.

### Test in Simulator

1. Launch the script of Stage simulator, localization and motion planning modules
```bash
    roslaunch roborts_bringup roborts_stage.launch
```

2. Run behavior test node for simple decisions
```bash
    rosrun roborts_decision behavior_test_node
```
Input different digital command like 1/2/3/4/5/6 and you can switch to perform different behaviors

### Test in Real World

#### Step 1： Test Robot Driver

1. Launch the script of robot driver module
```
    roslaunch roborts_bringup base.launch
```
Check the functionality to get information from MCU and control the robot through the command line tools

> [!Tip]
>
>Robot driver module communicates with MCU through virtual serial, thus peripheral port mapping needs configuring correctly. Please refer to [Peripheral Port Mapping](en/quick_start/setup_on_manifold2?id=peripheral-port-mapping) for more information。

#### Step 2: Test Simple Decisions

1. Launch the script of robot driver, static TF broadcaster, lidar, localization and motion planning modules
```bash
    roslaunch roborts_bringup roborts.launch
```

2. Run behavior test node for simple decisions
```bash
    rosrun roborts_decision behavior_test_node
```
Input different digital command like 1/2/3/4/5/6 and you can switch to perform different behaviors

