# Bringup Module

## Module Introduction

The bringup module mainly includes basic configuration files and startup scripts in the roborts_bringup Package. The module file directory is as follows.
```bash
roborts_bringup
├── CMakeLists.txt
├── launch                              #Launch startup script
│   ├── base.launch                     #Robot driver script
│   ├── roborts.launch                  #Functional script in the actual scene
│   ├── roborts_stage.launch            #Functional script in the stage simulation 
│   ├── mapping.launch                  #Mapping script in the actual scene
│   ├── mapping_stage.launch            #Mapping script in the stage simulation
│   ├── slam_gmapping.xml               #gmapping node script
│   └── static_tf.launch                #Static coordinate transformation script
├── maps                                #Map profile
│   ├── icra2018.pgm
│   ├── icra2018.yaml
│   ├── icra2019.pgm
│   └── icra2019.yaml
├── package.xml
├── rviz                                #rviz configuration file
│   ├── mapping.rviz
│   ├── roborts.rviz
│   └── teb_test.rviz
├── scripts                             
│   ├── udev                            #udev port mapping script
│   │   ├── create_udev_rules.sh        
│   │   ├── delete_udev_rules.sh
│   │   └── roborts.rules
│   └── upstart                         #startup script
│       ├── create_upstart_service.sh
│       ├── delete_upstart_service.sh
│       ├── jetson_clocks.sh
│       ├── max-performance.service
│       ├── max_performance.sh
│       ├── roborts.service
│       └── roborts-start.sh
└── worlds                              #stage configuration file
    ├── icra2018.world                  
    └── icra2019.world                  

```

## Script

Execute the script to add udev mapping  rules：

```shell
./create_udev_rules.sh
```

Execute the script to delete udev mapping  rules：
```shell
./delete_udev_rules.sh
```

The udev rule script is described in the `roborts.rules` file. According to the peripherals you need, you can add and modify it flexibly.

In Manifold2, execute the script to add a startup service：

```shell
./create_upstart_service.sh
```
Execute the script to delete the startup service：

```shell
./delete_upstart_service.sh
```

The startup service includes

- To execute the jetson_clock.sh script with the nvpmodel command to maximize performance.

- Start the roborts.service service and execute the roborts-start.sh script to run the ros launch script.

Users can modify the required script files and service files to customize the startup service according to their needs.

## Test and Run

In the actual scenario, run the robot driver script

```shell
roslaunch roborts_bringup base.launch
```

In the actual scenario, execute the script for functional nodes  in addition to the decision and scheduling module.

```shell
roslaunch roborts_bringup roborts.launch
```

In the stage simulation environment，execute the script for functional nodes  in addition to the decision and scheduling module.

```shell
roslaunch roborts_bringup roborts_stage.launch
```

In the actual scenario, execute the test script for gmapping

```shell
roslaunch roborts_bringup mapping.launch
```

In the stage simulation environment, execute the test script for gmapping

```shell
roslaunch roborts_bringup mapping_stage.launch
```

> [!Note]
>
> Pay attention, all launch scripts that are applied in the actual scenario will not launch the rviz node for visualization by default.
