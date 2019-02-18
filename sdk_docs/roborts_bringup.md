# 启动模块

## 模块介绍

启动模块主要包括基本的配置文件与启动脚本，位于roborts_bringup Package中，模块文件目录如下所示
```bash
roborts_bringup
├── CMakeLists.txt
├── launch                              #launch启动脚本
│   ├── base.launch                      #机器人驱动sdk启动脚本
│   ├── roborts.launch                  #除决策和调度模块的其他功能性节点测试脚本
│   ├── roborts_stage.launch            #对应上面的stage仿真测试脚本
│   ├── mapping.launch                  #建图的测试脚本
│   ├── mapping_stage.launch            #建图的stage仿真测试脚本
│   ├── slam_gmapping.xml               #gmapping建图节点启动脚本
│   └── static_tf.launch                #静态坐标变换脚本
├── maps                                #地图配置文件
│   ├── icra2018.pgm
│   ├── icra2018.yaml
│   ├── icra2019.pgm
│   └── icra2019.yaml
├── package.xml
├── rviz                                #rviz配置文件
│   ├── mapping.rviz
│   ├── roborts.rviz
│   └── teb_test.rviz
├── scripts                             
│   ├── udev                            #udev端口映射脚本
│   │   ├── create_udev_rules.sh        
│   │   ├── delete_udev_rules.sh
│   │   └── roborts.rules
│   └── upstart                         #开机自启脚本
│       ├── create_upstart_service.sh
│       ├── delete_upstart_service.sh
│       ├── jetson_clocks.sh
│       ├── max-performance.service
│       ├── max_performance.sh
│       ├── roborts.service
│       └── roborts-start.sh
└── worlds                              #比赛地图stage配置文件
    ├── icra2018.world                  
    └── icra2019.world                  

```

## 脚本

执行脚本添加udev映射规则：

```shell
./create_udev_rules.sh
```

执行脚本删除udev映射规则：
```shell
./delete_udev_rules.sh
```

其中，udev规则脚本详见`roborts.rules`文件，可根据所需外设进行灵活添加和修改

在Manifold2中，执行脚本添加开机启动服务：

```shell
./create_upstart_service.sh
```
执行脚本删除开机启动服务：

```shell
./delete_upstart_service.sh
```

其中，开机启动服务具体包括

- 开启max-performance.service服务来执行jetson_clocks.sh脚本与nvpmodel达到最大化的性能，

- 开启roborts.service服务执行roborts-start.sh脚本来运行ros launch脚本

用户可根据需求修改所需脚本文件和服务文件定制开机启动服务

## 测试与运行

在实际场景中，开启机器人驱动sdk脚本

```shell
roslaunch roborts_bringup base.launch
```

在实际场景中，执行除决策和调度模块的其他功能性节点测试脚本

```shell
roslaunch roborts_bringup roborts.launch
```

在stage仿真环境中，执行除决策和调度模块的其他功能性节点测试脚本

```shell
roslaunch roborts_bringup roborts_stage.launch
```

在实际场景中，执行gmapping建图的测试脚本

```shell
roslaunch roborts_bringup mapping.launch
```

在stage仿真环境中，执行gmapping建图的测试脚本

```shell
roslaunch roborts_bringup mapping_stage.launch
```

> [!Note]
>
> 注意，所有在实际场景中应用的launch脚本，默认不会启动rviz可视化界面。
