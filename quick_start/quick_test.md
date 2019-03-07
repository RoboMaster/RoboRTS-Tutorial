# 快速测试

> [!Warning]
>
>在测试之前，参考[软件依赖配置](quick_start/setup_on_manifold2?id=软件依赖配置)与[RoboRTS下载与编译](quick_start/setup_on_manifold2?id=roborts下载与编译)，确保所有的软件依赖完成安装，roborts相关包顺利编译完成。

### 在仿真环境中测试

1. 运行Stage仿真环境以及定位与规划功能模块
```bash
    roslaunch roborts_bringup roborts_stage.launch
```

2. 运行决策测试节点
```bash
    rosrun roborts_decision behavior_test_node
```
输入1\2\3\4\5\6来切换不同行为

### 在实际环境中测试

#### 步骤1： 测试机器人驱动节点

1. 运行机器人驱动sdk的脚本
```
    roslaunch roborts_bringup base.launch
```
通过命令行来测试是否能获取信息以及基本的控制功能

> [!Tip]
>
>机器人驱动部分通过虚拟串口进行通信，因此需要外设端口映射配置，参考[外设端口映射配置](quick_start/setup_on_manifold2?id=外设端口映射配置)相关文档。

#### 步骤2： 执行简单的决策

1. 运行机器人驱动sdk、激光雷达、静态TF广播以及定位与规划功能模块
```
    roslaunch roborts_bringup roborts.launch
```

2. 运行决策测试节点
```bash
    rosrun roborts_decision behavior_test_node
```
输入1\2\3\4\5\6来切换不同行为

