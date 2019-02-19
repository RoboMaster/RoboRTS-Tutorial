## 结构组成
RoboMaster机器人平台基于模块化设计，支持快拆结构，各模块可单独编程与调试使用。

<center class="half">
<img src="https://rm-static.djicdn.com/documents/20758/2f128eaf0f69e154755275553932690.png" width = 45% height = 45% /><img src="https://rm-static.djicdn.com/documents/20758/4c77c3e5940551547552816751545099.png" width = 30% height = 30% />
</center>

整个平台主要由三个部分组成

1. 底盘模块
<center>
<img src="https://rm-static.djicdn.com/documents/20758/6852f47fed9111547552846201962908.png" width = 50% height = 50% />
</center>

- 使用麦克纳姆轮，支持全向运动

- 动力系统由RoboMaster [M3508 P19 无刷直流减速电机](https://store.dji.com/product/rm-m3508-p19-brushless-dc-gear-motor)
 和RoboMaster [C620 电子调速器](https://store.dji.com/product/rm-c620-brushless-dc-motor-speed-controller)组成
- 使用RoboMaster [开发板](https://store.dji.com/product/rm-development-board-type-a?from=menu_products)[A型](https://store.dji.com/product/rm-development-board-type-a?from=menu_products) (STM32F427) 作为主控板


2. 云台模块 

<center>
<img src="https://rm-static.djicdn.com/documents/20758/f57a9c3c2a0cf1547552884336678019.jpeg" width = 40% height = 40% />
</center>


- 采用两轴云台支持2自由度的旋转运动

- 提供17mm TPU弹丸的拨弹与发射机构

- 云台动力系统由RoboMaster [GM6020 无刷电机](https://store.dji.com/cn/product/rm-gm6020-brushless-dc-motor) (包含电调) 组成

- 拨弹动力系统由[M2006 P36 无刷直流减速电机](https://store.dji.com/cn/product/rm-m2006-p36-brushless-motor) 组成

- 发射动力系统由 [DJI Snail 2305 竞速电机](https://store.dji.com/product/snail-racing-propulsion-system?from=menu_products) 组成
- 使用RoboMaster [Development Board Type A](https://store.dji.com/product/rm-development-board-type-a?from=menu_products) (STM32F427) 作为主控板

3. 裁判系统模块

<center>
<img src="https://rm-static.djicdn.com/documents/20758/70d4ea1ef88141547552911169893675.png" width = 40% height = 40% />
</center>

- 裁判系统是集成计算、通信、控制于一体的针对机器人比赛的电子判罚系统。开发者可以通过裁判系统，从特定的软件接口获取比赛进程和机器人状态的信息

- 裁判系统整体包含安装于机器人上的机载端以及安装在 PC 物理机上的服务器和客户端软件两部分。

- 机载端包含**主控模块**、**电源管理模块**、**装甲模块**、**测速模块**、**场地交互模块**等。

- 关于裁判系统模块更多信息请参考[相关资料](/resources)中的《裁判系统规范手册》。

除此之外，整个平台还包括**DT7遥控器**和智能锂电池 ([经纬 M100 TB47D电池](https://store.dji.com/product/matrice-100-tb47d-battery?from=autocomplete&position=0) 或 [TB48D](https://store.dji.com/product/matrice-100-tb48d-battery)) 及其充电器。关于机器人平台更多详细信息，请参考[相关文档](/documents)中的《AI机器人用户手册》。

整个平台可以容纳多种类型的传感器和计算设备，满足研究者定制化扩展开发的需求。整个平台提供了传感器安装架接口，适配多种类型的传感器包括单目相机、激光雷达、UWB定位套件、深度相机等。平台官方支持DJI Manifold 2作为机载端的计算设备，同时还兼容intel NUC和搭载相应尺寸扩展板的Nvidia Jetson TX1, TX2 或 Xavier设备。

## 硬件参数

| 结构                  |                       |
| :-------------------- | :-------------------- |
| 整机尺寸              | 600 x 450 x 460 mm    |
| 重量（含电池）        | 16.6 kg               |
| **性能**              |                       |
| 最大前进速度          | 3 m/s                 |
| 最大平移速度          | 2 m/s                 |
| 云台 Pitch轴 转动角度 | -20° ~ 20°            |
| 云台 Yaw轴 转动角度   | -90° ~ 90°            |
| 弹丸最大发射频率      | 10 发/秒              |
| 弹丸最大发射速度      | 25 m/s                |
| Remote Controller     | 200 发                |
| **电池**              |                       |
| 型号                  | DJI TB47D / DJI TB48D |
| 类型                  | LiPo 6s               |
| 电压                  | 22.8v                 |
| 电池容量              | 4500 mAH / 5700 mAH   |
| **遥控器**            |                       |
| 型号                  | DJI DT7               |
| Firmware upgrade      | 2.4 GHz               |
| 充电接口              | Micro USB             |
| **通信接口**          |                       |
| 接口类型              | Micro USB             |
| 通信方式              | STM32虚拟串口         |
| 波特率                | 921600                |


