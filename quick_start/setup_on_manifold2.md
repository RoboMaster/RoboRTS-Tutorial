本章节主要介绍RoboRTS在Manifold 2上的部署与应用。

> [!Tip]
>
> Manifold 2 是 DJI 为开发者打造的微型计算机，其中Manifold 2-G 系列搭载 NVIDIA Jetson TX2 模块，文中默认Manifold 2均为Manifold 2-G系列产品。本文档同样适用于Nvidia Jetson TX2原生开发套件。

## Manifold 2性能配置

### 开启Manifold 2最大性能

在Manifold 2中，可以使用NVPModel分配cpu核数，cpu和gpu的最大频率。Manifold 2默认模式只打开4个`CPU`核, 为获得最大运算性能，需要使用`nvpmodel`命令来更改配置。

#### 查看与更改模式：

```bash
sudo nvpmodel -q --verbose # 查看当前模式

sudo nvpmodel -m [mode] # 各模式介绍可以参考附录1中表格
sudo nvpmodel -m0 # 推荐配置
```
数字对应的模式如下：



| MODE |   MODE NAME    | DENVER 2 | FREQUENCY | ARM A57 | FREQUENCY | GPU FREQUENCY |
| :--: | :------------: | :------: | :-------: | :-----: | :-------: | :-----------: |
|  0   |     Max-N      |    2     |  2.0 GHz  |    4    |  2.0 GHz  |   1.30 Ghz    |
|  1   |     Max-Q      |    0     |           |    4    |  1.2 GHz  |   0.85 Ghz    |
|  2   | Max-P Core-All |    2     |  1.4 GHz  |    4    |  1.4 GHz  |   1.12 Ghz    |
|  3   |   Max-P ARM    |    0     |           |    4    |  2.0 GHz  |   1.12 Ghz    |
|  4   |  Max-P Denver  |    2     |  2.0 GHz  |    0    |           |   1.12 Ghz    |

#### 开启最大时钟频率:

Manifold 2在Home目录下会安装此脚本，更改时钟频率脚本，主要为了开启最大性能。

```bash
sudo ./jetson_clocks.sh
```

### 网速测试和连接

- Manifold 2上以太网带宽和速度测试

  ``` bash
  sudo apt-get install iperf
  #start server
  sudo iperf -s
  #start client
  sudo iperf -c 192.168.1.xxx -i 5
  ```

- Manifold 2上WiFi测试

  - 关闭wlan0节能模式

    ```bash
    iw dev wlan0 set power_save off #- to disable power save and reduce ping latency.
    iw dev wlan0 get power_save  # will return current state.

    ```

  - 查看WiFi的RSSI

    ```bash
    watch -n 0.1 iw dev wlan0 link
    ```

  > [!Tip]
  >
  > 在下载和编译roborts包后，可参考`roborts_bringup`模块文档中的[脚本](sdk_docs/roborts_bringup?id=脚本)小节，通过开机自启服务，开启最大性能同时关闭wlan0节能模式。


## 外设端口映射配置

  根据硬件接口（串口，USB或者ACM）来配置/etc/udev/rules.d中的udev文件，分别实现STM32设备虚拟串口和激光雷达的设备绑定：

  首先连接STM32设备的虚拟串口，lsusb可以查看Vendor和Product的ID，然后创建并配置/etc/udev/rules.d/roborts.rules文件

  ```bash
  KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="serial_sdk"

  ```
  同理配置激光雷达。然后重新加载并启动udev服务，可能需要重新插拔设备后生效。

  ```bash
  sudo service udev reload
  sudo service udev restart
  ```

  而配置多个相同型号的相机的会麻烦一些，由于Vendor和Product的ID是一样的，因此要查看每个相机具体的特性

  ```bash
  udevadm info --attribute-walk --name=/dev/video0
  ```

  一般可以用串口号不同（serial）作为属性来区分每个相机，例如：

  ```bash
  SUBSYSTEM=="usb", ATTR{serial}=="68974689267119892", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
  SUBSYSTEM=="usb", ATTR{serial}=="12345698798725654", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
  ```

  如果是廉价相机，可能串口号也相同，可以由连接HUB的物理端口不同（KERNEL或KERNELS绑定）来配置，例如：

  ```bash
  SUBSYSTEM=="usb", KERNEL=="2-3", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
  SUBSYSTEM=="usb", KERNEL=="2-4", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
  ```

>[!Tip]
>
>在下载和编译roborts包后，可参考`roborts_bringup`模块文档中的[脚本](sdk_docs/roborts_bringup?id=脚本)小节，执行udev端口映射脚本。



## 软件依赖配置

### ROS (ros-kinetic-ros-base)

Manifold 2默认安装ROS Kinetic。若使用其它平台，可参考ROS Wiki的[安装教程](http://wiki.ros.org/kinetic/Installation/Ubuntu)安装ROS。

> [!Note]
>
> 注意是否将`source /opt/ros/kinetic/setup.bash`写入`.bashrc`或`.zshrc`，以正确加载ROS相关环境变量。

安装ROS所需第三方依赖包，以及`SuiteSparse`，`Glog`，`Protobuf`等其他依赖

```bash
sudo apt-get install -y ros-kinetic-opencv3             \
                        ros-kinetic-cv-bridge           \
                        ros-kinetic-image-transport     \
                        ros-kinetic-stage-ros           \
                        ros-kinetic-map-server          \
                        ros-kinetic-laser-geometry      \
                        ros-kinetic-interactive-markers \
                        ros-kinetic-tf                  \
                        ros-kinetic-pcl-*               \
                        ros-kinetic-libg2o              \
                        ros-kinetic-rplidar-ros         \
                        ros-kinetic-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \
                        libgoogle-glog-dev              \
```

### 其它常用软件

- git
- cmake
- vim
- terminator
- htop

## RoboRTS下载与编译


```bash
# 创建工作空间文件夹
mkdir -p roborts_ws/src
# 切换至src目录
cd roborts_ws/src
# 下载RoboRTS源码
git clone https://github.com/RoboMaster/RoboRTS
# 编译源码
cd ..
catkin_make 
# 加载环境变量
source devel/setup.bash
```

> [!Note]
>
> 如果使用`zsh`，注意将`setup.bash`替换为`setup.zsh`。
