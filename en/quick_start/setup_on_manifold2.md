This chapter focuses on the deployment and application of RoboRTS on Manifold 2.

> [!Tip]
>
> Manifold 2 is a microcomputer create by DJI for developers. The Manifold 2-G series is equipped with the NVIDIA Jetson TX2 module. The Manifold 2 is the Manifold 2-G series by default. This document also applies to the Nvidia Jetson TX2 Native Development Kit.

## Performance Configuration

### Turn on the maximum performance

In Manifold 2, you can use NVPModel to allocate the cpu core number and the maximum frequency of cpu and gpu. Manifold 2 default mode only opens 4 `CPU` cores , to get maximum performance, you need to use the `nvpmodel` command to change the configuration.

#### View and change mode

```bash
sudo nvpmodel -q --verbose # View current mode

sudo nvpmodel -m [mode] # For the introduction of each mode, please refer to the table in Appendix 1.
sudo nvpmodel -m0 # Recommended configuration
```
The mode corresponding to the number is as follows：



| MODE |   MODE NAME    | DENVER 2 | FREQUENCY | ARM A57 | FREQUENCY | GPU FREQUENCY |
| :--: | :------------: | :------: | :-------: | :-----: | :-------: | :-----------: |
|  0   |     Max-N      |    2     |  2.0 GHz  |    4    |  2.0 GHz  |   1.30 Ghz    |
|  1   |     Max-Q      |    0     |           |    4    |  1.2 GHz  |   0.85 Ghz    |
|  2   | Max-P Core-All |    2     |  1.4 GHz  |    4    |  1.4 GHz  |   1.12 Ghz    |
|  3   |   Max-P ARM    |    0     |           |    4    |  2.0 GHz  |   1.12 Ghz    |
|  4   |  Max-P Denver  |    2     |  2.0 GHz  |    0    |           |   1.12 Ghz    |

#### Turn on the maximum clock frequency

Manifold 2 will install this script in the Home directory. Run this script changing the clock frequency, mainly to maximize performance.

```bash
sudo ./jetson_clocks.sh
```

### Network speed test and connection

- Ethernet bandwidth and speed test on Manifold 2

  ``` bash
  sudo apt-get install iperf
  #start server
  sudo iperf -s
  #start client
  sudo iperf -c 192.168.1.xxx -i 5
  ```

- WiFi test on Manifold 2

  - Turn off wlan0 energy saving mode

    ```bash
    iw dev wlan0 set power_save off #- to disable power save and reduce ping latency.
    iw dev wlan0 get power_save  # will return current state.

    ```

  - View  RSSI of WiFi

    ```bash
    watch -n 0.1 iw dev wlan0 link
    ```

  > [!Tip]
  >
  > Refer to the [Script](en/sdk_docs/roborts_bringup?id=script) section of the `roborts_bringup` module documentation to enable maximum performance and turn off wlan0 power-saving mode by booting the service after downloading and compiling the roborts packages.


## Peripheral Port Mapping

  According to the hardware interface (serial port, USB or ACM), configure the udev file in /etc/udev/rules.d to implement the device binding of the STM32 device virtual serial port and lidar:

  First connect to the virtual serial port of the STM32 device. lsusb can view the IDs of Vendor and Product, then create and configure the /etc/udev/rules.d/roborts.rules file.

  ```bash
  KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="serial_sdk"

  ```
  The same applies to the laser scanner. Then reload and start the udev service, which may take effect after the device is re-plugged.

  ```bash
  sudo service udev reload
  sudo service udev restart
  ```

  It is a bit more troublesome to configure multiple cameras of the same model. Since the IDs of Vendor and Product are the same, you should check the specific characteristics of each camera.

  ```bash
  udevadm info --attribute-walk --name=/dev/video0
  ```

  Generally, you can use the serial port number as a property to distinguish each camera, for example：

  ```bash
  SUBSYSTEM=="usb", ATTR{serial}=="68974689267119892", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
  SUBSYSTEM=="usb", ATTR{serial}=="12345698798725654", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
  ```

If it is a cheap camera, the serial port number may be the same, you can configure it according to the physical port of the connected HUB (KERNEL or KERNELS binding), for example：

  ```bash
  SUBSYSTEM=="usb", KERNEL=="2-3", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera0"
  SUBSYSTEM=="usb", KERNEL=="2-4", ATTR{idVendor}=="1871", ATTR{idProduct}=="0101", SYMLINK+="camera1"
  ```

>[!Tip]
>
>You can refer to the [Script](en/sdk_docs/roborts_bringup?id=script) section of the `roborts_bringup` module documentation to execute the udev port mapping script after downloading and compiling the roborts packages.



## Software Dependency Configuration

### ROS (ros-kinetic-ros-base)

Manifold 2 installs ROS Kinetic by default. If you use other platforms, you can install ROS by referring to the [Installation Guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

> [!Note]
>
> Pay attention that whether to write `source /opt/ros/kinetic/setup.bash` into `.bashrc` or `.zshrc` to properly load ROS-related environment variables.

Third-party dependencies required to install ROS, as well as other dependencies such as `SuiteSparse`, `Glog`, `Protobuf`

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

### Other recommended software

- git
- cmake
- vim
- terminator
- htop

## RoboRTS Download and Compile


```bash
# Create a workspace folder
mkdir -p roborts_ws/src
# Switch to the src directory
cd roborts_ws/src
# Download RoboRTS source code
git clone https://github.com/RoboMaster/RoboRTS
# Compile source code
cd ..
catkin_make 
# Load environment variables
source devel/setup.bash
```

> [!Note]
>
> If you use `zsh`, be careful to source  `setup.zsh` instead of `setup.bash`.
