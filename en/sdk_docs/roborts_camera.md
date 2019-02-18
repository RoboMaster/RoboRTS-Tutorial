# Camera driver module

## Module Introduction

The camera driver module encapsulates common camera drivers and publishes image data and camera parameters via ROS `image_transport`.

The camera module is located in the roborts_camera Package and depends on the abstract factory pattern and parameter reading in the roborts_common Package. The module file directory is as follows

```bash
roborts_camera
├── CMakeLists.txt
├── package.xml
├── cmake_module
│   └── FindProtoBuf.cmake
├── camera_base.h             #Camera abstract
├── camera_node.cpp           #Camera core running node and Main function
├── camera_node.h
├── camera_param.cpp          #Camera parameter reading
├── camera_param.h
├── config
│   └── camera_param.prototxt #Camera parameter profile
├── proto
│   ├── camera_param.pb.cc
│   ├── camera_param.pb.h
│   └── camera_param.proto    #Camera parameter definition file
├── test
│   └── image_capture.cpp     #Camera test node for image capture
└── uvc                       
    ├── CMakeLists.txt
    ├── uvc_driver.cpp        #uvc camera
    └── uvc_driver.h
```

The camera runs the node`roborts_camera_node`, which automatically dispatches the camera to publish image data by reading the configuration parameters of one or more cameras.

### Related Parameters
The parameters are defined in `proto/camera_param.proto`. For the configuration of the parameters, see `config/camera_param.prototxt`which accepts multiple camera configurations. Single camera parameters include

* camera_name (`string`)
  
  Camera name, namespace used to post camera messages

* camera_type (`string`)
  
  Camera type, used to register and instantiate key values of camera objects in abstract factory mode, such as uvc camera as "uvc"

* camera_path (`string`)
  
  Camera path, used to open the camera port path name

* camera_matrix (`double[]`)

  Camera internal reference matrix,  generally 3X3

* camera_distortion (`double[]`)

  Camera distortion matrix

* fps (`uint32`)

  Camera frame rate（Unit frame per second）

* resolution

    * width (`uint32`) 

      Resolution width (in pixels)

    * height (`uint32`)

      Resolution length (in pixels)

    * width_offset (`uint32`)

      Resolution width offset value (in pixels) for the Crop offset when the hardware captures the image frame

    * height_offset (`uint32`)

      Resolution length offset value (in pixels) for the Crop offset when the hardware captures the image frame

* auto_exposure (`bool`)

  Auto exposure

* exposure_value (`uint32`)

  Exposure value

* exposure_time (`uint32`)

  Exposure time (unit us)

* auto_white_balance (`bool`)

  Automatic white balance

* white_balance_value (`uint32`)

  White balance value

* auto_gain (`bool`)

  Auto gain

* gain_value (`uint32`)

  Gain value

* contrast (`uint32`)

  Contrast

### Output

/camera_name/camera_info ([sensor_msgs/CameraInfo]())

 Camera parameter information

/camera_name/image_raw ([sensor_msgs/Image]())

 Camera raw image data


## Compile and Run

### Compile 

Compile in the ROS workspace

```shell
catkin_make -j4 roborts_camera_node
```

### Run

Start camera node

```shell
rosrun roborts_camera roborts_camera_node
```

Open rqt image viewer

```shell
rqt_image_view
```
