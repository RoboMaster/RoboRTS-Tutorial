# Localization

## Module Introduction

The localization node is a required dependency in the localization system. By analysing data from sensors using specific algorithms, the node acquires the transformation of coordinate systems from the robot and the map, which indicates the pose and position of the robot. The default localization algorithm in `roborts_localization` is AMCL algorithm. For more information about AMCL and relative parameters, see [AMCL](en/sdk_docs/roborts_localization?id=amcl).


The data flow diagram is as follows:

![localization](https://rm-static.djicdn.com/documents/20758/0ba7ea27fc531547553277041255026.png)


The file directory is as shown below:

```bash
├── CMakeLists.txt
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindGlog.cmake
├── config
│   └── localization.yaml  # parameter config file for localization, loaded in the launch file
├── localization_config.h  # parameter structure class for localization
├── localization_math.cpp  
├── localization_math.h    # common math related functions, for internal use only
├── localization_node.cpp  # main node and main function
├── localization_node.h    
├── log.h                  # Glog Wrapper
├── package.xml
├── types.h                # Type define
├── amcl                   # AMCL algorithm directory
│   ├── amcl_config.h      # AMCL parameter config
│   ├── amcl.cpp           # AMCL main logic 
│   ├── amcl.h
│   ├── CMakeLists.txt    
│   ├── config
│   │   └── amcl.yaml      # AMCL parameter config file, loaded in the launch file
│   ├── map
│   │   ├── amcl_map.cpp   
│   │   └── amcl_map.h     # AMCL map related calculation
│   ├── particle_filter    # particle filter directory
│   │   ├── particle_filter.cpp
│   │   ├── particle_filter_gaussian_pdf.cpp
│   │   ├── particle_filter_gaussian_pdf.h
│   │   ├── particle_filter.h
│   │   ├── particle_filter_kdtree.cpp
│   │   ├── particle_filter_kdtree.h
│   │   └── particle_filter_sample.h
│   └── sensors            # Odometer sensor model and lidar sensor model
│       ├── sensor_laser.cpp
│       ├── sensor_laser.h
│       ├── sensor_odom.cpp
│       └── sensor_odom.h
```




Localization node can be started independently from the command line:

```bash
# compile roborts_localization
catkin_make -DCMAKE_BUILD_TYPE=Release localization_node

rosrun roborts_localization localization_node
```


Or run the node from launch file:

```xml
<!-- Load parameters -->
<rosparam command="load" file="$(find roborts_localization)/config/localization.yaml" />
<rosparam command="load" file="$(find roborts_localization)/amcl/config/amcl.yaml" />

<!-- Run the Localization Node -->
<node pkg="roborts_localization" type="localization_node" name="localization_node" respawn="false" />
```

Test result in RViz:

![localization_rviz](https://rm-static.djicdn.com/documents/20758/f172f1f29aa2f1547553303273821911.png)



## Input

* /map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

  (required) Subscribe to static map data.

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  (required) Obtain transformation between coordinate systems. (odom->bask_link）

* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))

  (required) Subscribe to laser scan data.

* /initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

  (optional) Estimate the pose and initialize the particle filter with this mean and variance. Corresponds to 2D Pose Estimate in Rviz.

* /uwb ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html))

  (optional) UWB data used to correct global localization data.

## Output

* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

  Estimated pose by the AMCL algorithm.

* /particlecloud ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

  The pose of the particles in the particle filter.

* /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  Publish the transformation between `odom` and the `map` coordinate systems.


> [!NOTE]
> before running `roborts_localization` make sure roborts_base and drivers for laser scan is correctly installed and running.



## Related Parameters

* odom_frame_id (`string`, default: "odom")

  coordinate system of odom

* base_frame_id (`string`, default: "base_link")

  coordinate system of the robot body

* global_frame_id (`string`, default: "map")

  coordinate system of the map

* laser_topic_name (`string`, default: "scan")

  name of the topic published by laser scan

* map_topic_name (`string`, default: "map")

  name of the topic published by the map

* init_pose_topic_name (`string`, default: "initialpose")

  name of the topic to which the estimated pose is sent

* transform_tolerance (`double`, default: 0.1)

  tf publish time interval


* initial_pose_x (`double`, default: 1)

  x position of initially estimated pose

* initial_pose_y (`double`, default: 1)

  y position of initially estimated pose

* initial_pose_a (`double`, default: 0)

  yaw angle of initially estimated pose

* initial_cov_xx (`double`, default: 0.1)

  the xx covariance of the initially estimated pose

* initial_cov_yy (`double`, default: 0.1)

  the yy covariance of the initially estimated pose

* initial_cov_aa (`double`, default: 0.1)

  the aa covariance of the initially estimated pose

* enable_uwb (`bool`, default: true)

  whether to enable uwb correction

* uwb_frame_id (`string`, default: "uwb")

  UWB coordinate system

* uwb_topic_name (`string`, default: "uwb")

  name of the topic published by UWB

* use_sim_uwb (`bool`, default: false）

  whether to use fake UWB data generated by stage simulator

* uwb_correction_frequency (`int`, default: 20)

  frequency of UWB correction


## AMCL

### ALgorithm Introduction

![autonomous mobile robot](https://rm-static.djicdn.com/documents/20758/6b0ea21b6d57a1547553332830529202.png)

Adaptive Monte Carlo Localization (AMCL) is a set of localization algorithms for 2D robot motion. The main algorithmic principle stems from the combination and implementation of the **MCL**, **Augmented_MCL**, and **KLD_Sampling_MCL** algorithms described in Probabilistic Robotics, Chapter 8.3. Meanwhile, the Motion Model described in Chapter 5 and the Sensor Model described in Chapter 6 are used for motion prediction and particle weight update in AMCL.

The integrated AMCL in `roborts_localization` comes with additional functions for random angles, random positions and angle initialization, in order to facilitate rapid deployment in the competition.


The basic logic of the algorithm is shown as follows.
![AMCL_flowchart](https://rm-static.djicdn.com/documents/20758/e75d3533fc03c1547553357220164263.png)



### Related Parameters

* min_particles (int, default: 50)

  the minimum number of particles in the particle filter

* max_particles (int, default: 2000)

  the maximum number of particles in the particle filter

* use_global_localization (bool, default: false)

  whether to randomly initialize global localization on start up

* random_heading (bool, default: false)

  whether to initialize random angle on start up

* update_min_d (double, default: 0.1)

  the displacement threshold for filter update


* update_min_a (double, default: 0.5)

  the rotation threshold for filter update


* odom_model (ENUM, ODOM_MODEL_OMNI)

  robot motion model, currently only supports omnidirectional wheel odometer models

* odom_alpha1 (double, default: 0.005)

  error parameter 1 for odometer model

* odom_alpha2 (double, default: 0.005)

  error parameter 2 for odometer model

* odom_alpha3 (double, default: 0.01)

  error parameter 3 for odometer model

* odom_alpha4 (double, default: 0.005)

  error parameter 4 for odometer model

* odom_alpha5 (double, default: 0.003)

  error parameter 5 for odometer model

* laser_min_range (double, default: 0.15)

  the minimum effective distance of the laser radar


* laser_max_range (double, default: 8.0)

  the maximum effective distance of the laser radar

* laser_max_beams (int, default: 30)

  the maximum number of laser beams

* laser_model (ENUM, default: LASER_MODEL_LIKELIHOOD_FIELD_PROB)

  laser radar rangefinder model. Currently only the likelihood domain improvement model is supported.

* z_hit (double, default: 0.5)

  the $z_{hit}$ parameter in the likelihood domain model

* z_rand (double, default: 0.5)

  the $z_{rand}$ parameter in the likelihood domain model

* sigma_hit (double, default: 0.2)

  the $\sigma_{hit}$ parameter in the likelihood domain model

* laser_likelihood_max_dist (double, default: 2.0)

  the maximum distance between the ranging point and the obstacle in the likelihood domain model

* do_beamskip (bool, default: true)

  whether to ignore part of the laser beams in Position Tracking phase, in order to avoid unpredictable errors, such as moving objects.

* beam_skip_distance (double, default: 0.5)

  the distance to ignore obstacles detected by the laser beam

* beam_skip_threshold (double, default: 0.3)

  the threshold to ignore obstacles detected by the laser beam

* beam_skip_error_threshold (double, default: 0.9)

  the error threshold for beam to ignore/skip obstacles

* resample_interval (int, default: 2)

  interval for resampling

* recovery_alpha_slow (double, default: 0.001)

  $\alpha_{slow}$ parameter in **Augmented_MCL**

* recovery_alpha_fast (double, default: 0.1)

  $\alpha_{test}$ parameter in **Augmented_MCL**

* kld_err (double, default: 0.05)

  $\epsilon$ parameter in **KLD_Sampling_MCL**

* kld_z (double, default: 0.99)

  $(1-\delta)$ in **KLD_Sampling_MCL**

* laser_filter_weight (double, default: 0.4)

  weight parameter used to filter out measured laser data that has lower weight 

* max_uwb_particles (int, default: 10)

  the maximum resampling number with UWB as the mean in resampling phase

* uwb_cov_x (double, default: 0.06)

  the x covariance of the Gaussian distribution with UWB as the mean in resampling phase

* uwb_cov_y (double, default: 0.06)

  the y covariance of the Gaussian distribution with UWB as the mean in resampling phase

* resample_uwb_factor (double, default: 4.0)

  resampling factor used to determine symmetric localization





