# Software Architecture
The entire software system decouples the driver module, function module and algorithm module to enhance the portability of the platform and improve the efficiency of collaborative development by developers in different fields.

The entire software architecture is shown below:

![system_diagram](https://rm-static.djicdn.com/documents/20758/40aac2fa9e6b81547552996504129513.png)

Based on the platform, developers can develop in a buttom-up fashion to control the robot to complete different tasks. The whole framework is divided into two parts:


As the low-level computing platform, the STM32 microcontroller runs RTOS (Real-Time Operating System) with limited computing power to perform various real-time tasks:

- Closed-loop control tasks with feedback based on encoder or IMU

- Acquisition, pre-processing and forwarding of sensor information for pose estimation

- Forward information related to the competition, such as robot HP value, etc.

On-board computing devices (such as Manifold 2) act as the upper-level computing platform, responsible for the execution of various algorithms with large amounts of computation:

- Environment perception, including real-time localization, mapping, target detection, identification and tracking

- Motion planning, including global planning (path planning), local planning (trajectory planning)

- An observation-action-based decision model interface that adapts to various decision frameworks such as FSM (Finite State Machine), Behavior Tree and other learning-based decision frameworks.

The two computing platforms communicate with the specified open source protocol through the serial port. The platform also provides a ROS-based API interface, which can be used for secondary development in ROS using C++ or Python. Based on this interface, developers can control the movement of the chassis and the gimbal, control the launching mechanism to fire the projectile by changing the input frequency,  the velocity of projectiles and the quantity of projectiles, and receive the sensor information and game data in real-time. In more advanced applications, developers can define their own protocols based on the related documentation in [roborts_base](en/sdk_docs/roborts_base) to extend the capabilities of the robot.

# Applications

The original intention of RoboRTS and RoboMaster mobile robot platform is to encourage more developers to participate in the research of robot intelligent decision-making under different scenarios and rules. The first step is to implement some robot perception or planning algorithms based on this hardware platform, so the observation space and action space can be abstracted, and better decision models can be constructed.

Our primary scenario is the decision-making of robotic competition, as shown below:

Pursuit and search:

![airobot1](https://rm-static.djicdn.com/documents/20758/2c66c923154ae1547553014851539095.gif)

Patrol:

![airobot2](https://rm-static.djicdn.com/documents/20758/299a5a10c187a1547553038856640888.gif)




