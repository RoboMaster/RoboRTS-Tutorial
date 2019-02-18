RoboRTS is an open source software platform based on real-time strategy robotics research , which goal is to provide a versatile hardware and software solution for multi-robot decision research based on learning (reinforcement learning, imitation learning or neroevolution).

This article will introduce the `RoboRTS` platform from the following aspects:

1. Get started quickly (in order for people to run the demo quickly)
   - Composition and construction of hardware
     - Mechanical structure, hardware selection and link
     - Installation and connection
   - Composition and preparation of software
     - Embedded layer STM32 programming code
     - Installation and connection of software： 
         - Nvidia Jetson TX2 related configuration（adapt to the install configuration of Jetson）
         - ROS and depend installation
         - udev configuration，network configuration，Boot service configuration

     Configuration of the simulation platform
     - Stage
     - Gazebo
   - Simple test and debug

       - Demo
2. Developer SDK(in order to let people invoking function and brief develop better) 
   - Agreement
   - Overall system framework and implementation functions
   - RoboRTS interface common format, each interface call and test instance
3. Introduction to System Algorithms (for science tutorials or algorithmic knowledge summaries)
   - State estimation
   - Visual perception
   - Route planning
   - Feedback controls
   - Behavior decision
4. System design ideas Modularity (in order to make people better build and improve the platform)
   - Mechanical design
   - Three platform system design patterns
       - STM32
       - RoboRTS
       - Simulation platform ×
5. Functional requirements and future planning

> **Tips*： 
>
> 1. The second part only writes the interface module
>     - Overall format, running ideas
>     - Dependent nodes and input information
>     - Output information
> 2. The third part writes the part of the specific implementation algorithm, mainly what is the parameter of each algorithm, and how to debug the parameters, preferably with theory and derivation.


