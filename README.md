# legged_control

## Introduction

legged_control is an NMPC-WBC legged robot control stack and framework based
on [OCS2](https://github.com/leggedrobotics/ocs2) and [ros-control](http://wiki.ros.org/ros_control).

The advantage shows below:

1. To the author's best knowledge, this framework is probably the best-performing open-source legged robot MPC control
    framework;
2. You can deploy this framework in your A1 robot within a few hours;
3. Thanks to the ros-control interface, you can easily use this framework for your custom robot.

I believe this framework can provide a high-performance and easy-to-use model-based baseline for the legged robot
community.

## Installation

### Source code

The source code is hosted on GitHub: [qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control).

```
# Clone legged_control
git clone git@github.com:qiayuanliao/legged_control.git
```

### OCS2

OCS2 is a huge monorepo; **DO NOT** try to compile the whole repo. You only need to compile `ocs2_legged_robot_ros` and
its dependencies following the step below.

1. You are supposed to clone the OCS2, pinocchio, and hpp-fcl as described in the documentation of OCS2.

    ```
    # Clone OCS2
    git clone git@github.com:leggedrobotics/ocs2.git
    # Clone pinocchio
    git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
    # Clone hpp-fcl
    git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
    # Clone ocs2_robotic_assets
    git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
    # Install dependencies
    sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
    ```

2. Compile the `ocs2_legged_robot_ros` package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
    instead of `catkin_make`. It will take you about ten minutes.

    ```
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
    catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
    ```

    Ensure you can command the ANYmal as shown in
    the [document](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot) and below.
    ![](https://leggedrobotics.github.io/ocs2/_images/legged_robot.gif)

### Build

Build the source code of `legged_control` by:

```
catkin build legged_controllers legged_unitree_description
```

Build the simulation (**DO NOT** run on the onboard computer)

```
catkin build legged_gazebo
```

Build the hardware interface real robot. If you use your computer only for simulation, you **DO NOT** need to
compile `legged_unitree_hw` (TODO: add a legged prefix to the package name)

```
catkin build legged_unitree_hw
```

## Quick Start

```
roslaunch legged_unitree_description empty_world.launch
```

```
roslaunch legged_controllers load_controller.launch 
```

```
sudo apt install ros-noetic-rqt-controller-manager
rosrun rqt_controller_manager rqt_controller_manager
```

5. Set the gait in the terminal of `load_controller.launch`, then use RViz (you need to add what you want to display by
    yourself) and control the robot by `cmd_vel` and `move_base_simple/goal`:

![ezgif-5-684a1e1e23.gif](https://s2.loli.net/2022/07/27/lBzdeRa1gmvwx9C.gif)

### Note

- **THE GAIT AND THE GOAL ARE COMPLETELY DIFFERENT AND SEPARATED!**  You don't need to type stance while the robot is
    lying on the ground **with four foot touching the ground**; it's completely wrong since the robot is already in the
    stance gait.
- The target_trajectories_publisher is for demonstration. You can combine the trajectory publisher and gait command into
    a very simple node to add gamepad and keyboard input for different gaits and torso heights and to start/stop
    controller (by ros service).
