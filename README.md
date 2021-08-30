# Tethered-TurtleBot3-Waffle-Pi
This repository consists of the code for "Gathering Scattered Objects with Tethered Robot Duo"

## Code Description

### Gazebo Simulation

`turtlebot3_gazebo` includes the necessary files for running simulation in Gazebo. **"tethered_turtle.urdf.xacro" and "tethered_turtle.gazebo.xacro"** in `urdf` define the model of the tethered TurtleBots. **"real.world"** in `worlds` defines the world environment. **"turtlebot_motor.launch"** in `launch` and **"torque_controller.yaml"** in `config` define the controllers. **"turtlebot3_real_world.launch"** in `launch` helps to launch the Gazebo simulation. **"traj.py"** and **"turtle_adaptive_one.py"** in `src` are two python files that control the motors.

### Real-world Experiment

`turtlebot3_torque` includes the firmware that needs to be uploaded to OpenCR 1.0 on Turtlebot3 Waffle Pi. Check out the files in `examples/turtlebot3_torque_waffle/turtlebot3_torque_core/`, as well as **"turtlebot3_torque_motor_driver.h"** in `include/turtlebot3_torque` and **"turtlebot3_torque_motor_driver.cpp"** in `src/turtlebot3_torque`. Note that we command the torque of each wheel of Turtlebot3 by writing the current instead of commanding the velocity of Turtlebot3.

## Program Running

