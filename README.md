# Tethered-TurtleBot3-Waffle-Pi
This repository consists of the code for "Object Gathering with a Tethered Robot Duo".

## Code Description

### Gazebo Simulation

`turtlebot3_gazebo/` includes the necessary files for running simulation in Gazebo. **"tethered_turtle.urdf.xacro"** and **"tethered_turtle.gazebo.xacro"** in `urdf/` define the model of the tethered TurtleBots. **"real.world"** in `worlds/` defines the world environment. **"turtlebot_motor.launch"** in `launch/` and **"torque_controller.yaml"** in `config/` define the controllers. **"turtlebot3_real_world.launch"** in `launch/` helps to launch the Gazebo simulation. **"traj.py"** and **"turtle_adaptive_one.py"** in `src/` are two python files that control the motors.

### Real-world Experiment

`turtlebot3_torque/` includes the firmware that needs to be uploaded to OpenCR 1.0 on Turtlebot3 Waffle Pi. Check out the files in `examples/turtlebot3_torque_waffle/turtlebot3_torque_core/`, as well as **"turtlebot3_torque_motor_driver.h"** in `include/turtlebot3_torque/` and **"turtlebot3_torque_motor_driver.cpp"** in `src/turtlebot3_torque/`. Note that we command the torque of each wheel of Turtlebot3 by writing the current instead of commanding the velocity of Turtlebot3.

`Pi/` includes the files that need to be uploaded to Raspberry Pi 3B+ after proper SBC setup as in [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup). **"traj.py"** reads the trajectories information from traj_opt.txt, and **"turtle_adaptive_one.py"** controls the motors. **"plot_traj.py"** should be on your local laptop or PC. **"real_traj.txt"** is a file recording the real trajectories of the two Turtlebot3.

## Program Running

### Gazebo Simulation
Open a terminal, and run:
```
roscore
```

Open another terminal, and run:
```
roslaunch turtlebot3_gazebo turtlebot3_real_world.launch
```

After the Gazebo window is opened, open another 2 terminals and run **"turtle_adaptive_one.py"** and **"traj.py"** in `turtlebot3_gazebo/src/`sequentially.

### Real-world Experiment

Before continuing, make sure your laptop or PC and the SBC on TurtleBot3 are connected to the same WiFi. Configure their **ROS_MASTER_URI** and **ROS_HOSTNAME** properly.

On your laptop or PC, open a terminal, and run:
```
roscore
```

Open another terminal, ssh to the corresponding SBC, and run:
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Open another terminal, ssh to the corresponding SBC, and run:
```
./turtle_adaptive_one.py
```

Open another terminal, ssh to the corresponding SBC, and run:
```
./traj.py
```

After each experiment, put the TurtleBot3 to anywhere you want as the starting point, and press any arrow button on the remote controller for a while to reset the robot.
