# px4_control
ROS control interface for PX4 quadcopters. The main motivation of the existence for this REPO is to have a position controller that takes into consideration Position + Velocity + Acceleration from a Path Planner. This allows precise position control instead of PX4's own position controller that only takes into consideration either Position OR Velocity OR Acceleration commands.

This software has three different modes that one can choose from:

- Attitude mode: this mode sends thrust and attitude references based on a Xbox joystick.

- Local Position Control mode: this mode gets Position+Velocity+Acceleration references, calculates control commands and sends thrust + attitude to Px4. The controller is based on:
http://ieeexplore.ieee.org/abstract/document/5717652/

- Px4 Position Control mode: this mode gets position references and sends them to Px4's own position controller. 

The References for the previously mentioned Position Controllers can come from two different sources:

- Joystick: this software can already take in data from joysticks and integrate them to get smooth Position + Velocity + Acceleration references.

- ROS Topic: this software can also take incoming references from a Path Planner that publishes into the topic ``` /px4_control/PVA_Ref```

## Dependencies

- This software was developed for ROS Kinetic in Ubuntu 16.04. We haven't tested for other distributions. See installation procedure in http://wiki.ros.org/kinetic/Installation/Ubuntu.

- Eigen

```sudo apt-get install libeigen3-dev ```

- MAVROS

``` sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras```

- ROS Joystick Drivers (we have only tested Wired XBox One, Wireless Xbox 360, and Wired Xbox 360 controllers):

```https://github.com/ros-drivers/joystick_drivers```


## Compiling

- Copy the present REPO into a catkin workspace, e.g.:

```
cd ~/catkin_ws/src
git clone https://github.com/marcelinomalmeidan/px4_control.git
```

- Compile the catkin workspace, e.g.:

```
cd ~/catkin_ws
catkin_make
```

## Testing

- We recommend test in simulation before testing with a real Quadcopter. Examples of PX4 simulation environments are Gazebo (https://dev.px4.io/en/simulation/gazebo.html) or Microsoft's Airsim (https://github.com/Microsoft/AirSim).

- In order to test the software from this repo, the following steps should be taken:
	- Run PX4 software
	- Run MAVROS (an example of .launch file is in ```/launch/px4.launch```)
		- You will have to change the argument ```fcu_url``` in px4.launch to match the IP/Ports used by your PX4 software.
		- ```roslaunch px4_control px4.launch```
	- Run Joystick driver (an example of .launch file is in ```/launch/joy.launch```)
		- Make sure you have a joystick connected to the computer.
		- ```roslaunch px4_control joy.launch```
	- Run px4_control (examples of .launch files can be found in ```/launch/dragonfly.launch``` or ```/launch/gazebo.launch```)
		- Make sure to tune the parameters in the .launch files. For instance, the flight performance depends largely on the "mass" and "thrustRatio" parameters. Also, you have to choose the appropriate joystick driver from one of the three (joyXboxOne  /  joyXbox360  /  joyXbox360Wired) in the .launch file. I suggest to create our own launch file for tuning, e.g., myQuad.launch.
		- ```roslaunch px4_control myQuad.launch```

**The following joystick buttons correspond to:**

<img src="http://compass.xboxlive.com/assets/c7/a1/c7a12fbe-af04-4a90-92f2-18338219c2aa.png?n=one-controller-front-l.png" width="480"> <img src="http://compass.xboxlive.com/assets/0f/8b/0f8babd7-1e9e-4122-996a-9b81f9482679.png?n=one-controller-back-l.png" width="480">

* Select (3): Terminate program and shut down this ROS node.
* A: Land Mode (turn motors off)
* B: ROS Position Control Mode
	* Position references come from the topic ```/px4_control/PVA_Ref```
* X: Joystick Position Control Mode
	* Altitude can be changed using LB (2) and RB (7). Horizontal translation can be changed using the Right Stick (10). Yaw reference is changed when pushing LT (14) and RT (11).
* Y: Joystick Attitude Mode.
	* Roll and Pitch are commanded through the Joystick's Right Stick (10). Thrust is commanded through the Left Stick (1). Yaw reference is changed when pushing LT (14) and RT (11).
* 8 (Left): Select Local Position Control mode. 
* 8 (Right): Select Px4 Position Control mode. 


## Tuning Position Controller PID

All the controller PID parameters can be set in the .launch file. However, it might be tedious to stop the controller every time that one wants to change/tune the PID gains. In order to avoid stopping the controller for every parameter change, the following services are implemented:

- ```/px4_control_node/updateQuadParam```: Used to update flight parameters in the following order: mass, gravity and thrustRatio. This can be called in command line as below:

	- ```rosservice call /px4_control_node/updateQuadParam '[0.5, 9.81, 2.4]'```

- ```/px4_control_node/updatePosControlParam```: Used to update PID parameters if the following order: kpx, kpy, kpz, kvx, kvy, kvz, kix, kiy, kiz, maxInteg_x maxInteg_y maxInteg_z. An example of command line to call this service is shown below:

	- ```rosservice call /px4_control_node/updatePosControlParam '[10, 10, 10, 5, 5, 7.5, 0, 0, 0, 0, 0, 0]'```

An alternative to call these parameters is to use the Matlab script in ```/Extras/SetParametersGazebo.m```. However, this requires Matlab with the Robotics Toolbox installed. 
