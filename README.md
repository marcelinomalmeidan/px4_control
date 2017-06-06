# px4_control
ROS control interface for PX4 quadcopters. The main motivation of the existence for this REPO is to have a position controller that takes into consideration Position + Velocity + Acceleration from a Path Planner. This allows precise position control instead of PX4's own position controller that only takes into consideration either Position OR Velocity OR Acceleration commands.

This software has three different modes that one can choose from:

- Attitude mode: this mode sends thrust and attitude references based on a Xbox joystick.

- Local Position Control mode: this mode gets Position+Velocity+Acceleration references, calculates control commands and sends thrust + attitude to Px4. The controller is based on:
http://ieeexplore.ieee.org/abstract/document/5717652/

- Px4 Position Control mode: this mode gets position referencesand sends them to Px4's own position controller. 

The References for the previously mentioned Position Controllers can come from two different sources:

- Joystick: this software can already take in data from joysticks and integrate them to get smooth Position + Velocity + Acceleration references.

- ROS Topic: this software can also take incoming references from a Path Planner that publishes into the topic ``` /px4_control/PVA_Ref```

## Dependencies

- This software was developed for ROS Kinetic in Ubuntu 16.04. We haven't tested for other distributions. See installation procedure in ```http://wiki.ros.org/kinetic/Installation/Ubuntu ```

- Eigen

```sudo apt-get install libeigen3-dev ```

- MAVROS

``` sudo apt-get install ros-kinetic-mavros ros-indigo-mavros-extras```

- ROS Joystick Drivers (we have only tested Wired XBox One, Wireless Xbox 360, and Wired Xbox 360 controllers):

```https://github.com/ros-drivers/joystick_drivers```


## Compiling

- Copy the present REPO into a catkin workspace, e.g.:

```
cd ~/catkin_ws/src
git clone git@github.com:marcelinomalmeidan/px4_control.git
```

- Compile the catkin workspace, e.g.:

```
cd ~/catkin_ws
catkin_make
```

## Testing

- We recommend test in simulation before testing with a real Quadcopter. Examples of simulation environments can be Gazebo (https://dev.px4.io/en/simulation/gazebo.html) or Microsoft's Airsim (https://github.com/Microsoft/AirSim).

- In order to test the software from this repo, the following steps should be taken:
	- Run PX4 software
	- Run MAVROS (an example of .launch file is in ```/Extras```)
	- Run Joystick driver (an example of .launch file is in ```/Extras```)
	- Run px4_control (examples of .launch files can be found in ```/launch```)

**The following joystick buttons correspond to:**

* A: Land Mode (turn motors off)
* B: ROS Position Control Mode
	* Position references come from the topic ```/px4_control/PVA_Ref```
* X: Joystick Position Control Mode
	* Altitude can be changed using LB and RB. Horizontal translation can be changed using the Right Analog Directional. Yaw reference is changed when pushing LT and RT.
* Y: Joystick Attitude Mode.
	* Roll and Pitch are commanded through the Joystick's Right Analog Directional. Thrust is commanded through the Left Analog Directional. Yaw reference is changed when pushing LT and RT.