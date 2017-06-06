# px4_control
ROS control interface for PX4 quadcopters. The main motivation of the existence for this REPO is to have a position controller that takes into consideration Position + Velocity + Acceleration from a Path Planner. This allows precise position control instead of PX4's own position controller that only takes into consideration either Position OR Velocity OR Acceleration commands.

This software has three different modes that one can choose from:

- Attitude mode: this mode sends thrust and attitude references based on a Xbox joystick.

- Local Position Control mode: this mode gets Position+Velocity+Acceleration references, calculates control commands and sends thrust + attitude to Px4. The controller is based on:
http://ieeexplore.ieee.org/abstract/document/5717652/

- Px4 Position Control mode: this mode gets position referencesand sends them to Px4's own position controller. 

The References for the previously mentioned Position Controllers can come from two different sources:

- Joystick: this software can already take in data from joysticks and integrate them to get smooth Position + Velocity + Acceleration references.

- ROS Topic: this software can also take incoming references from a Path Planner that publishes into the topic ```shell /px4_control/PVA_Ref```

## Compiling