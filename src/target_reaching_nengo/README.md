# target_reaching_nengo

Target reaching code with primitives, arm_robot and Nengo

* BaseNetwork with 3xVoluntary (near-far, up-down, left-right) is initialized in Main_TR_CL-class
* The network gets current error between target and TCP from Error-class via subscribing to error-topic
* The network gets current positions of the arm joints from Feedback-class via subscribing to joint_states-topic
* The network publishes desired arm joints positions in BaseNetwork-class
* Mapping from desired arm joints positions to arm joint controller is implemented in TargetReachingToHBPMapping-class

## To start the main target reaching
* gazebo with arm and target should be running
* start nengo:
```
roslaunch target_reaching_nengo Main_TR_CL_nengo.launch
```
* for HBP mapping is necessary:
```
roslaunch target_reaching_nengo hbp_mapping.launch
```
