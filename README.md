# Project - Anthropomorphic Robotic Arm

This project implements the <u>first</u> and the <u>second</u> point of the assignment. The <u>third</u> point was being developed but is far from finished.

The source code is divided into three main group: the **start_detect group**, the **decision_maker/kinematic group** and the **move_robot group**.

The start_detect group is in charge of the detection and recognition of the objects. To do so we use 2 cameras, one in a slightly higher position than the
table and one directly above. The above camera is used not for the recognition itself, but rather for getting precise coordinates of the block, and uses openCV,
while the front camera uses yolo v5.

Once the blocks are correctly identified and localized (done by merging the results of the 2 cameras) those information are sent to the decision_maker/kinematic
group. Here a moving plan is created to make the robot move properly, following a 6-step process for every piece, and using kinematic to determine the correct 
value to be assigned to every joint. 

Once a step of the plan is ready, the information are sent to the move_robot node.

This last node is the only one that is actually interfaced with the robot, and sends the joints position directly to the actuator that executes the command.



To start the simulation:

```/bin/bash
roslaunch support world.launch
```

To start the assignment, in a different window:

```/bin/bash
rosrun start assignment2
```

