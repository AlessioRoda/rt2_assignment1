# First Assignment of the Research Track 2 course (action branch), Alessio Roda 4458313

The purpose of this package is to create a simulation in wich the robot follows some random positions generated by the a random server and reach it.
The robot can also stop if user orde it, but oly when it arrives in the last position that had to reach.
In this branch respect to the main branch, the files "state_machine.cpp" and "position_service.cpp" are implemented as ROS2 component and the general architecture is divided in two parts: one devleopped with ROS2 component and the other one develloped by mantainig the scripts of the main branch in ROS1.
So by considering this, in order to run the code there will be an interaction between ROS1 and ROS2 that can be possible thanks to a "ros bridge", if you haven't installed it yet you can fin the guide to download and install here: https://github.com/ros2/ros1_bridge.

Before start remembre to download the rt2_assignment1 in your ros workspace, since as said before it's necessary in order to run the code. 

## Description of the architecture
 
 The code here was developped for ros2 foxy, so make sure that you have this distribution in your computer before start. This part of the architecture is based on two components:
 
 1) position_service_component generate a random position and send it with RandomPosition custom service message.
 2) state_machine_component is the "main" of the architecture: it gets the commad custom service message and if it's "start", it publish the position the robot has to reach by setting the coordinates it takes from the RandomPosition custom service message. Then it checks if user wants to stop the robot and in case stop sending random positions to reach.
 
 In this package there are four folders:
 
 * launch
 * src
 * srv
 * urdf

There is also a docs folder wich contains a deeper documentation about the code, please have a look to it to understand in detail what every code do, here below there's a more "superficial" description about what the folders contains and what is changed respect to the main branch.

 
The "launch" one contains two launch files 

 * ros2_sim.py
 
 which is the launch files to run the simulation with gazeebo. The file provides to load the components in the rt2_assignment1 container.

In "src" there are the "position_service.cpp" and "state_machine.cpp" codes, they have been developped as ros component and the scope of the code is to generate a random position that is send from the position_service_component to the state_machine_component via RandomPosition custom message. When the component state_machine_component gets the position from the server it sets the parametres of the posiotion to reach in a Position custom service message and sends to the /go_to_point node thanks to the ros bridge, then the robot starts moving to the target to reach. The state_machine_component performs a loop in wich verifies if the user sends the 0 command in order to stop the robot; in this case state_machine_component won't send a new toarget to reach once the current target position has been reached. 

The "srv" folder contains the Command.srv and RandomPosition.srv files wich contains the information for the message respectively for the command that the user provides in the "/user_interface" node, and the position that robot has to reach provided by the "/random_position_server" node.

Finally "urdf" is a folder that contains the description of the robot we are using in the simulation.

There are also the gazeebo_script.sh and mapping_rules.yaml files, the first one is an executable to run the entire simulation, while the second one it's necessary to compile the ros bridge in order to share the service custom messages between the two ros distributions.

## Behaviour of the architecture

As said befeore the entire achitecture is based either in ROS2 and ROS1 code thanks to the ros bridge; the scope of the bridge is to share the custom server messages RandomService, Position and Command from the ROS2 distribution to the ROS1 one. For what regarards the sharing of messages between the nodes and the components it's similar to what happensa in action branch, in this case there's no sharing of /cmd_vel messages between the sate_machine_component and the gezeebo simulation since there isn't the possibility to stop the robot before it reaches a defined goal.

## How to run the code

First before runnig the code make sure you have downladed the ros bridge, you will have to compile it again in order to permit it to shere the custom service messages. Once you have done this there are different ways to run the code

### Run with gazeebo_script.sh

The easiest way to launch the entire simulation is just to copy and paste the gazeebo_script.sh file in the root folder, then you have to open it and provide to create in the root folder three files: ros.sh, ros2.sh, ros12.sh. These files are necessary to access to set the correct ros distribution in the terminal.  

Before proceiding install gnome-terminal with

```
install gnome-terminal
```

In case you have to create them you can just copy and paste 

ros.sh:
```
#!/bin/bash
source your_ros1_work_space/devel/setup.bash
```

ros2.sh:
```
#!/bin/bash
source your_ros2_work_space/install/setup.bash
```
ros12.sh:

```
#!/bin/bash
source your_ros1_work_space/install/setup.bash
source your_ros2_work_space/install/setup.bash
```
Make sure that these files and the gazeebo_script one have the permission to be executable.

Now the only thing that you have to do in order to launch the entire simulation is to open the terminal in the root folder ad digit 

```
./gazeebo_script.sh
```

Now three different terminals should have been opened and you can find the user terminal interface to move the robot in the first one.

### Run by launching the nodes and the components via terminal

To run the simulation without launching the file you can create a terminal in your_ros1_workspace and configure it with ros1, then type 

```
roslaunch rt2_assignment1 sim_bridge.launch
```
Now open another terminal, configure it with both ros1 and ros2, then in your workspace run the ros bridge by typing

```
ros2 run ros1_bridge dynamic_bridge
```
Finally open anotherter terminal, configure it with ros2, then in your workspace run

```
ros2 launch rt2_assignment1 ros2_sim.py
```
