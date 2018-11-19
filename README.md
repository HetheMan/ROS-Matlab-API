# ROS-Matlab-API
Matlab abstraction of the robot (AMIGOBOT) and the connection to ROS.
##Robot Class.
The simulator robot is an AMIGOBOT robot type. The .xml to import the robot in the ROS simulator is included in the resources folder.
To create a robot just call the constructor of the class to create an object.
* * robot =  Robot(nameOfTheRobot, type) * *
type = 0 -> Simulation robot.
type = 0 -> Real robot.

The methods and properties are explained in the Robot.m file inside src folder.

##Map_Manager Class
Abstraction to connect to the ros simulation and mapping and localization algorithms are implemented too.
To create a Map_manager just call the constructor of the class to create an object.
* * map_Manager = Map_Manager(ROS_MASTER_IP,LOCAL_IP) * *

The methods and properties are explained in the Map_Manager file inside src folder.

## Main

Example of connection and robot creation.



