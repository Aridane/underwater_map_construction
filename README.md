# This is my README

#General description of contents
##charle_control
Contains launch files needed for controlling the joints in the simulator

##charle_description
Contains the code for gazebo plugins. Both the sonar simulation plugins can be found here. It also contains the robot model under de *urdf folder

##charle_gazebo
Contains world models for gazebo and the launch files for running the simulator and some related nodes.

##inertial_navigation
Brute attempt of integrating accelerations twice to obtain positions. Not a great idea.

##mlsm_manager
Contains the node in charge of generating the map and also the ICP class for matching.

##sonar_control
A node that moves the sonar so we can get a scan for each position using the step specified. Lets us move the servo to perform 3D scans.

##sonar_processing
Contains the nodelets for data processing (SonarToCloud, Thresholder, OutlierRemover, and Wall2DDetector)

##type_definitions
Contains type definitions, specially the ones needed for the MLSM structure.
