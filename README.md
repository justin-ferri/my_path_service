# my_path_service

my_path_service is a ros package that utilizes a path_service and path_client to move a robot from the bottom left corner of a maze to the top left corner given a set of poses. The set of poses are sent from the client to the service. The service processes the information.

This class depends on the example_ros_service package from wsnewman's learning_ros repository. This package must be in your working ros directory to compile.

## Example usage

To use my_path_service's nodes, run the following commands:

roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
rosrun my_path_service my_path_service
rosrun my_path_service my_path_client
    
