# localization_py
1) getting the transformation between camera and robot's base link frame. Can be done through tf2 or manually code it as it is a fixed transformation.
2) save the transformations of the apriltags in the map frame in a dictionary with the keys as the apriltags id. the look up is done through tf transform but there is also a function that can read a json file to get that. 
3) the transforms form the camera(camera_optical_frame) to the seen apriltags are saved in a dictionary with the keys as the apriltag id. 
4) we then loop over the seen apriltag and calculate the transformation from the robot(base_link) to the map frame.
5) we then can get the location of the robot in the world frame and publish it to "robot_pose" topic for the navigation stack to read