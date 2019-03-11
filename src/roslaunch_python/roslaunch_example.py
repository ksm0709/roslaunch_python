#!/usr/bin/env python
import rospy, rospkg

from roslaunch_python.roslaunch_interface import RosLaunchInterface
from roslaunch_python.roslaunch_client import RosLaunchClient

if __name__ == "__main__":

    rospy.init_node('example_manager')    
    rospack = rospkg.RosPack()

    launch_client = RosLaunchClient(server_name='roslaunch_server')
    ex1_interface = RosLaunchInterface(name='example1', yaml_file=rospack.get_path('roslaunch_python')+'/launch/interface.yaml')
    ex2_interface = RosLaunchInterface(name='example2', yaml_file=rospack.get_path('roslaunch_python')+'/launch/interface.yaml')

    ex1_interface.set_arg('green','true')
    ex2_interface.set_arg('green','false')

    # Launch start & stop 'example.launch' with interface ex1_interface
    launch_client.start(ex1_interface)
    rospy.Rate(0.333).sleep() 
    launch_client.stop(ex1_interface)
    rospy.Rate(0.333).sleep() 

    # Launch start & stop 'example.launch' with interface ex2_interface
    launch_client.start(ex2_interface)
    rospy.Rate(0.333).sleep() 
    launch_client.stop(ex2_interface)
    rospy.Rate(0.333).sleep() 

 
