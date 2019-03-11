#!/usr/bin/env python
import rospy
import yaml

from roslaunch_python.msg import LaunchInterface
from roslaunch_python.srv import *

from roslaunch_python.roslaunch_interface import RosLaunchInterface

class RosLaunchClient(object):
    '''
    ROS Launch Client
    @brief this class provides connection to RosLaunchServer.  

    @author TaehoKang (ksm07091@gmail.com)
    '''

    def __init__(self, server_name = "roslaunch_server"):
        super(RosLaunchClient, self).__init__()

        self.proxy_add = rospy.ServiceProxy(server_name+"/add", LaunchAdd)
        self.proxy_start = rospy.ServiceProxy(server_name+"/start", LaunchStart)
        self.proxy_stop = rospy.ServiceProxy(server_name+"/stop", LaunchStop)
        self.proxy_status = rospy.ServiceProxy(server_name+"/check_status", LaunchCheckStatus)

        time = rospy.Time.now()
        found = False
        while rospy.Time.now() - time < rospy.Duration(10.0):
            try:
                rospy.logwarn("[RosLaunchClient] Searching server ({:})".format(server_name))
                self.proxy_status.call('-alive')
                found = True
                break
            except rospy.ServiceException:
                rospy.sleep(2.0)               

        if not found:
            rospy.logerr("[RosLaunchClient] Failed to find server ({:})".format(server_name))
            exit(0)

        rospy.loginfo("[RosLaunchClient] Succeeded to find server ({:})".format(server_name))

    def add(self, interface_obj):
        result = self.proxy_add.call(interface=interface_obj.to_msg())
        rospy.loginfo("[RosLaunchClient] Add interface ... ({:})".format(result))
        return result.succeeded

    def start(self, interface_obj):
        result = self.proxy_start.call(interface=interface_obj.to_msg())
        rospy.loginfo("[RosLaunchClient] Start interface ... ({:})".format(result))
        return result.succeeded

    def stop(self, interface_obj):
        result = self.proxy_stop.call(name=interface_obj._name)
        rospy.loginfo("[RosLaunchClient] Stop interface ... ({:})".format(result))
        return result.succeeded
    
    def status(self, interface_obj):
        result = self.proxy_status.call(name=interface_obj._name)
        rospy.loginfo("[RosLaunchClient] Get interface status ... ({:})".format(result))
        return result.result


   
    
