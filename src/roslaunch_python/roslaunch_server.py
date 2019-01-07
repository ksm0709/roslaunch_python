#!/usr/bin/env python
import rospy
from roslaunch_python.roslaunch_subprocess import RosLaunchSubprocess
from roslaunch_python.roslaunch_interface import RosLaunchInterface
from roslaunch_python.srv import *
from roslaunch_python.msg import LaunchInterface

class RosLaunchServer(object):
    '''
    ROS Launch Server
    @brief this class provides rosservice interfaces that run/stop roslaunch files.
           and it also provides argument management(=manage launch file <arg> tag values) utilizing RosLaunchInterface class

    @author TaehoKang (ksm07091@gmail.com)
    '''
    def __init__(self):
        super(RosLaunchServer, self).__init__()

        self.launchs = {}
        self.interfaces = {}

        self.srv_add = rospy.Service("~add",LaunchAdd, self.add_cb)
        self.srv_start = rospy.Service("~start",LaunchStart,self.start_cb)
        self.srv_stop = rospy.Service("~stop",LaunchStop,self.stop_cb)
        self.srv_status = rospy.Service("~check_status",LaunchCheckStatus,self.check_cb)

    def __del__(self):
        for launch in self.launchs:
            launch.stop()        

    def update(self, intf):
        if intf.name not in self.launchs :
            self.launchs[ intf.name ] = RosLaunchSubprocess()
            self.interfaces[ intf.name ] = RosLaunchInterface()

        valid = self.interfaces[ intf.name ].from_msg(intf)

        return valid

    def add_cb(self, req):
        result = self.update( req.interface )
        return LaunchAddResponse(result)

    def start_cb(self, req):
        if self.update( req.interface ) :
            launch_cmd = self.interfaces[ req.interface.name ].get_launch_cmd()
            result = self.launchs[ req.interface.name ].start( launch_cmd, req.interface.output )
            return LaunchStartResponse(succeeded=result)
        else:
            return LaunchStartResponse(succeeded=False)

    def stop_cb(self, req):
        result = False
        if req.name in self.launchs:
            result = self.launchs[ req.name ].stop() 
        return LaunchStopResponse(result)
    
    def check_cb(self, req):
        result = False

        if req.name == '-alive':
            result = True 

        if req.name in self.launchs:
            result = self.launchs[ req.name ].is_running()

        return LaunchCheckStatusResponse(result)

if __name__ == '__main__':
    
    rospy.init_node("roslaunch_server")
    server = RosLaunchServer()
    rospy.spin()
