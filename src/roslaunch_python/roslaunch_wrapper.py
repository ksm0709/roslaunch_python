#!/usr/bin/env python
import roslaunch
import rospy

class RosLaunchWrapper(object):
    '''
    ROS Launch API Wrapper
    @brief this wrapper class enables executing roslaunch in python script
           just by passing string argument to start function. 
           
           Example :
            launch_wrapper = RosLaunchWrapper() 
            launch_wrapper.start("[package] [~~~.launch] [arg1:=value1] ...")
            ...
            if launch_wrapper.is_running() == True :
                ...
            
            launch_wrapper.stop()

    @autor TaehoKang (ksm07091@gmail.com)
    '''
    def __init__(self):
        super(RosLaunchWrapper, self).__init__()

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.cli_args = None
        self.launch_file = None
        self.launch_args = None
        self.parent = None
        self.running = False

    def start(self, args):
        self.cli_args = str(args).split(' ')
        self.launch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)
        self.launch_args = self.cli_args[2:]

        rospy.loginfo(self.cli_args)
        rospy.loginfo(self.launch_file)
        rospy.loginfo(self.launch_args)

        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.launch_file)
        self.parent.start()
        self.running = True

    def stop(self):
        if self.parent != None and self.running == True :
            self.parent.shutdown()
            self.running = False

    def is_running(self):
        return self.running