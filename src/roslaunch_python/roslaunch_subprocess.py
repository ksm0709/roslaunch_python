#!/usr/bin/env python
import subprocess
import rospy

class RosLaunchSubprocess(object):
    '''
    ROS Launch Subprocess
    @brief this class enables executing roslaunch in python script utilizing subprocess
           just by passing string argument to start function. 
           
           Example :
            launch_sub = RosLaunchSubprocess() 
            launch_sub.start("[package] [~~~.launch] [arg1:=value1] ...")
            ...
            if launch_sub.is_running() == True :
                ...
            
            launch_sub.stop()

    @autor TaehoKang (ksm07091@gmail.com)
    '''
    def __init__(self):
        super(RosLaunchSubprocess, self).__init__()

        self.process = None
        self.running = False

    def start(self, cmd, output):
        if self.running :
            self.stop()

        rospy.loginfo("[RosLaunchSubprocess] exec : {:}".format(cmd))

        if output == "screen":
            output = None
        else:
            output = subprocess.PIPE

        self.process = subprocess.Popen("exec " + cmd, stdout=output,shell=True)
        self.check_state()

        return self.running == True

    def stop(self):
        if self.process != None and self.running == True :
            self.process.terminate()
            self.process.wait()
            self.check_state()  
        else:
            rospy.logwarn("[RosLaunchSubprocess] Tried to stop launch file.. but it is not running")

        return self.running == False

    def is_running(self):
        return self.running

    def check_state(self):
        self.running = False
        state = self.process.poll()

        if state is None:
            self.running = True   
            rospy.loginfo("[RosLaunchSubprocess] Process is running")
        elif state < 0:
            rospy.loginfo("[RosLaunchSubprocess] Process terminated with error")
        elif state > 0:
            rospy.loginfo("[RosLaunchSubprocess] Process terminated without error")

    
