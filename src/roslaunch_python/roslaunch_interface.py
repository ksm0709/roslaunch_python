#!/usr/bin/env python
import rospy
import yaml

from roslaunch_python.msg import LaunchInterface

class RosLaunchInterface(object):
    '''
    ROS Launch Interface
    @brief this class contains the information about launch file (package, file name, args)
           these informations can be loaded by yaml file also.

    @author TaehoKang (ksm07091@gmail.com)
    '''

    _name = ""
    _package = ""
    _file = ""
    _output = ""
    _args = {}
    _valid = False
    _initialized = False

    def __init__(self, name='', yaml_file='', package='', file_name='', output='', args={}):
        super(RosLaunchInterface, self).__init__()

        self._valid = self.set_interface(name, yaml_file, package, file_name, output, args)
        self._initialized = True

    def set_interface(self, name='', yaml_file='', package='', file_name='', output='',args={}):

        if yaml_file is not '' and name is not '':

            with open(yaml_file, 'r') as stream : 
                setting = yaml.load(stream)

                if not 'interface' in setting.keys():
                    rospy.logerr("There is no \'interface\' namespace in {:}", yaml_file)
                    return False

                flag = False
                for key in setting['interface'].keys():
                    if key == name :
                        val = setting['interface'][key]

                        #invalid interface
                        if not 'package' in val.keys() or not 'file_name' in val.keys():
                            break
                        
                        self._name = name
                        self._package = val["package"]
                        self._file = val["file_name"]

                        if 'output' in val.keys():
                            self._output = val["output"]
                        if 'args' in val.keys():

                            for arg_key in val["args"].keys():
                                arg_val = str(val["args"][arg_key])

                                if type(val["args"][arg_key]) == 'bool':
                                    arg_val = arg_val.lower()

                                self._args[arg_key] = arg_val

                        flag = True

                if flag == False:
                    rospy.logerr("[RosLaunchInterface] Invalid interface is given... ({:})".format(name))
                    return False

        else:
            if package == '' or file_name == '' or name == '':
                if self._initialized :
                    rospy.logerr("[RosLaunchInterface] Invalid interface is given... ({:})".format(name))

                return False
            else :
                self._name = name
                self._package = package
                self._file = file_name
                self._output = output
                self._args = args

        rospy.loginfo("[RosLaunchInterface] Valid interface is given ({:}, {:}, {:})".format(name, self._package, self._file))
        return True

    def get_launch_cmd(self):

        if self._valid is False:
            rospy.logerr("[RosLaunchInterface] Can not make launch command using invalid interface") 
            return None

        cmd = "roslaunch " + self._package + ' ' + self._file

        for key in self._args.keys():
            cmd = cmd + ' ' + key + ':=' + self._args[key]

        return cmd

    def to_msg(self):

        result = LaunchInterface()
        result.name = self._name
        result.yaml_file = ''
        result.package = self._package
        result.file = self._file
        result.output = self._output
        result.arg_keys = self._args.keys()
        result.arg_values = self._args.values()

        return result

    def from_msg(self, msg):
        args = {}
        for idx in range(len(msg.arg_keys)):
            args[ msg.arg_keys[idx] ] = msg.arg_values[idx]

        self._valid = self.set_interface(name=msg.name, yaml_file=msg.yaml_file, package=msg.package, file_name=msg.file, output=msg.output, args=args)

        return self._valid

    def set_arg(self,key,value):
        self._args[key] = value 

    def clear_arg(self,key):
        del self._args[key] 
