#!/usr/bin/env python
from ..command_action_interface import *

from robotnik_msgs.msg import ptz as Ptz
from std_msgs.msg import Float64, Int32

class PtzActionInterface(CommandActionInterface):
    def __init__(self, name, parameters):
        self.client = None
        self.feedback_msg = None
        self.feedback_str = ''

        self.name = name
        self.set_parameters(parameters)

        self.args_description = ['Pan', 'Tilt', 'Zoom']
        self.args_types = [float, float, float]
        self.args_void_allowed = [False, False, False]
        self.default_values = []
        self.output_types = []
        self.output_description = []
        
    def set_parameters(self, parameters):

        '''
            Set all the required parameters of the interface
        '''
        
        self.check_parameters(parameters)

        self.parameters = parameters

        self.ros_control = self.get_parameter('ros_control', False)

        if self.ros_control == False:

            self.ptz_topic = self.get_parameter('ptz_topic', 'axis/cmd')
            self.ptz_pub = rospy.Publisher(self.ptz_topic, Ptz, queue_size=10)
        
        else:

            self.pan_topic = self.get_parameter('pan_topic', 'joint_pan_position_controller/command')
            self.tilt_topic = self.get_parameter('tilt_topic', 'joint_tilt_position_controller/command')
            self.zoom_topic = self.get_parameter('zoom_topic', 'joint_zoom_position_controller/command')

            self.pan_pub = rospy.Publisher(self.pan_topic, Float64, queue_size=10)
            self.tilt_pub = rospy.Publisher(self.tilt_topic, Float64, queue_size=10)
            self.zoom_pub = rospy.Publisher(self.zoom_topic, Int32, queue_size=10)


    def check_parameters(self, parameters):
        '''
            Checks that the handler has all required parameters set
        '''
        if 'type' not in parameters.keys():
            self.raise_exception('check_parameters: Handler has not "type" parameter defined.')
        else:
            if not self.is_parameter_type_correct(parameters['type'], str):
                self.raise_exception('check_parameters: Handler has "type" parameter defined but is not a string.')

        return

    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        parsed_arguments = self.parse_args(args)

        if self.ros_control == False:

            ptz = Ptz()
            ptz.pan = parsed_arguments[0]
            ptz.tilt = parsed_arguments[1]
            ptz.zoom = parsed_arguments[2]
            ptz.relative = False
            ptz.mode = 'position'
            self.ptz_pub.publish(ptz)

        else:

            pan = Float64()
            pan.data = parsed_arguments[0]
            self.pan_pub.publish(pan)

            tilt = Float64()
            tilt.data = parsed_arguments[1]
            self.tilt_pub.publish(tilt)

            zoom = Int32()
            zoom.data = parsed_arguments[2]
            self.zoom_pub.publish(zoom)

        return True, ""

    def get_output(self):

        return "" 

    def cancel_cmd(self):
        return

    def get_feedback(self):
        '''
            Return feedback string
        '''
        return "Ok"

    def is_active(self):
        return False

    def has_succeeded(self):
        return True

    def get_result(self):
        '''
            Gets the result as string of the action
        '''
        
        return "OK"

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''


    def parse_feedback(self):
        '''
            Parse from feedback object to string 
        '''
        # DELETE THIS BLOCK AFTER IMPLEMENTING YOUR HANDLER
        # FEEDBACK_MSG HAS THE SAME STRUCTURE AS THE ACTION FEEDBACK MSG
        # THAT USES THE ACTION SERVER DEFINED BY THE NAMESPACE OF THE HANDLER
        # DEFINE THE RETURN VALUE BASED ON THE ACTION FEEDBACK
        return #str(self.feedback_msg)