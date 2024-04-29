#!/usr/bin/env python
from robot_simple_command_manager.handlers.command_publisher_interface import *
from robotnik_msgs.msg import  ptz

class PtzPositionCommandPublisherInterface(CommandPublisherInterface):

    def __init__(self, name, parameters):
        self.args_description = ['pan', 'tilt', 'zoom', 'relative']
        self.args_types = [float, float, float, bool]
        self.args_void_allowed = [False, False, False, False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandPublisherInterface.__init__(self, name, parameters)
        self.client = rospy.Publisher(self.namespace, ptz, queue_size=1)

    def set_parameters(self, parameters):
        CommandPublisherInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            # Fill the different fields of the message
            # using the args list (i.e. self.msg.first_argument = args[0])
            self.msg = ptz()
            self.msg.pan = args[0]
            self.msg.tilt = args[1]
            self.msg.zoom = args[2]
            self.msg.relative = args[3]
            self.msg.mode = "position"

    def parse_feedback(self):
        '''
        Parse the feedback_msg and return a string with the feedback of the command.
        This string feedback will be used by the command_manager action server to update
        its status.
        For actions, the feedback_msg equals to the feedback message published by the target server.
        For services, the feedback_msg equals to the response message of the target server. 
        '''
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg)

    def parse_output(self):
        '''
        Returns a string following the output format (defined by output_description and output_types)
        separating the different fields with blank spaces.

        self.result_msg object contains the response message of the target server (Action or Server).

        Example:  
            Config:
            self.output_description = ['frame_id', 'tolerence']
            self.output_types = [string, float]

            Output:
            'robot_base_footprint 2.0'
        '''
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''
