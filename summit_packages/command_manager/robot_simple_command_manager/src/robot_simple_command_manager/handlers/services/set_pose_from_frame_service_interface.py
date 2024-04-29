#!/usr/bin/env python
from robot_simple_command_manager.handlers.command_service_interface import *
from marker_mapping.srv import  InitPoseFromFrameRequest

class SetPoseFromFrameServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_description = ['frame_id']
        self.args_types = [str]
        self.args_void_allowed = [False]
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            self.request = InitPoseFromFrameRequest()
            self.request.frame_id = args[0]

    def parse_feedback(self):
        '''
        Parse the feedback_msg and return a string with the feedback of the command.
        This string feedback will be used by the command_manager action server to update
        its status.
        For actions, the feedback_msg equals to the feedback message published by the target server.
        For services, the feedback_msg equals to the response message of the target server. 
        '''
        return self.feedback_msg.message

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
    
    def has_succeeded(self):
        return self.feedback_msg.success
