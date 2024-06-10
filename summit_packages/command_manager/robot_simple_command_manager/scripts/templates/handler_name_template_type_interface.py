#!/usr/bin/env python
from robot_simple_command_manager.handlers.command_template_type_interface import *
## publisher
from publisher_package.msg import  msg_type_publisher
## publisher
## action
from action_package.msg import  msg_type_goalGoal
## action
## service
from service_package.srv import  msg_type_requestRequest
## service

class HandlerNameTemplateTypeInterface(CommandTemplateTypeInterface):

    ## action, service, procedure, publisher
    def __init__(self, name, parameters):
        self.args_description = []
        self.args_types = []
        self.args_void_allowed = []
        self.default_values = []

        self.output_description = []
        self.output_types = []

        CommandTemplateTypeInterface.__init__(self, name, parameters)
        ## publisher
        self.client = rospy.Publisher(self.namespace, msg_type_publisher, queue_size=1)
        ## publisher
    ## action, service, procedure, publisher

    ## action, service, procedure, publisher
    def set_parameters(self, parameters):
        CommandTemplateTypeInterface.set_parameters(self, parameters)
    ## action, service, procedure, publisher

    ## action, service, procedure, publisher
    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            ## action
            # Fill the different fields of the goal message
            # using the args list (i.e. self.goal.first_argument = args[0])
            self.goal = msg_type_goalGoal()
            ## action
            ## service, procedure
            # Fill the different fields of the request message
            # using the args list (i.e. self.request.first_argument = args[0])
            self.request = msg_type_requestRequest()
            ## service, procedure
            ## publisher
            # Fill the different fields of the message
            # using the args list (i.e. self.msg.first_argument = args[0])
            self.msg = msg_type_publisher()
            ## publisher
    ## action, service, procedure, publisher

    ## action, service, publisher
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
    ## action, service, publisher

    ## action, service, procedure, publisher
    def parse_output(self):
        '''
        Returns a string following the output format (defined by output_description and output_types)
        separating the different fields with blank spaces.

        self.result_msg object contains the response message of the target server (Action or Server).

        Example:  
            Config:
            self.output_description = ['frame_id', 'tolerence']
            self.output_types = ['string', 'float']

            Output:
            'robot_base_footprint 2.0'
        '''
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''
    ## action, service, procedure, publisher