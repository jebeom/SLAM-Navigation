#!/usr/bin/env python
import sys

from importlib import import_module

import rostopic

if sys.version_info.major == 3:
    from robot_simple_command_manager.handlers.command_interface import *
else:
    from command_interface import *

class CommandPublisherInterface(CommandInterface):

    def __init__(self, name, parameters):
        CommandInterface.__init__(self, name, parameters)
        self.msg = None
        self.client = None

        #self.import_messages()

        # Saves the time of the feedback msg
        #self.last_time_received_feedback = rospy.Time(0)
        # Timeout to control the lack of communication from the action server
        #self.communication_timeout = rospy.Duration.from_sec(10)

        #self.data = None
        #self.desired_value = None


    
    def import_messages(self):
        pass
        resolved_topic_name = rospy.resolve_name(self.namespace )
        topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)

        if topic_type is None:
            msg = 'Could not find \"' + self.namespace + '\" topic.'
            CommandInterface.raise_exception(self, msg)
        topic_type_module, topic_type_name = tuple(topic_type.split('/'))


        msg_module = import_module(topic_type_module + '.msg')
        self.type_topic = getattr(msg_module, topic_type_name)

    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        parsed_arguments = self.parse_args(args)
        self.build_msg(parsed_arguments)

        self.client.publish(self.msg)
        
        return True,""


    def cancel_cmd(self):
        pass
    
    def get_feedback(self):
        '''
            Return feedback string
        '''
        return self.feedback_str

    def is_active(self):
        self.feedback_str = "Data published in " + str(self.namespace)
        return False

    def has_succeeded(self):
        
        return True

    def get_result(self):
        '''
            Gets the result as string of the action
        '''
        result = "Data published in " + str(self.namespace) 
        return result

    def feedback_cb(self, feedback):
        pass
    
    @abstractmethod
    def build_msg(self, args):
        '''
            Return True if the command has been sent
        '''
        pass

    def parse_feedback(self):
        feedback = "Data published in " + str(self.namespace) 
        return feedback
