#!/usr/bin/env python
import sys
from importlib import import_module

import rostopic
import actionlib
from actionlib_msgs.msg import GoalStatus

if sys.version_info.major == 3:
    from robot_simple_command_manager.handlers.command_interface import *
else:
    from command_interface import *


class CommandActionInterface(CommandInterface):

    def __init__(self, name, parameters):
        CommandInterface.__init__(self, name, parameters)

        self.import_messages()

        self.goal = self.type_goal()
        self.client = actionlib.SimpleActionClient(self.namespace, self.type_action)
        # Saves the time of the feedback msg
        self.last_time_received_feedback = rospy.Time(0)

    def import_messages(self):
        resolved_topic_name = rospy.resolve_name(self.namespace + '/result')
        topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)

        try:
            action_type = topic_type[:-len('ActionResult')]
        except Exception as identifier:
            msg = 'Could not find \"' + self.namespace + '\" action server.'
            CommandInterface.raise_exception(self, msg)

        action_type_module, action_type_name = tuple(action_type.split('/'))
        msg_module = import_module(action_type_module + '.msg')
        self.type_action = getattr(msg_module, action_type_name + 'Action')

        self.type_action_goal = getattr(msg_module, action_type_name + 'ActionGoal')
        self.type_action_result = getattr(msg_module, action_type_name + 'ActionResult')
        self.type_action_feedback = getattr(msg_module, action_type_name + 'ActionFeedback')

        self.type_goal = getattr(msg_module, action_type_name + 'Goal')
        self.type_result = getattr(msg_module, action_type_name + 'Result')
        self.type_feedback = getattr(msg_module, action_type_name + 'Feedback')
    
    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandInterface.set_parameters(self, parameters)
        
        # Timeout to control the lack of communication from the action server
        timeout_seconds = self.get_parameter('communication_timeout', 10)
        
        if timeout_seconds < 0.0:
            msg = 'communication_timeout parameter set to negative value which'
            msg += ' it is impossible. Automatically set to 0.0 for "%s" command.'
            rospy.logerr(msg %(self.name))
            timeout_seconds = 0.0
        
        if timeout_seconds == 0.0:
            msg = 'communication_timeout parameter set to 0.0, there will not be'
            msg += ' action communication monitoring for "%s" command.'
            rospy.logwarn(msg % (self.name))
        
        self.communication_timeout = rospy.Duration.from_sec(timeout_seconds)


    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        parsed_arguments = self.parse_args(args)

        self.build_msg(parsed_arguments)

        if self.client.wait_for_server(rospy.Duration.from_sec(2)) == False:
            return False,"The server is not active"

        self.client.send_goal(self.goal, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
        self.last_time_received_feedback = rospy.Time.now()

        return True,""

    def cancel_cmd(self):
        self.client.cancel_all_goals()

    def feedback_cb(self, feedback):
        self.feedback_msg = feedback
        self.last_time_received_feedback = rospy.Time.now()
        self.feedback_str = self.parse_feedback()
    
    def done_cb(self, code, result):
        self.result_msg = result
        return

    def get_feedback(self):
        '''
            Return feedback string
        '''
        return self.feedback_str

    def is_active(self):
        state = self.client.get_state()
        communication_timeout = False
        if self.communication_timeout.secs != 0.0:
            communication_timeout = ((rospy.Time.now() - self.last_time_received_feedback) >= self.communication_timeout)
        else:
            msg = 'Action communication monitoring is disabled for "%s" command'
            msg += '. Communication will not be considered as timedout.'
            rospy.logwarn_throttle(5, msg %(self.name))
        return ((state == GoalStatus.ACTIVE or state == GoalStatus.PENDING) and communication_timeout == False)

    def has_succeeded(self):
        state = self.client.get_state()
        return state == GoalStatus.SUCCEEDED

    def get_result(self):
        '''
            Gets the result as string of the action
        '''
        communication_timeout = False
        if self.communication_timeout.secs != 0.0:
            communication_timeout = ((rospy.Time.now() - self.last_time_received_feedback) >= self.communication_timeout)
        
        if communication_timeout == True:
            return "Communication timeout with the server"

        self.output = self.parse_output()
        return self.client.get_goal_status_text()

    @abstractmethod
    def build_msg(self, args):
        '''
            Return True if the command has been sent
        '''
        pass

    @abstractmethod
    def parse_feedback(self):
        pass
