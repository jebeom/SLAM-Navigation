#!/usr/bin/env python

# Class to manage the comunication with any instance of the command manager

import rospy
import actionlib

from robot_simple_command_manager_msgs.msg import *
from robot_simple_command_manager_msgs.srv import *

class CommandManagerInterface():

    def __init__(self, namespace):
        '''
            param namespace as string
        '''
        self._namespace = namespace
        self._feedback_msg = RobotSimpleCommandFeedback()
        self._last_time_received_feedback = rospy.Time(0)
        # dictionary of  robot_simple_command_manager_msgs/HandlerInfo
        self._handler_info = {}
        # Timeout to control the lack of communication from the action server
        self._communication_timeout = rospy.Duration.from_sec(10)

        self._action_client = actionlib.SimpleActionClient(self._namespace + '/action', RobotSimpleCommandAction)
        self._handler_info_service_client = rospy.ServiceProxy(self._namespace + '/get_handler_info_list', GetHandlerInfoList)
        
        return

    def wait_for_server(self, duration):
        '''
         param duration as seconds (double)
         return true if ok, false in case of error
        '''
        ret = self._action_client.wait_for_server(rospy.Duration.from_sec(duration))
        
        if ret == False:
            rospy.logerr('error waiting for %s', self._namespace)
            return False
        
        ret = self._get_all_handlers_info()
        if ret == False:
            rospy.logerr('error getting handlers from %s', self._namespace)
            return False
        
        return True


    
    def get_namespace(self):
        return self._namespace

    def get_result(self):
        return self._action_client.get_result()

    def get_state(self):
        return self._action_client.get_state()
    
    def get_feedback(self):
        return self._feedback_msg

    def send_goal(self, goal):
        self._action_client.send_goal(goal, feedback_cb=self._feedback_cb)
        self._tick_feedback()
    
    def cancel(self):
        self._action_client.cancel_all_goals()

    def is_timed_out(self):
        return (rospy.Time.now() - self._last_time_received_feedback) >= self._communication_timeout

    def is_command_valid(self, command):
        '''
        Check if the command and arguments are valid
        param command as string[]
        return True if the command is valid,False otherwise
        '''
        # example: GOTO 0 0 0

        if len(command) == 0:
            return False
        command_id = command[0]
        if command_id not in  self._handler_info:
        # if self._handler_info.has_key(command_id) == False:
            rospy.loginfo('The command %s does not exist in %s', command_id, self._namespace)
            return False
        # takes the rest of arguments and compares with the number of types (args) of the registered command
        if len(command[1:]) != len(self._handler_info[command_id].types):
            rospy.loginfo('The arguments for command %s (%s vs %s) are not correct in %s', command_id, str(command[1:]), str(self._handler_info[command_id].types), self._namespace)
            return False
        
        return True

    def _feedback_cb(self, feedback):
        self._feedback_msg = feedback
        self._tick_feedback()

    def _tick_feedback(self):
        self._last_time_received_feedback = rospy.Time.now()

    def _get_all_handlers_info(self):
        '''
        Gets all the available handlers along with the related information
        return true if ok, false if error

        '''
        msg = GetHandlerInfoListRequest()
        ret = GetHandlerInfoListResponse()
        msg.name = ['']
        ret = self._handler_info_service_client.call(msg)
        if ret.ret.success == True:
            for handler in ret.handlers:
                self._handler_info[handler.name] = handler
            return True
        else:
            rospy.logerr('Error getting handlers info from %s: %s', self._namespace, ret.ret.message)
            return False