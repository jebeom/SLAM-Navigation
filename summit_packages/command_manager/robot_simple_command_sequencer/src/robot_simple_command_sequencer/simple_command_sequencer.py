#!/usr/bin/env python
import re

import rospy
import actionlib

from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, TriggerResponse

from robot_simple_command_manager_msgs.msg import *
from robot_simple_command_manager_msgs.srv import SetCommandString, SetCommandStringResponse
from robot_simple_command_manager_msgs.srv import ManageCommandManager, ManageCommandManagerResponse
from robot_simple_command_manager_msgs.srv import GetCommandManagerList, GetCommandManagerListResponse
from actionlib_msgs.msg import GoalStatus
from rcomponent.rcomponent import *
from robot_simple_command_sequencer.sequence_list import SequenceList
from robot_simple_command_sequencer.command_manager_interface import CommandManagerInterface

try:
    from Queue import Queue, PriorityQueue
except ImportError:
    from queue import Queue, PriorityQueue


import yaml

"""
    QUEUE of SequenceList
        SequenceList:
                SEQ1 SEQ2 ... SQn [LOOP]
                    SEQx -> [CMD1 CMD2 .. CMDn]
"""

class SimpleCommandSequencer(RComponent):
    """
    A class used to load simple command handlers
    ...

    """

    def __init__(self):
        self.feedback_msg = CommandStringFeedback()
        self.feedback_action_msg = RobotSimpleCommandFeedback()
       
        # Available sequences of commands
        self.sequences_dict = {}
        # sequence list of commands in execution
        self.current_sequence_list = None
        self.current_sequence_list_id = ''
        self.current_sequence_list_remaining = 0
        # current command sent/in execution
        self.current_command = ''
        self.maxsize_queue = 100
        # Queue of sequence list to execute
        self.sequences_list_queue = PriorityQueue(maxsize=self.maxsize_queue)
        
        # Flag to enable/disable adding sequences into the queue
        self.allow_queuing = False
        # Contains the words cannot be used as sequence ids
        self._sequence_reserved_keywords = ['LOOP', 'PRIO']
        # Contains the string to publish in a topic the current sequence status
        self.current_sequence_string = '-'
        self._default_queue_priority = 100
        # Maximum value for the priority that would be accepted from a command
        self._max_queue_priority = 1
        # Priority applied when no priority is set
        self._current_priority = self._default_queue_priority
        # Flag of file sequences read
        self.sequences_already_read = False
        # Flag to enable the dequeue of the pending sequences. Use with caution
        self._enable_dequeue = True
        # Flag to enable the queue of new sequences. Use with caution
        self._enable_queue = True

        

        # log data
        self.log = CommandLog()

        # status publisher
        self.sequencer_status = SequencerStatus()

        self.commands_output = []

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)
        #self.sequences_dict = rospy.get_param('~sequences', {})
        self.sequences_file = rospy.get_param('~sequences_file', 'sequences_default.yaml')
        self.action_client_namespace = rospy.get_param('~action_client_namespace', 'command_manager')
        self.allow_queuing = rospy.get_param('~allow_queuing', False)
        self.validate_sequence_keywords()

        self.add_sequence_name = rospy.get_param('~add_sequence', '~add_sequence')
        rospy.loginfo('%s:: Using add sequence service: %s' % (self._node_name, self.add_sequence_name))
        self.remove_sequence_name = rospy.get_param('~remove_sequence', '~remove_sequence')
        rospy.loginfo('%s:: Using remove sequence service: %s' % (self._node_name, self.remove_sequence_name))
        self.get_sequences_name = rospy.get_param('~get_sequences', '~get_sequences')
        rospy.loginfo('%s:: Using get sequence service: %s' % (self._node_name, self.get_sequences_name))

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.command_sub = rospy.Subscriber('~command', CommandString, self.topic_cmd_cb)
        self.command_cancel_sub = rospy.Subscriber('~cancel', Empty, self.topic_cmd_cancel_cb)

        self.command_feedback_pub = rospy.Publisher(
            '~feedback', CommandStringFeedback, queue_size=1)

        self.current_sequence_publisher = rospy.Publisher(
            '~current_sequence', String, queue_size=1)

        self.status_publisher = rospy.Publisher(
            '~status', SequencerStatus, queue_size=1)

        self.command_service = rospy.Service('~command', SetCommandString, self.service_cmd_cb)
        self.command_cancel_service = rospy.Service('~cancel', Trigger, self.service_cmd_cancel_cb)

        self.command_action_server = actionlib.SimpleActionServer('~action', RobotSimpleCommandAction, None, False)
        self.command_action_server.register_goal_callback(self.action_goal_cb)
        self.command_action_server.register_preempt_callback(self.action_preempt_cb)
        self.command_action_server.start()
        # simple action client to call the local action server
        self.command_action_server_client = actionlib.SimpleActionClient('~action', RobotSimpleCommandAction)

        # self.action_client = actionlib.SimpleActionClient(self.action_client_namespace, RobotSimpleCommandAction)
        self.action_client = CommandManagerInterface(self.action_client_namespace)

        self.get_sequences_service = rospy.Service(self.get_sequences_name,
                                                  GetCommandManagerList,
                                                  self.get_sequences)

        self.add_sequence_service = rospy.Service(self.add_sequence_name,
                                                  ManageCommandManager,
                                                  self.add_sequence)

        self.remove_sequence_service = rospy.Service(self.remove_sequence_name,
                                                  ManageCommandManager,
                                                  self.remove_sequence)

        self.command_log_publisher = rospy.Publisher(
            '/robot/command_log', CommandLog, queue_size=1)

        return 0

    def load_sequences(self,file):
        try:
            self.sequences_dict = self.load_yaml_file(file)
            # remove empty spaces at the beginning and end of the string
            for sequence in self.sequences_dict:
                for i in  range(0, len(self.sequences_dict[sequence])):
                    self.sequences_dict[sequence][i] = self.sequences_dict[sequence][i].lstrip().rstrip()

            print (self.sequences_dict)
        except:
            rospy.logerr('%s::Unable to load sequences from: %s' % (self._node_name, self.sequences_file))
            return False

        for key in self.sequences_dict:
                self.sequencer_status.loaded_sequences.append(key)
        return True


    def save_yaml_file(self,data_dict,file):
        with open(file, 'w+') as f:
           yaml.dump(data_dict, f,default_flow_style=False)

    def load_yaml_file(self,file):
        with open(file) as f:
            # use safe_load instead load
            data = yaml.safe_load(f)
            return data

    def add_sequence(self, msg):
        ret = ManageCommandManagerResponse()

        ret.ret.success = True
        ret.ret.message = 'OK: Adding sequence %s' % (msg.Command.id)

        if msg.Command.id == '':
            ret.ret.success = False
            ret.ret.message ='Error: the id cannot be empty'
            rospy.logerr("%s",ret.ret.message)
            return ret

        if msg.Command.command == '':
            ret.ret.success = False
            ret.ret.message ='Error: the command cannot be empty'
            rospy.logerr("%s",ret.ret.message)
            return ret

        try:
            self.sequences_dict[msg.Command.id]= self.split_command_semicolon(msg.Command.command)
            self.save_yaml_file(self.sequences_dict,self.sequences_file)
        except :
            txt = '%s::Unable to add sequence' % (self._node_name)
            rospy.logerr("%s",txt)
            ret.ret.success=False
            ret.ret.message = txt

        rospy.loginfo("%s",ret.ret.message)
        return ret


    def remove_sequence(self, msg):

        ret = ManageCommandManagerResponse()

        if self.command_action_server.is_active() == True:
            ret.ret.success=False
            ret.ret.message = '%s::Unable to remove sequences if any sequence is active ' % (self._node_name)
        else:
            if msg.Command.id in self.sequences_dict:
                try:
                    del self.sequences_dict[msg.Command.id]
                    self.save_yaml_file(self.sequences_dict,self.sequences_file)
                    ret.ret.success=True
                    ret.ret.message='%s::OK Removing sequence %s' % (self._node_name,msg.Command.id)
                except :
                    ret.ret.success=False
                    ret.ret.message = '%s::Unable to remove sequence' % (self._node_name)
            else:
                ret.ret.success=False
                ret.ret.message = '%s::Unable to remove unexistent sequence %s' % (self._node_name,msg.Command.id)

        if ret.ret.success:
            rospy.loginfo("%s",ret.ret.message)
        else:
            rospy.logerr("%s",ret.ret.message)
        return ret


    def dict_to_list(self):
        l = CommandManagerArray()
        for seq in self.sequences_dict:
            C = CommandManager()
            C.id = seq
            bStart = True
            for c in self.sequences_dict[seq]:
                if (bStart):
                    C.command = c
                    bStart = False
                else:
                    C.command = C.command + " ; " + c

            l.commands.append(C)
        return l


    def get_sequences(self, msg):

        ret = GetCommandManagerListResponse()
        ret.list = self.dict_to_list()

        ret.success = True

        return ret

    def publish_log(self):
        self.log.node_name = self._node_name
        self.log.process_name = self.current_sequence_list_id
        self.log.command = self.current_command
        self.command_log_publisher.publish(self.log)

    def init_state(self):
        """ Actions perfomed in init state"""
        if not(self.sequences_already_read):
            self.load_sequences(self.sequences_file)
            self.sequences_already_read = True

        if self.action_client.wait_for_server(2) == True:
            self.switch_to_state(State.READY_STATE)
        else:
            rospy.loginfo_throttle(30, "%s::init_state: waiting for command server %s..." %
                                   (self._node_name, self.action_client.get_namespace()))


    def last_error(self,msg):
        if (msg==""):
            self.sequencer_status.last_error = ""
        else:
            self.sequencer_status.last_error = "Sequence: %s, Command: %s, Index: %d, Error: %s" % (
                                                                        self.sequencer_status.current_sequence.name,
                                                                        self.sequencer_status.current_sequence.current_command,
                                                                        self.sequencer_status.current_sequence.command_index,
                                                                        msg)
    def ready_state(self):
        """Actions performed in ready state"""
        
        # publish overall status
        self._publish_status()
        # command processing
        self._action_control_loop()



    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        self.command_sub.unregister()

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def topic_cmd_cb(self, msg):
        """Subscriber command_sub callback

        Forward the message to the action server

        Args:
            msg : CommandString object that contains the user command
        """
        if self._enable_queue == False:
            rospy.logerr('%s::topic_cmd_cb: command not accepted because the queue is disabled' %
                             (self._node_name))
            return

        if self._state != State.READY_STATE:
            rospy.logerr_throttle(5, '%s::topic_cmd_cb: command not accepted because the component is not READY' % (
                self._node_name))
            return

        if self.command_action_server.is_active() == True and self.allow_queuing == False:
            return

        is_valid, ret_msg, parsed_command = self.validate_command(msg.command)
        if is_valid == True:
            self.forward_command_to_local_action_server(msg.command)
        else:
            rospy.logerr_throttle(5, '%s::topic_cmd_cb: command %s is not valid: %s' % (
                self._node_name, msg.command, ret_msg))

    def topic_cmd_cancel_cb(self, msg):
        """Subscriber command_sub_cancel callback

        Forwards the cancel to the action server

        Args:
            msg : Empty object to cancel the current command
        """
        if self.command_action_server.is_active():
            self.cancel_command_of_local_action_server()

    def service_cmd_cb(self, msg):
        """Service command_service callback

        Forward the message to the send_cmd method

        Args:
            msg : SetCommandString object that contains the user command

        Return:
            An SetCommandStringResponse that conatins a bool and str
        """
        res = SetCommandStringResponse()

        if self._enable_queue == False:
            res.ret.success = False
            res.ret.code = StatusCodes.REJECTED
            res.ret.message = "New command not accepted because the queue is disabled"
            return res
            return

        if self._state != State.READY_STATE:
            res.ret.success = False
            res.ret.code = StatusCodes.REJECTED
            res.ret.message = "New command not accepted because the component is not READY"
            return res

        if self.command_action_server.is_active() == True and self.allow_queuing == False:
            res.ret.success = False
            res.ret.code = StatusCodes.REJECTED
            res.ret.message = "New command not allowed while another command is running"
            return res

        is_valid, ret_msg, parsed_command = self.validate_command(msg.command)
        if is_valid == True:
            self.forward_command_to_local_action_server(msg.command)
            res.ret.code = StatusCodes.ACTIVE
            res.ret.success = True
            res.ret.message = 'Command received and processed'
        else:
            res.ret.success = False
            res.ret.code = StatusCodes.REJECTED
            res.ret.message = ret_msg

        return res

    def service_cmd_cancel_cb(self, msg):
        """Service command_service_cancel callback

        Runs cancel_cmd method when recieves a cancel msg

        Args:
            msg : Trigger object to cancel the current command

        Return:
            A TriggerResponse object that contains a bool and a str
        """
        ret = TriggerResponse()

        if self.command_action_server.is_active():
            self.cancel_command_of_local_action_server()
            ret.success = True
            ret.message = 'Command correctly cancelled'
        else:
            ret.success = False
            ret.message = 'No command running to cancel'

        return ret

    def action_goal_cb(self):
        """Action server goal callback

        Accepts the new goal if not command running.  Rejects the incoming
        goal if a command is running
        """
        if self._enable_queue == False:
            self.command_action_server.accept_new_goal()
            rospy.logerr('%s::action_goal_cb: command not accepted because the queue is disabled' %
                             (self._node_name))
            return

        # No action running
        if self.command_action_server.is_active() == False:

            goal = self.command_action_server.accept_new_goal()
            
            # if Not ready -> reject
            if self._state != State.READY_STATE:
                rospy.logerr('%s::action_goal_cb: command not accepted because the component is not READY' %
                             (self._node_name))
                result = RobotSimpleCommandResult()
                result.result.command = goal.command.command
                result.result.success = False
                result.result.message = "Component Not Ready"
                result.result.code = StatusCodes.REJECTED
                self.command_action_server.set_aborted(result=result, text=result.result.message)
                return

            is_valid, ret_msg, parsed_command = self.validate_command(goal.command.command)
            if is_valid == True:
                ret, ret_msg = self.process_command(parsed_command)
                if ret == True:
                    rospy.loginfo('%s::action_goal_cb: command %s processed. Ret = (%s, %s)',
                                  self._node_name, goal.command.command, ret, ret_msg)
                else:
                    # Aborting the action
                    rospy.logerr('%s::action_goal_cb: command %s is not valid: %s' %
                                 (self._node_name, goal.command.command, ret_msg))
                    result = RobotSimpleCommandResult()
                    result.result.command = goal.command.command
                    result.result.success = False
                    result.result.message = ret_msg
                    result.result.code = StatusCodes.REJECTED
                    self.command_action_server.set_aborted(result=result, text=result.result.message)
            else:
                # Aborting the action
                rospy.logerr('%s::action_goal_cb: command %s is not valid: %s' %
                             (self._node_name, goal.command.command, ret_msg))
                result = RobotSimpleCommandResult()
                result.result.command = goal.command.command
                result.result.success = False
                result.result.message = ret_msg
                result.result.code = StatusCodes.REJECTED
                self.command_action_server.set_aborted(result=result, text=result.result.message)
        
        # Action running. Queue it?
        else:
            goal = self.command_action_server.accept_new_goal()

            if self.allow_queuing == True:
                is_valid, ret_msg, parsed_command = self.validate_command(goal.command.command)
                if is_valid == True:
                    ret, ret_msg = self.process_command(parsed_command)
                    if ret == True:
                        rospy.loginfo('%s::action_goal_cb: command %s processed. Ret = (%s, %s)',
                                      self._node_name, goal.command.command, ret, ret_msg)
                    else:
                        # Ignoring the action
                        rospy.logerr('%s::action_goal_cb: command %s is not valid: %s' %
                                     (self._node_name, goal.command.command, ret_msg))
                else:
                    # Not valid
                    rospy.logerr('%s::action_goal_cb: command %s is not valid: %s' %
                                 (self._node_name, goal.command.command, ret_msg))
            # do nothing -> discards the command
            else:
                rospy.logwarn(
                    '%s::action_goal_cb: New command not allowed while another command is running', self._node_name)

    def action_preempt_cb(self):
        """Action server preempt callback

        Cancels the current active goal or ignore the incoming goal if
        the preempt request has been triggered by new goal available.
        """

        if self.command_action_server.is_active():
            has_new_goal = self.command_action_server.is_new_goal_available()
            # If preempt request by new action
            if has_new_goal == False:
                self.cancel_cmd()
            else:
                rospy.logwarn(
                    '%s::action_preempt_cb: preemption due to a new command is not allowed', self._node_name)
        else:
            rospy.logwarn('%s::action_preempt_cb: No command is running', self._node_name)

    # END CALLBACKS TO MANAGE CLIENT INPUT

    def send_cmd(self, msg):
        """Sends the command to the command server

        Args:
            msg : String that contains the user command

        Return:
            Returns a tuple (bool, str) that shows if the command has been
            correctly sent
        """

        command_sent = True
        ret_msg = 'OK'

        goal = RobotSimpleCommandGoal()
        goal.command.command = msg

        try:
            self.action_client.send_goal(goal)
        except Exception as e:
            command_sent = False
            msg = 'Error sending command %s' % (msg)
            rospy.logerr('%s::send_cmd: %s' % (self._node_name, ret_msg))

        return (command_sent, ret_msg)

    def cancel_cmd(self):
        """Cancels the current command running in the remote action server and finish the local action

        Return:
            True if the command has been correctly cancelled.
            False otherwihse.
        """
        self.action_client.cancel()

        self.finish_action(StatusCodes.CANCELLED)

        rospy.logwarn('%s::cancel_cmd: Command %s correctly cancelled' %
                      (self._node_name, self.current_command))
        return True

    def split_command(self, command):
        """Splits the command from string to list of strings

        Splits the msg string to a list.

        Args:
            msg : string that contains the command

        Return:
            A list with the current command
        """
        return re.sub(' +', ' ', command).strip().split(" ")

    def split_command_semicolon(self, command):
        """Splits the command from string to list of strings

        Splits the msg string to a list.

        Args:
            msg : string that contains the command

        Return:
            A list with the current command
        """
        return re.sub(' +', ' ', command).strip().split(";")


    def update_feedback(self):
        """Updates feedback messages

        Updates both feedback_msg and feedback_action_msg with information
        provided by the current_handler
        """
        
        current_sequence_list = self.current_sequence_list.get_current_sequence_id()
        
        # Show the current iteration if it's in loop mode
        loop_cycles = self.current_sequence_list.get_loop_cycles()
        if loop_cycles > 0:
            current_sequence_list += '(%d)'%(loop_cycles)
        
        self.feedback_msg.command = '%s:%s' % (current_sequence_list, self.current_command)
        try:
            action_client_feedback = self.action_client.get_feedback()
            self.feedback_msg.status = action_client_feedback.feedback.status
        except:
            self.feedback_msg.status = "Error"

        self.feedback_action_msg.feedback = self.feedback_msg

    def publish_feedback(self):
        """Publishes the updated feedback
        """
        self.command_feedback_pub.publish(self.feedback_msg)
        self.command_action_server.publish_feedback(self.feedback_action_msg)

    def reject_goal(self, command):
        result = RobotSimpleCommandResult()
        result.result.command = command  # self.current_command
        result.result.success = False
        result.result.message = 'This goal has been rejected. Probably because ' +\
            'other command is running. Cancel before ' +\
            'sending other goal.'
        result.result.code = StatusCodes.REJECTED

        return

    def finish_action(self, code, msg=''):
        """Finishes the current action

        Args:
            code : int value codified as StatusCodes object
            msg: string to set the message of the result
        """
        result = RobotSimpleCommandResult()
        result.result.command = self.current_sequence_list_id
        result.result.message = msg
        result.result.code = code

        if code == StatusCodes.CANCELLED:
            result.result.success = False
            self.command_action_server.set_preempted(result=result, text=result.result.message)
        elif code == StatusCodes.SUCCEEDED:
            result.result.success = True
            self.command_action_server.set_succeeded(result=result, text=result.result.message)
        elif code == StatusCodes.FAILED:
            result.result.success = False
            self.command_action_server.set_aborted(result=result, text=result.result.message)
        else:
            result.result.success = False
            rospy.logwarn(
                '%s::finish_action: the code passed is not being handled! Setting the action as aborted', self._node_name)
            self.command_action_server.set_aborted(result=result, text=result.result.message)

        self.clear_command()

    def clear_command(self):
        """Reset some internal variables"""
        self.feedback_msg = CommandStringFeedback()
        self.feedback_action_msg = RobotSimpleCommandFeedback()
        self.current_command = ''
        self.current_sequence_list = None
        self.sequences_list_queue = PriorityQueue(self.maxsize_queue)

    def forward_command_to_local_action_server(self, command):
        """Sends the command to itself to the action server

        Args:
            command : string

        Return:
            0 if OK
            -1 if error
        """
        goal = RobotSimpleCommandGoal()
        goal.command.command = command
        self.command_action_server_client.send_goal(goal)

        return 0

    def cancel_command_of_local_action_server(self):
        """Cancels current action

        Args:


        Return:
            0 if OK
            -1 if error
        """

        self.command_action_server_client.cancel_all_goals()

        return 0

    def validate_command(self, command):
        """Checks if the command is valid (tag exists, number of arguments is correct)

        Args:
            command: string with command

        Return:
            Returns a tuple of (bool, msg, string[])
            True,msg,[parsed_command] if it is valid
            False,msg,[] if it is NOT valid
        """
        parsed_command = self.split_command(command)
        keyword_counter = 0
        priority = self._default_queue_priority
        is_loop = False
        loop_iterations = -1
        commands_without_reserved = []

        # look for reserved keywords
        for cmd in parsed_command:
            if cmd == 'LOOP':
                keyword_counter += 1
            elif cmd.startswith('LOOP') and cmd.split('LOOP')[1].isdigit():
                loop_iterations = int(cmd.split('LOOP')[1])
                keyword_counter += 1
            elif cmd.startswith('PRIO') and cmd.split('PRIO')[1].isdigit():
                priority = cmd.split('PRIO')[1]
                keyword_counter += 1
            else:
                commands_without_reserved.append(cmd)

        # check if it is a simple command         
        if self.action_client.is_command_valid(commands_without_reserved) == False:
            for cmd in commands_without_reserved:
                if cmd not in self.sequences_dict:
                # if self.sequences_dict.has_key(cmd) == False:
                    rospy.logerr('%s::validate_command: command %s is not valid', self._node_name, str(parsed_command))
                    return False, "ERROR", []  
        
        if len(parsed_command) < 1 + keyword_counter:
            return False, "ERROR", []

        if is_loop == True and len(parsed_command) < 2:
            return False, "ERROR", []

        return True, "OK", parsed_command

    def process_command(self, command):
        """Processes the command and put it in the execution queue

        Args:
            command: string[] with valid sequences

        Return:
            Returns a tuple of (bool, msg)
            True,msg if it is valid
            False,msg if it is NOT valid
        """
        keyword_counter = 0
        priority = self._current_priority
        self._current_priority += 1
        is_loop = False
        loop_iterations = -1
        commands_without_reserved = []

        # look for reserved keywords
        for cmd in command:
            if cmd == 'LOOP': 
                keyword_counter += 1
                is_loop = True
            elif cmd.startswith('LOOP') and cmd.split('LOOP')[1].isdigit():
                loop_iterations = int(cmd.split('LOOP')[1])
                is_loop = True
                keyword_counter += 1
            elif cmd.startswith('PRIO') and cmd.split('PRIO')[1].isdigit():
                priority = int(cmd.split('PRIO')[1])
                keyword_counter += 1
            else:
                commands_without_reserved.append(cmd)
        
        # check if it is a simple command         
        if self.action_client.is_command_valid(commands_without_reserved) == True:
            command_list = [' '.join(command)]
            sequence_list = SequenceList(command_list)
            # The name of the sequence contains all the keywords (including reserved ones)
            # The command does not have to contain any reserved keyword
            sequence_list.append(str(command_list), [' '.join(commands_without_reserved)]) 
            rospy.loginfo('%s::process_command: command %s as sequence', self._node_name, str(command_list))
        # check if the sequence is correct         
        else:
            try:
                sequence_list = SequenceList(command)
                for cmd in commands_without_reserved:   
                    # check if it is a current sequence
                    command_list = self.sequences_dict[cmd]
                    sequence_list.append(cmd, command_list)
                    rospy.loginfo('%s::process_command: sequence %s -> %s', self._node_name, cmd, str(command_list))               
            except KeyError as e:
                    return False, "Sequence %s is not valid" % (cmd)

        if is_loop == True:
            if loop_iterations == -1:
                sequence_list.set_loop(True)
            else:
                sequence_list.set_loop(True, loop_iterations)

        # At least one
        if len(command) < 1 + keyword_counter:
            return False, "Empty command"

        if sequence_list.is_loop() == True and len(command) < 2:
            return False, "Empty command"

        try:
            self.sequences_list_queue.put((priority, sequence_list))
        except Queue.Full as e:
            return False, "Queue of sequences is full"

        return True, "OK"


    def validate_sequence_keywords(self):
        """
            Checks that the sequences id are not reserved keywords
            In case there were, it removes them
        """
        keys_to_remove = []
        for sequence in self.sequences_dict:
            if sequence in self._sequence_reserved_keywords:
                keys_to_remove.append(sequence)

        for sequence in keys_to_remove:
            rospy.logerr(
                '%s::validate_sequence_keywords: the sequence %s is a keyword that cannot be used. It has been removed', self._node_name, sequence)
            del self.sequences_dict[sequence]
    
    def _publish_status(self):
        """
            Gathers and publish the current status
            It should be called in ready_state
        """

        self.current_sequence_string = '-Seq_id: %s Command: %s Remaining Commands:%d ' % (self.current_sequence_list_id,
                                                                                           self.current_command,
                                                                                           self.current_sequence_list_remaining)
        self.current_sequence_publisher.publish(self.current_sequence_string)

        self.sequencer_status.current_sequence.name=self.current_sequence_list_id
        self.sequencer_status.current_sequence.current_command = self.current_command
        queued_sequences = list(self.sequences_list_queue.queue)
        # print str(queued_sequences)
        self.sequencer_status.queued_sequences = []
        for q in queued_sequences:
            self.sequencer_status.queued_sequences.append(q[1].get_id())

        if self.current_sequence_list == None:
            self.sequencer_status.current_sequence.commands = []
            self.sequencer_status.current_sequence.command_index = 0
        else:
            self.sequencer_status.current_sequence.commands = self.current_sequence_list.get_current_commands()
            self.sequencer_status.current_sequence.command_index = self.current_sequence_list.get_current_index()

        self.status_publisher.publish(self.sequencer_status)


    def _action_control_loop(self):
        """
            Performs all the logic related to the action server, dequeue, command execution and control
            It should be called in ready_state
        """
        # Action server running
        if self.command_action_server.is_active() == True:                  
            
            # No sequence active
            if self.current_sequence_list == None:                           
                self.commands_output = []
                # There are pending sequences in the queue
                if self.sequences_list_queue.empty() == False:   

                    if self._enable_dequeue == True:          
                        sequence_priority, self.current_sequence_list = self.sequences_list_queue.get()
                        self.current_sequence_list_id = self.current_sequence_list.get_id()
                        self.current_sequence_list_remaining = self.current_sequence_list.get_count_remaining_commands()
                        rospy.loginfo('%s::_action_control_loop: Getting new sequence (%s)(prio %d) from the queue. %d remaining in the queue',
                                    self._node_name, self.current_sequence_list_id, int(sequence_priority), self.sequences_list_queue.qsize())
                        self.log.status = "Getting Next Sequence"
                        self.log.result = "OK"
                        self.log.error = False
                    else:
                        rospy.loginfo_throttle(10, '%s::_action_control_loop: dequeue is disabled'%self._node_name)
                        self.log.status = "Dequeue is disabled"
                        self.log.result = "OK"
                        self.log.error = False

                # No more sequences: FINISH!
                else:      
                    rospy.loginfo('%s::_action_control_loop: the sequence queue is empty!', self._node_name)
                    self.finish_action(StatusCodes.SUCCEEDED)
                    self.log.status = "No More Sequences"
                    self.log.result = "OK"
                    self.log.error = False
            
            # Ongoing sequence
            else:
                # No commands in execution
                if self.current_command == '':

                    current_command = self.current_sequence_list.get_next_command()

                    if current_command != None:
                        # Check if command expects previous command output
                        if self.sequencer_status.current_sequence.command_index > 0:
                            command_inputs = re.findall("[$]\w+", current_command)
                            if len(command_inputs) != 0:
                                for input in command_inputs:
                                    command_input_index = int(input[1:])
                                    current_command = current_command.replace(\
                                                        input, self.commands_output[command_input_index])

                        self.current_command = current_command
                        # SEND THE ACTION
                        ret, ret_msg = self.send_cmd(self.current_command)
                        if ret == True:
                            rospy.loginfo('%s::_action_control_loop: sending new command: %s',
                                          self._node_name, self.current_command)
                            self.log.status = "Send"
                            self.log.result = "OK"
                            self.log.error = False
                            self.last_error("")
                        else:
                            rospy.logerr('%s::_action_control_loop: error sending command: %s. Finishing sequence',
                                         self._node_name, self.current_command)
                            self.log.status = "End"
                            self.log.result = "Error Sending Command:"+ret_msg
                            self.log.error = True
                            self.finish_action(StatusCodes.FAILED)
                            self.last_error("Error Sending Command")

                    else:  # The sequence is finished

                        # If the sequence is in loop
                        if self.current_sequence_list.is_loop() == True:
                            self.current_sequence_list.reset()
                            self.log.status = "Loop"
                            self.log.result = ""
                            self.log.error = False
                            rospy.logwarn('%s::_action_control_loop: the sequence (%s) has finished but it is in loop mode! (%d cycles)',
                                          self._node_name, self.current_sequence_list.get_id(), self.current_sequence_list.get_loop_cycles())
                        else:
                            rospy.loginfo('%s::_action_control_loop: the sequence (%s) has finished!',
                                          self._node_name, self.current_sequence_list.get_id())
                            self.log.status = "End Sequence"
                            self.log.result = "End Sequence"
                            self.log.error = False
                            self.current_sequence_list = None
                # Waits  for the end of the command
                else:   
                    self.update_feedback()
                    self.publish_feedback()
                    self.current_sequence_list_remaining = self.current_sequence_list.get_count_remaining_commands()
                    rospy.loginfo_throttle(5, '%s::_action_control_loop: waiting for the end of the command: %s' %
                                           (self._node_name, self.current_command))

                    action_client_state = self.action_client.get_state()
                    action_client_result = self.action_client.get_result()
                    communication_timeout = self.action_client.is_timed_out()

                    if communication_timeout == True:
                        rospy.logerr('%s::_action_control_loop: Timeout getting feedback from the client. Command %s. Finishing sequence',
                                     self._node_name, self.current_command)
                        self.finish_action(StatusCodes.FAILED, msg='Timeout: No response from the action client')
                        self.log.status = "End"
                        self.log.result = "Timeout Error"
                        self.log.error = True
                        self.last_error("Timeout Error")
                 
                    elif action_client_state != GoalStatus.ACTIVE and action_client_state != GoalStatus.PENDING:
                        
                        if action_client_result == None:  # If nothing is received from get_result
                            if action_client_state == GoalStatus.SUCCEEDED:
                                rospy.loginfo('%s::_action_control_loop: the command (%s) has finished correctly!',
                                              self._node_name, self.current_command)
                                self.current_command = ''
                                self.log.status = "End"
                                self.log.result = "End Sequence"
                                self.log.error = False
                            else:
                                rospy.logerr('%s::_action_control_loop: the command (%s) has finished wrongly (goal_status = %d)!',
                                             self._node_name, self.current_command, action_client_state)
                                self.finish_action(StatusCodes.FAILED, msg="goal_status = %d" % (action_client_state))
                                self.log.status = "End"
                                self.log.result = "Command Finished wrongly"
                                self.log.error = True
                                self.last_error("Finished wrongly code:%d msg:%s!" % (action_client_result.result.code, action_client_result.result.message))

                        elif action_client_result.result.success == True:
                            rospy.loginfo('%s::_action_control_loop: the command (%s) has finished correctly!',
                                          self._node_name, self.current_command)
                            self.commands_output.append(action_client_result.result.output)
                            self.current_command = ''
                            self.log.status = "End"
                            self.log.result = "End Sequence"
                            self.log.error = False
                        else:
                            rospy.logerr('%s::_action_control_loop: the command (%s) has finished wrongly (code = %d, msg = %s)!',
                                         self._node_name, self.current_command, action_client_result.result.code, action_client_result.result.message)
                            self.finish_action(StatusCodes.FAILED, msg=action_client_result.result.message)
                            self.log.status = "End"
                            self.log.result = "Command Finished wrongly"
                            self.log.error = True
                            self.last_error("Finished wrongly code:%d msg:%s!" % (action_client_result.result.code, action_client_result.result.message))
            self.publish_log()
        
        # No action running
        else:
            self.publish_log()
            self.log.status = ""
            self.log.result = ""
            self.log.error = False
            self.current_sequence_list_id=''
    
    def _set_dequeue(self, value):
        """
            Enables/Disables the extraction from the queue
            param value as bool
        """
        if self._enable_dequeue != value:
            rospy.loginfo('%s::_set_dequeue: setting enable_dequeue to %s', self._node_name, str(value))
            self._enable_dequeue = value
    
    def _set_queue(self, value):
        """
            Enables/Disables the insertion to the queue
            param value as bool
        """
        if self._enable_queue != value:
            rospy.loginfo('%s::_set_queue: setting enable_dequeue to %s', self._node_name, str(value))
            self._enable_queue = value

