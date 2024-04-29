#!/usr/bin/env python
from ..command_service_interface import *

from robot_local_control_msgs.msg import LocalizationStatus


class SetEnvironmentServiceInterface(CommandServiceInterface):
    def __init__(self, name, parameters):
        self.args_description = ["environment_name"]
        self.args_types = [str]
        self.args_void_allowed = [False]
        self.default_values = []
        self.output_types = []
        self.output_description = []
        
        self.localization_sub = None
        self.target_environment = ""
        self.last_state = ""
        # should pass by standby state always
        self.standby_transition = False
        self.localization_status_msg = None
        self.is_not_active_time = None
        self.is_not_active_hysteresis = 2.0
        self.resend_command_timer = 5.0
        self.max_number_of_trials = 5
        self.trials = 0

        CommandServiceInterface.__init__(self, name, parameters)

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandServiceInterface.set_parameters(self, parameters)

    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''
        self.localization_sub = rospy.Subscriber("robot_local_control/LocalizationComponent/status", LocalizationStatus, self.localization_cb)
        self.last_state = ""
        self.standby_transition = False
        self.is_not_active_time = None
        self.trials = 0

        if type(args) == list:
            self.target_environment = args[0]
            self.request.environment = args[0]


    def parse_feedback(self):
        '''
            Parse from feedback object to string
        '''
        return self.feedback_msg.message
    
    def has_succeeded(self):
        #print str(self.result_msg)
        if self.result_msg.success == False:
            return False

        self.localization_sub.unregister()
        return (self.localization_status_msg.state == "READY") and (self.standby_transition == True) and (self.localization_status_msg.environment == self.target_environment)

    def is_active(self):
        
        if self.result_msg.success == False:
            rospy.sleep(1)
            if self.trials <= self.max_number_of_trials:
                self.resend_command()
                self.trials += 1
                rospy.logwarn('Failed calling the service. Retrying command %s (%d of %d)'
                            % (self.name, self.trials, self.max_number_of_trials))
                return True
            return False
        
        on_emergency = False
        ret = True
        if self.localization_status_msg != None:
            #self.feedback_cb('state: %s, type: %s, environment: %s'%(self.localization_status_msg.state, self.localization_status_msg.type, self.localization_status_msg.environment))
            self.feedback_msg.message = 'state: %s, type: %s, environment: %s'%(self.localization_status_msg.state, self.localization_status_msg.type, self.localization_status_msg.environment)
            self.result_msg.message = self.feedback_msg.message
            self.feedback_str = self.parse_feedback()
            if self.localization_status_msg.state == "FAILURE" or self.localization_status_msg.state == "EMERGENCY":
                ret = False
                on_emergency = True
                #rospy.loginfo('on_emergency')
            elif (self.standby_transition == True) and (self.localization_status_msg.state == "READY") and (self.localization_status_msg.environment == self.target_environment):
                #rospy.loginfo('%s, %s, %s'%(self.standby_transition,self.localization_status_msg.state, self.localization_status_msg.environment))
                ret = False

        
        # Apply an hysteresis to the composed state before deciding it's not active
        if ret == False:
            if self.is_not_active_time == None:
                self.is_not_active_time = rospy.Time.now()
                ret = True
            elif (rospy.Time.now() - self.is_not_active_time).to_sec() < self.is_not_active_hysteresis:
                ret = True
                #rospy.loginfo('< hysteresis')
        else:
            # reset timer
            self.is_not_active_time = None

        if ret == False and on_emergency == True and self.trials <= self.max_number_of_trials: #it didn't finish correctly 
            #rospy.loginfo('resend?')
            if (rospy.Time.now() - self.is_not_active_time).to_sec() >= self.resend_command_timer:
                # lets try to send it again
                self.resend_command()
                self.trials += 1
                rospy.logwarn('Retrying command %s (%d of %d)'
                            % (self.name, self.trials, self.max_number_of_trials))
                self.is_not_active_time = None                          
                ret = True
            else:
                ret = True
        return ret
        '''succeed = self.last_state == "READY"
        error = self.last_state == "ERROR"
        return succeed == False and error == False'''

    def localization_cb(self, msg):
        self.localization_status_msg = msg     
        if self.last_state == "STANDBY" and msg.state == "READY":
            self.standby_transition = True  
        #if msg.environment == self.target_environment:
        self.last_state = msg.state
        return

    def get_result(self):
        return self.result_msg.message

    def resend_command(self):
        '''
            Return True if the command has been sent
        '''
        tmp = self.client.call(self.request)
        self.feedback_cb(tmp)
        return True,""
