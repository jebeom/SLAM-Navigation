#!/usr/bin/env python
import rospy

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from robot_simple_command_manager_msgs.msg import RobotSimpleCommandAction, RobotSimpleCommandGoal, CommandManagerStatus, StatusCodes
from robotnik_urcap_bridge_msgs.msg import CommandState 
from rcomponent.rcomponent import *
from robot_local_control_msgs.msg import Status
from poi_manager_msgs.srv import GetPOIs, GetPOIsRequest
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

from xmlrpc_server.rc_server import XMLRPCServer
from rospy_message_converter import message_converter
from ast import literal_eval

class URCapCommandBridge(XMLRPCServer):
    def __init__(self):
        XMLRPCServer.__init__(self)

    def ros_read_params(self):
        XMLRPCServer.ros_read_params(self)

        self.kinematics = "diff"
        self.kinematics = rospy.get_param('~kinematics', self.kinematics)

        self.command_manager_ns = 'command_manager'
        self.command_manager_ns = rospy.get_param('~command_manager_ns', self.command_manager_ns)

        self.result_topic_name = 'result'

        self.rlc_msg = Status()
        
        # Command manager info 
        self.command_running = False
        self.command_sent = False
        self.result_code = CommandState.ACTIVE
        self.last_command = "Unknown"

        self.paused = False
        return 0

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)
    
        self.result_pub = rospy.Publisher('~' + self.result_topic_name, Int16, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)
        self.rlc_status_sub = rospy.Subscriber('robot_local_control/state', \
                            Status, self.rlc_status_cb)
        self.cm_status_sub = rospy.Subscriber(self.command_manager_ns + "/status", \
                            CommandManagerStatus, self.cm_status_cb) 
        self.get_pois_client = rospy.ServiceProxy("poi_manager/get_poi_list", GetPOIs)

        self.client = SimpleActionClient(self.command_manager_ns + "/action", RobotSimpleCommandAction)
    
    def setup(self):
        self.feedback_msg = "No feedback received :c"
        self.server.register_function(self.get_command_result, 'robot.command_result.result')
        self.server.register_function(self.get_feedback, 'robot.command_result.feedback')

        self.server.register_function(self.is_command_running, 'robot.command_manager.is_active')
        self.server.register_function(self.get_last_command, 'robot.command_manager.last_command')
        
        self.server.register_function(self.send_command, 'robot.command_manager.command')
        self.server.register_function(self.cancel_command, 'robot.command_manager.cancel')
        
        
        self.server.register_function(self.get_pois, 'robot.get_pois')

        # Pause/Resume interaction
        self.server.register_function(self.pause, 'robot.pause')
        self.server.register_function(self.pause, 'robot.pause_robot')
        self.server.register_function(self.is_paused, 'robot.is_paused')
        self.server.register_function(self.resume, 'robot.resume')
        self.server.register_function(self.resume, 'robot.resume_robot')

        # Retrieve localization information
        self.server.register_function(self.is_localization_reliable, 'robot.is_localization_reliable')
        self.server.register_function(self.get_environment, 'robot.environment')
        self.server.register_function(self.get_localization_type, 'robot.localization_type')

        # Retrieve battery information
        self.server.register_function(self.get_battery_level, 'robot.battery_level')
        self.server.register_function(self.get_consumption, 'robot.consumption')
        self.server.register_function(self.get_is_charging, 'robot.is_charging')

        # Retrive safety information
        self.server.register_function(self.get_safety_mode, 'robot.safety_mode')
        self.server.register_function(self.get_is_on_safety_stop, 'robot.is_on_safety_stop')

        # Retrive kinematics information
        self.server.register_function(self.get_kinematics, 'robot.kinematics')
        self.server.register_function(self.is_omni, 'robot.is_omni')

        # Retrieve robot state
        self.server.register_function(self.get_is_free, 'robot.is_free')
        self.server.register_function(self.get_robot_state, 'robot.robot_state')

        XMLRPCServer.setup(self)

        return 0
    
    def get_feedback(self):
        return self.feedback_msg

    def get_command_result(self):
        return self.result_code

    def is_command_running(self):
        return self.command_running

    def cm_status_cb(self, msg):
       self.last_command = msg.command
       self.command_running = (msg.code == "ACTIVE")
       #print("REAL CODE: ", msg.code, type(msg.code), "DESIRED CODE: ", StatusCodes.ACTIVE, type(StatusCodes.ACTIVE))

    def get_last_command(self):
       return self.last_command

    def send_command(self, msg):
        if self.command_running == True:
           return False

        if type(msg) == str:
            msg = literal_eval(msg)
        
        goal = {"command": msg}
        goal_type_name = 'robot_simple_command_manager_msgs/RobotSimpleCommandGoal'
        goal_rosmsg = message_converter.convert_dictionary_to_ros_message(goal_type_name, goal)

        self.client.send_goal(goal_rosmsg)
        rospy.sleep(0.5)
        self.command_sent = True

        return True
    
    def cancel_command(self):
        self.client.cancel_goal()
        return True

    def pause(self):
        self.paused = True
        return True

    def is_paused(self):
        return self.paused

    def resume(self):
        self.paused = False
        return True

    def ready_state(self):
        if self.paused == True and self.command_running == True and self.command_sent == True:
            self.cmd_vel_pub.publish(Twist())


        if self.command_sent == True:
            action_status = self.client.get_state()
            if action_status == GoalStatus.ACTIVE:
                self.result_code = CommandState.ACTIVE
                self.result_pub.publish(self.result_code)
                rospy.loginfo_throttle(1, "Command currently running")
                return
            else:
                result = self.client.get_result()

                self.command_sent = False
                self.result_code = action_status
                self.feedback_msg = result.result.message

                self.result_pub.publish(self.result_code)
                rospy.loginfo_throttle(1, "Command not running or finished: %s", self.feedback_msg)
    
    def shutdown(self):
        XMLRPCServer.shutdown(self)

    def rlc_status_cb(self, msg):
        self.rlc_msg = msg

    def get_pois(self):
        request = GetPOIsRequest()
        request.environment = self.rlc_msg.localization_status.environment
        response = self.get_pois_client(request)

        pois = response.p_list
        pois_names = []
        for poi in pois:
            pois_names.append(poi.name)
        
        return pois_names
    
    def is_localization_reliable(self):
        return self.rlc_msg.localization_status.reliable
    
    def get_environment(self):
        return self.rlc_msg.localization_status.environment

    def get_localization_type(self):
        return self.rlc_msg.localization_status.type

    def get_battery_level(self):
        return int(self.rlc_msg.robot_status.battery.level)
    
    def get_consumption(self):
        return str(self.rlc_msg.robot_status.battery.current)
    
    def get_is_charging(self):
        return self.rlc_msg.robot_status.battery.is_charging
    
    def get_safety_mode(self):
        return self.rlc_msg.robot_status.safety_status.safety_mode
     
    def get_is_on_safety_stop(self):
        return self.rlc_msg.robot_status.safety_status.safety_stop
    
    def get_kinematics(self):
        return self.kinematics
    
    def is_omni(self):
        return self.kinematics == "omni"
    
    def get_is_free(self):
        # Robot is free if all components are ready and is not executing a command

        # Get if RLC components are ready
        nav_state = self.rlc_msg.navigation_status.state
        nav_state = nav_state.lower()
        nav_ready = nav_state == self.rlc_msg.navigation_status.NAVIGATION_STATE_READY

        loc_state = self.rlc_msg.localization_status.state
        loc_state = loc_state.lower()
        loc_ready = loc_state == self.rlc_msg.localization_status.LOCALIZATION_STATE_READY

        rlc_ready = nav_ready and loc_ready

        # No command running
        no_running = not self.is_command_running()
        free = rlc_ready and no_running

        return free
    
    def get_robot_state(self):
        free = self.get_is_free()
        if self.paused == True:
            return "paused"
        
        if free == True:
            return "ready"
        
        running = self.is_command_running()
        if running == True:
            return "executing"
        
        return "not_ready"
