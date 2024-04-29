#!/usr/bin/env python
from ..command_action_interface import *

from robot_local_control_msgs.msg import Pose2DStamped, LocalizationStatus
from poi_manager_msgs.srv import GetPOI, GetPOIRequest
from robotnik_msgs.msg import ptz as Ptz
from std_msgs.msg import Float64, Int32

class PtzTagActionInterface(CommandActionInterface):
    def __init__(self, name, parameters):
        self.client = None
        self.feedback_msg = None
        self.feedback_str = ''

        self.name = name
        self.set_parameters(parameters)

        self.args_description = ['tag_name']
        self.args_types = [str]
        self.args_void_allowed = [False]
        self.default_values = []
        self.output_types = []
        self.output_description = []
        
        self.localization_sub = rospy.Subscriber("robot_local_control/LocalizationComponent/status", LocalizationStatus, self.localization_cb)

    def set_parameters(self, parameters):

        '''
            Set all the required parameters of the interface
        '''
        

        self.check_parameters(parameters)

        self.parameters = parameters

        self.ros_control = self.get_parameter('ros_control', False)

        self.get_poi_ns = self.get_parameter('get_poi_ns', 'poi_manager/get_poi')
        self.frame_id = self.get_parameter('frame_id', 'robot_map')
        self.environment = self.get_parameter('default_environment', 'unknown')

        self.pan_joint = self.get_parameter('pan_joint', 'ptz_pan_joint')
        self.tilt_joint = self.get_parameter('tilt_joint', 'ptz_tilt_joint')
        self.zoom_joint = self.get_parameter('zoom_joint', 'ptz_zoom_joint')

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

    def get_joint_value(self, list_of_joint_dicts, joint_name, is_zoom):

        success = False
        value = 0

        for i in range(0, len(list_of_joint_dicts)):

            joint_dict = list_of_joint_dicts[i]

            if joint_name in joint_dict.name:
                value = joint_dict.position
                success = True

        if not success and is_zoom:
            rospy.logwarn("Ignoring zoom "+joint_name+" joint because does not exist in poi file. Probably the PTZ has not zoom.")

        elif not success:
            self.raise_exception("Could not get position from "+joint_name+", joint not found in poi file")

        return value

    def send_command(self, args):
        '''
            Return True if the command has been sent
        '''
        parsed_arguments = self.parse_args(args)

        client = rospy.ServiceProxy(self.get_poi_ns, GetPOI)
        request = GetPOIRequest()
        request.name = args[0]
        request.environment = self.environment
        response = client.call(request)
        if not response.success:
            self.raise_exception("Could not get pose from POI "+args[0]+" ns = "+self.get_poi_ns+", environment = "+self.environment)

        pan_value  = self.get_joint_value(response.p.joints, self.pan_joint, False)
        tilt_value = self.get_joint_value(response.p.joints, self.tilt_joint, False)
        zoom_value = self.get_joint_value(response.p.joints, self.zoom_joint, True)
        
        if self.ros_control == False:

            ptz = Ptz()
            ptz.pan  = pan_value
            ptz.tilt = tilt_value
            ptz.zoom = zoom_value
            ptz.relative = False
            ptz.mode = 'position'
            self.ptz_pub.publish(ptz)

            # print(ptz.pan)
            # print(ptz.tilt)
            # print(ptz.zoom)
            # print("------")

        else:

            pan = Float64()
            pan.data = pan_value
            self.pan_pub.publish(pan)

            tilt = Float64()
            tilt.data = tilt_value
            self.tilt_pub.publish(tilt)

            zoom = Int32()
            zoom.data = zoom_value
            self.zoom_pub.publish(zoom)

            # print(pan.data)
            # print(tilt.data)
            # print(zoom.data)
            # print("++++++++")

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

    def localization_cb(self, msg):
        self.environment = msg.environment

    def parse_feedback(self):
        '''
            Parse from feedback object to string 
        '''
        # DELETE THIS BLOCK AFTER IMPLEMENTING YOUR HANDLER
        # FEEDBACK_MSG HAS THE SAME STRUCTURE AS THE ACTION FEEDBACK MSG
        # THAT USES THE ACTION SERVER DEFINED BY THE NAMESPACE OF THE HANDLER
        # DEFINE THE RETURN VALUE BASED ON THE ACTION FEEDBACK
        return #str(self.feedback_msg)