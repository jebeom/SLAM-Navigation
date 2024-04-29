#!/usr/bin/env python

import rospy
from ping_time_monitor.msg import PingStatus
from rosmon_msgs.srv import StartStop
import os
import rospkg

args = {}
rosmon_start_stop_service = None

def ping_status_callback(data):
    rosmon_start_stop_service = rospy.ServiceProxy('/' + args['rosmon_node'] + '/start_stop', StartStop)
    if data.packet_loss >= 1.0:
        rospy.logwarn("Lost some packets. Packet loss: %f", data.packet_loss)
        if data.packet_loss >= args['packet_loss_stop_threshold']:
            rospy.logerr("Lost more than %f percent of packages. Stopping rosmon instance %s", args['packet_loss_stop_threshold'], args['rosmon_node'])
            rosmon_start_stop_service('', '', 2)
    if data.packet_loss <= args['packet_loss_start_threshold']:
        rospy.loginfo("Lost less than %f percent of packages. Starting rosmon instance %s", args['packet_loss_start_threshold'], args['rosmon_node'])
        rosmon_start_stop_service('', '', 1)

def main():
    rospy.init_node('node_manager')

    rospack = rospkg.RosPack()

    arg_defaults = {
      'ping_time_monitor_topic_name': '/ping_time_monitor/status',
      'packet_loss_stop_threshold': 100.0,
      'packet_loss_start_threshold': 0.0,
      'rosmon_node': 'rosmon_node_name',
    }

    for name in arg_defaults:
        try:
            if rospy.search_param(name):
                args[name] = rospy.get_param('~%s'%name)
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerror('%s: %s'%(e, _name))

    rospy.Subscriber(args['ping_time_monitor_topic_name'], PingStatus, ping_status_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
