#!/usr/bin/env python
import rospy
from urcap_command_bridge import URCapCommandBridge

def main():

   rospy.init_node("urcap_command_bridge_node")


   _name = rospy.get_name().replace('/','')

   command_result = URCapCommandBridge()
   rospy.loginfo('%s: starting'%(rospy.get_name()))
   command_result.start()


if __name__ == "__main__":
    main()