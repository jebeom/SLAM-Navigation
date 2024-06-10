#!/usr/bin/env python
import sys
import rospy

if sys.version_info.major == 3:
    from robot_simple_command_manager.simple_command_manager import SimpleCommandManager
else:
    from simple_command_manager import SimpleCommandManager


def main():

    rospy.init_node("command_manager_node")

    rc_node = SimpleCommandManager()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
