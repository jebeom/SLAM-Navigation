#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gps_filter import GPSFilter


def main():

    rospy.init_node("gps_filter_node")

    rc_node = GPSFilter()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
