#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cartographer_pbstream_creator import CartographerPbstreamCreator


def main():

    rospy.init_node("cartographer_pbstream_creator_node")

    rc_node = CartographerPbstreamCreator()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
