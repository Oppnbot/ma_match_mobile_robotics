#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

def print_message():
    rospy.loginfo("Hello World, what a day to be alive! :)")

if __name__ == '__main__':
    rospy.init_node('print_node')
    rate = rospy.Rate(10) # Hz
    rospy.loginfo("PRINT NODE namespace: " + rospy.get_namespace())
    #rospy.spin()
    while not rospy.is_shutdown():
        print_message()
        rate.sleep()