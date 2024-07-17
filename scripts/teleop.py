#!/usr/bin/env python

# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import argparse

import rospy
import sys, select, termios, tty
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def arm(args, state):
    try:
        arming_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/arming", CommandBool)
        ret = arming_cl(value=state)
    except rospy.ServiceException as ex:
        print(ex)

    if not ret.success:
        rospy.loginfo("ARM Request failed.")
    else:
        rospy.loginfo("ARM Request success.")

def takeoff(args):
    try:
        takeoff_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/takeoff", CommandTOL)

        ret = takeoff_cl(altitude=1, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as ex:
        print(ex)

    if not ret.success:
        rospy.loginfo("TAKEOFF Request failed. Make sure drone is in GUIDED mode and armable.")
    else:
        rospy.loginfo("TAKEOFF Request success.")

def set_mode(args, mode):
    try:
        setmode_cl = rospy.ServiceProxy(args.mavros_ns + "/set_mode", SetMode)

        ret = setmode_cl(base_mode=0, custom_mode=mode)
    except rospy.ServiceException as ex:
        print(ex)

    if not ret.mode_sent:
        rospy.loginfo("SET MODE {} Request failed.".format(mode))
    else:
        rospy.loginfo("SET MODE {} Request success.".format(mode))


def rc_override_control(args):

    rospy.init_node("mavteleop")
    rospy.loginfo("MAV-Teleop: RC Override control type.")


    override_pub = rospy.Publisher(args.mavros_ns + "/rc/override", OverrideRCIn, queue_size=10)

    throttle_ch = 1000


    while(1):
        roll = 1500
        pitch = 1500
        yaw = 1500
        key = getKey()
        #rospy.loginfo("Key: %s", key)
        if key == 'a':
            arm(args, True)
        elif key == 'd':
            arm(args, False)
        elif key == 't':
            takeoff(args)
        elif key == 'u':
            throttle_ch=1500
            set_mode(args, "LOITER")
        elif key == 'y':
            set_mode(args, "GUIDED")
        elif key == 'n':
            set_mode(args, "LAND")
        elif key == 'r': #UP
            throttle_ch+=10
        elif key == 'f': #FIX
            throttle_ch=1500
        elif key == 'v': #DOWN
            throttle_ch-=10 
        elif key == 'j': #LEFT
            roll=1400   
        elif key == 'l': #RIGHT
            roll=1600   
        elif key == 'i': #FORWARD
            pitch=1400 
        elif key == 'k': #BACKWARD
            pitch=1600
        elif key == 'g': #YAW LEFT
            yaw=1400
        elif key == 'h': #YAW RIGHT
            yaw=1600 	    
        if (key == '\x03'):
                break

        rc = OverrideRCIn()
        rc.channels[0] = roll
        rc.channels[1] = pitch
        rc.channels[2] = throttle_ch
        rc.channels[3] = yaw
        rc.channels[4] = 1000
        rc.channels[5] = 1000
        rc.channels[6] = 1000
        rc.channels[7] = 1000

        #rospy.loginfo("Channels: %d %d %d %d", rc.channels[0], rc.channels[1],rc.channels[2] , rc.channels[3])

        override_pub.publish(rc)


def main():
    parser = argparse.ArgumentParser(description="Teleoperation script for Copter-UAV")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default="/mavros")
    parser.add_argument('-v', '--verbose', action='store_true', help="verbose output")
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('-rc', '--rc-override', action='store_true', help="use rc override control type")
    mode_group.add_argument('-att', '--sp-attitude', action='store_true', help="use attitude setpoint control type")
    mode_group.add_argument('-vel', '--sp-velocity', action='store_true', help="use velocity setpoint control type")
    mode_group.add_argument('-pos', '--sp-position', action='store_true', help="use position setpoint control type")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    print("Press 'a' to Arm, 'd' to Disarm")
    print("Press 't' to Takeoff, 'n' to Land")
    print("Press 'y' for GUIDED, 'u' for LOITER")
    print("Press 'r' for Throttle Up, 'f' for Fix, 'v' for Throttle Down")
    print("Press 'j' to Left, 'l' to Right")
    print("Press 'i' to Forward, 'k' to Backward")
    print("Press 'g' to Yaw Left, 'h' to Yaw Right")

    if args.rc_override:
        rc_override_control(args)


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()

