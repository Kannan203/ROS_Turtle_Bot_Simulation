#! /usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import *

active_ = False #########
position_ = Point()
pub = None
state_ = 0
state_desc = {
0:'find the wall',
1:'Turn left',
2:'follow the wall',
}
regions_ ={
'fright': 0,
'front' : 0,
'fleft' : 0,
}
def wall_follower_switch(msg):
    global active_
    active_ = msg.data
    res = SetBoolResponse()
    res.success = True
    return res
def callback(msg):
    global regions_
    regions_ = {
    'fright': min(min(msg.ranges[310:320]),10),
    'front' : min(min(msg.ranges[0:6]+msg.ranges[354:]),10),
    'fleft': min(min(msg.ranges[50:60]),10),
    }
    take_action()
def change_state(state):
    global state_, state_description
    if state is not state_:
        #print(state)
        state_ = state

def take_action():
    global regions_
    regions = regions_
    d = 3
    state_description = ''

    if regions['fleft'] > d and regions['front'] > d and regions['fright'] > d :
        state_description = 'Case 1-nothing'
        change_state(0)
    if regions['fleft'] > d and regions['front'] > d and regions['fright'] < d :
        state_description = 'Case 2-fright'
        change_state(2)
    if regions['fleft'] > d and regions['front'] < d and regions['fright'] > d :
        state_description = 'Case 3-front'
        change_state(1)
    if regions['fleft'] > d and regions['front'] < d and regions['fright'] < d :
        state_description = 'Case 4-front and fright'
        change_state(1)
    if regions['fleft'] < d and regions['front'] > d and regions['fright'] > d :
        state_description = 'Case 5-fleft'
        change_state(0)
    if regions['fleft'] < d and regions['front'] > d and regions['fright'] < d :
        state_description = 'Case 6-fleft and fright'
        change_state(0)
    if regions['fleft'] < d and regions['front'] < d and regions['fright'] > d :
        state_description = 'Case 7-fleft and front'
        change_state(1)
    if regions['fleft'] < d and regions['front'] < d and regions['fright'] < d :
        state_description = 'Case 8-nothing'
        change_state(1)
    else:
        state_description = 'Unknown_State'
        #rospy.loginfo(regions)
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def Turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_wall():
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub

    rospy.init_node('wall_follower')
    rate = rospy.Rate(20)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    sub = rospy.Subscriber('/scan',LaserScan,callback)
    srv = rospy.Service('wall_follower_switch',SetBool,wall_follower_switch)
    while not rospy.is_shutdown():
        if not active_:
            continue
        msg = Twist()
        if state_ == 0 :
            msg = find_wall()
        elif state_ == 1 :
            msg = Turn_left()
        elif state_ == 2 :
            msg = follow_wall()
        else:
            rospy.loginfo('Unknown_State')
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
