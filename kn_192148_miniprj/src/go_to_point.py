#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point,Pose
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from goal_publisher.msg import PointArray
from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
from kn_192148_miniprj.srv import GoToPoint, GoToPointResponse
import math

active_ = False

position_ = Point()
r_yaw_ = 0

r_state_ = 0
global goal_pos_
goal_pos_ = Pose()
##print(goals)
# goal_pos_.position.x = 1
# goal_pos_.position.y = 0
# goal_pos_.z = 0


yaw_precision_ = math.pi/90
dist_precision_ = 0.1

pub = None
# def goal(msg):
#     global goal_pos_
#     goal_pos_.position.x = msg.position.x
#     goal_pos_.position.y = msg.position.y
#     goal_pos_.z = 0 #####
#     #print('goal_position123_x',goal_pos_.position.x)
def go_to_point_switch(msg):
    global active_, goal_pos_
    active_ = msg.data
    goal_pos_ = msg.destination
    res = GoToPointResponse()
    res.success = True
    print('kkkkkkkk')
    return res
def callback(msg):
    global position_
    global r_yaw_
    position_ = msg.pose[1].position
    rot = msg.pose[1].orientation
    (r,p,r_yaw_) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
def change_state(state):
    global r_state_
    r_state_ = state
    print ('State Changed to [%s]'% r_state_)
def fix_yaw(x,y):
    global r_state_,r_yaw_,yaw_precision_,pub
    desired_yaw_ = math.atan2(y - position_.y,x - position_.x )
    err_yaw = desired_yaw_ - r_yaw_
    twist_msg = Twist()
    if math.fabs(err_yaw)>yaw_precision_:
        twist_msg.angular.z = 0.5 if err_yaw > 0 else -0.7
    pub.publish(twist_msg)
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)
def go_straight(x,y):
    global r_yaw_,yaw_precision_,r_state_,goal_pos_
    desired_yaw_ = math.atan2(y - position_.y,x - position_.x )
    err_yaw = desired_yaw_ - r_yaw_
    err_pos = math.sqrt(pow(y - position_.y,2)+ pow(x - position_.x,2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        pub.publish(twist_msg)
    else:
        change_state(2)
    if math.fabs(err_yaw)>yaw_precision_:
        change_state(0)
def done():
    global r_state_
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    r_state_ = 0

def main():
    global pub,active_, r_state_
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
    sub = rospy.Subscriber('/gazebo/model_states',ModelStates,callback)
    # sub_goal = rospy.Subscriber('/goal_point',Pose,goal)####
    srv = rospy.Service('go_to_point_switch',GoToPoint,go_to_point_switch)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if r_state_ == 0:
                fix_yaw(goal_pos_.position.x,goal_pos_.position.y)
            elif r_state_ == 1:
                go_straight(goal_pos_.position.x,goal_pos_.position.y)
            elif r_state_ == 2:
               done()


            else:
                rospy.loginfo('Unknown_State')


            rate.sleep()
if __name__ == '__main__':
    main()
