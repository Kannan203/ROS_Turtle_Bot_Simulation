#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from goal_publisher.msg import PointArray
from tf.transformations import euler_from_quaternion
from std_srvs.srv import *
#import ipdb

import math
global srv_client_go_to_point_,srv_client_wall_follower_
position_ = Point()
goal_pos_ = Point()
goal_pos_.x = [0]*20
goal_pos_.y = [0]*20
goal_pos_.z = 0
yaw_ = 0
l = 0
yaw_error_allowed_ = 5 * (math.pi / 180)
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
regions_ = None
state_desc_ = ['Go to point','Wall follower']
state_description = {
0 : 'go_to_point',
1 : 'wall_follower',
}
state_ = 0
count_loop_ = 0
count_state_time_ = 0

def distance_from_line(p0,x,y):
    global position_, goal_pos_
    p1 = position_
    equ_1 = math.fabs((y - p1.y) * p0.x - (x - p1.x) * p0.y + (x * p1.y) - (y * p1.x))
    equ_2 = math.sqrt(pow(y - p1.y, 2) + pow(x - p1.x, 2))
    distance = equ_1/equ_2
    return distance
def change_state(state):
    global srv_client_go_to_point_, srv_client_wall_follower_
    global state_, state_desc_,count_state_time_
    state_ = state
    count_state_time_ = 0
    if state_ == 0:
        res = srv_client_go_to_point_(True)
        res = srv_client_wall_follower_(False)
    if state_ == 1:
        res = srv_client_go_to_point_(False)
        res = srv_client_wall_follower_(True)

def position(msg):
    global position_,yaw_
    position_ = msg.pose[1].position
    rot = msg.pose[1].orientation
    (r,p,yaw_) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
def goal(msg):######
    global goal_pos_
    for l in range(len(goal_pos_.x)):
        goal_pos_.x[l] = msg.goals[l].x
        #print('x_position',goal_pos_.x)
        goal_pos_.y[l] = msg.goals[l].y
        goal_pos_.z = 0
def scanner(msg):
    global regions_
    regions_ = {
    'fright': min(min(msg.ranges[310:320]),10),
    'front' : min(min(msg.ranges[0:6]+msg.ranges[354:]),10),
    'fleft': min(min(msg.ranges[50:60]),10),
    }
def main():
    global regions_,position_,goal_pos_,state_,yaw_,yaw_error_allowed_
    global srv_client_go_to_point_,srv_client_wall_follower_,l
    global count_loop_,count_state_time_
    rospy.init_node('turtlebot')
    rate = rospy.Rate(20)
    pos_pub = rospy.Publisher('/goal_point',Pose,queue_size = 1) ######
    my_goal = Pose()



    sub_position = rospy.Subscriber('/gazebo/model_states',ModelStates,position)
    sub_goal = rospy.Subscriber('/goals',PointArray,goal)
    sub_laser = rospy.Subscriber('/scan',LaserScan,scanner)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch',SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch',SetBool)

    rospy.set_param('gain',"{'x':1,}")
    change_state(0)
    #print('xyz_position',goal_pos_.x[1])
    #for i in range(0,20): ###########################
    #my_goal.position.x = goal_pos_.x[i]###########################
    #print('g12lj',my_goal.position.x)
    #my_goal.position.y = goal_pos_.y[i]############################
    #pos_pub.publish(my_goal)

    while not rospy.is_shutdown():
        #print('goal_position',my_goal.position.x[1])
        if regions_ == None:
            continue
        #print('123_position',goal_pos_.x[1])

            #print(position_from_line)

        if state_ == 0:
            #print('789_position',goal_pos_.x)
            my_goal.position.x = goal_pos_.x[l]
            my_goal.position.y = goal_pos_.y[l]
            #print('456_position',my_goal.position.x)
            position_from_line = distance_from_line(position_,goal_pos_.x[l],goal_pos_.y[l])
            l += 1
            if regions_['front'] > 0.15 and regions_['front'] < 1:
                change_state(1)
        elif state_ == 1:
            if count_state_time_ > 5 and position_from_line < 0.1:
                change_state(0)
                count_loop_ += 1
            if count_loop_ == 20:
                count_state_time_ += 1
                count_loop_ = 0
                            #rospy.loginfo('distance to line: [%.2f]',distance_from_line(position_))
                           #pos_pub.publish(goal)
        pos_pub.publish(my_goal)
        rate.sleep()

    #


if __name__ == "__main__":
    main()
