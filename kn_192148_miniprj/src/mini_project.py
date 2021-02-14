#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from goal_publisher.msg import PointArray
from tf.transformations import euler_from_quaternion
from std_srvs.srv import *
from kn_192148_miniprj.srv import *
#import ipdb

import math
global srv_client_go_to_point_,srv_client_wall_follower_,my_goal,goal_pos_
position_ = Point()
goal_pos_ = Pose()
my_goal = Pose()
yaw_ = 0
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
    #print('rrrrrrrr',my_goal)
    if state_ == 0:
        req = GoToPointRequest()
        req.data = True
        req.destination.position.x = my_goal.position.x
        req.destination.position.y = my_goal.position.y
        #print('ssssssssssssss',req)
        res = srv_client_go_to_point_(req)
        res = srv_client_wall_follower_(False)
    if state_ == 1:
        res = srv_client_go_to_point_(False)
        res = srv_client_wall_follower_(True)

def position(msg):
    global position_,yaw_
    position_ = msg.pose[1].position
    rot = msg.pose[1].orientation
    (r,p,yaw_) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
# def goal(msg):
#     global goal_pos_
#     goal_pos_ = msg
#     print('goalpositionsssss',msg)
#     # for l in range(20):
#     #     goal_pos_.x[l] = msg.goals[l].x
#     #     #print('x_position',goal_pos_.x)
#     #     goal_pos_.y[l] = msg.goals[l].y
#     #     goal_pos_.z = 0
def scanner(msg):
    global regions_
    regions_ = {
    'fright': min(min(msg.ranges[310:320]),10),
    'front' : min(min(msg.ranges[0:6]+msg.ranges[354:]),10),
    'fleft': min(min(msg.ranges[50:60]),10),
    }

def main():
    global regions_,position_,goal_pos_,state_,yaw_,yaw_error_allowed_
    global srv_client_go_to_point_,srv_client_wall_follower_
    global count_loop_,count_state_time_
    rospy.init_node('turtlebot')
    rate = rospy.Rate(20)
    # pos_pub = rospy.Publisher('/goal_point',Pose,queue_size = 1) ######
    goal_pos_ = rospy.wait_for_message('/goals',PointArray)
    # print(goal_pos_.goals)

    for i in range(len(goal_pos_.goals)):
        print('starrti')


        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')

        srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch',GoToPoint)
        srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch',SetBool)

        rospy.set_param('gain',"{'x':1,}")
        my_goal.position.x = goal_pos_.goals[i].x

        my_goal.position.y = goal_pos_.goals[i].y
        print('789_position',my_goal)
        change_state(0)
        #print('xyz_position',goal_pos_.x[1])
        #for i in range(0,20): ###########################
        #my_goal.position.x = goal_pos_.x[i]###########################
        #print('g12lj',my_goal.position.x)
        #my_goal.position.y = goal_pos_.y[i]############################
        #pos_pub.publish(my_goal)


       #print('goal_position',my_goal.position.x[1])
        if regions_ == None:
            continue
        #print('123_position',goal_pos_.x[1])

            #print(position_from_line)

        if state_ == 0:
            # if (my_goal.position.x) == math.ceil(position_.x):
            #     continue
            # else:
            #     rospy.sleep(25)

            #print('456_position',my_goal.position.x)
            position_from_line = distance_from_line(position_,goal_pos_.goals[i].x,goal_pos_.goals[i].y)
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
        print(my_goal.position.x)
        print(round(position_.x,1))
        # pos_pub.publish(my_goal)
        rospy.sleep(20)
        if 1.4 <= (round(position_.x,1)) and my_goal.position.y == position_.y:
            continue
        else:
            rospy.sleep(25)


    #


if __name__ == "__main__":
    rospy.Subscriber('/gazebo/model_states',ModelStates,position)
    #rospy.Subscriber('/goals',PointArray,goal)
    rospy.Subscriber('/scan',LaserScan,scanner)
    main()
