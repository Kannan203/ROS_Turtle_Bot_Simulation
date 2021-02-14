#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Point,Pose
from sensor_msgs.msg import LaserScan, Range
from std_srvs.srv import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import sqrt,pow
import numpy as np 
from numpy import inf

import math


class Point_to_Point_detection(object):

    def __init__(self):
        
        self.active_ = True 
        self.state_ = 0
        self.position_ = Point()
        self.yaw_precision_ = math.pi/90
        self.dist_precision_ = 0.45

        self.scan_value = []
        self.goal_pos_x = 1.98#-1.01#1.82#1.98
        self.goal_pos_y = 3.98#3.34#2.35#3.98
        self.curr_pos = 0

        self.yaw_ = 0
        self.delta = 0.3

        self.dmin = []
        self.rad_d = 0
        self.the_d = 0
        self.d = 0
        self.inf_rep = 7

        self.prev_x = 0
        self.prev_y = 0
        self.curr_x = 0
        self.curr_y = 0
        
        self.angle_min= -1.57079637051
        self.angle_max= 1.53938043118
        self.angle_incr = 0.0314159281552
        self.initial_x_1 = [0]*2
        self.initial_y_1 = [0]*2
        self.initial_x = 0

        #self.angle = 0
        #self.init_yaw.append(self.angle)
        

        rate = rospy.Rate(20)
        time = rospy.Rate(20)
        self.regions_ ={
        'fright': 0,
        'front' : 0,
        'fleft' : 0,
        }

        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.laser = rospy.Subscriber('/scan',LaserScan,self.scanner)
        #self.srv = rospy.Service('wall_follower_switch',SetBool,self.wall_follower_switch)
        self.pos = rospy.Subscriber('/gazebo/model_states',ModelStates,self.call_pos)

        while not rospy.is_shutdown():
            if not self.active_:
                continue
            else:
                if self.state_ == 0:
                    #self.pos_x = self.position_.x
                    #temp = self.pos_x
                    self.initial_x_1[0] = self.position_.x
                    if len(self.initial_x_1) == 1 :
                        self.initial_x = self.initial_x_1
                        print('initial x',self.initial_x)

                   # self.pos_y = self.position_.y
                    #temp = self.pos_y
                    self.initial_y = 0

            
                    self.move_head_goal(self.goal_pos_x,self.goal_pos_y)
                   # print('move_head_goal')

                if self.state_ == 1:
                    self.go_to_goal(self.goal_pos_x,self.goal_pos_y)
                   # print('go_to_goals')

                if self.state_ == 2:
                    self.Find_sudden_point()
                    print('obstacle found')
                
                if self.state_ == 3:
                    self.rotate_head((self.x_pos1),(self.y_pos1))
                    #print('rotate head')
                
                if self.state_ == 4:
                    self.go_straight((self.x_pos1),(self.y_pos1))
                    #print('move straight')

                if self.state_ == 5:
                    velocity = Twist()
                    velocity.angular.z = 0.0
                    velocity.linear.x = 0
                    self.pub.publish(velocity)
                    print('Reached')
            rate.sleep()
    
    def move_head_goal(self,x,y):

        self.m1 = self.slope(self.initial_x,self.initial_y,self.goal_pos_x,self.goal_pos_y)

        goal_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = self.normalised_yaw(goal_yaw_ - self.yaw_)
        velocity = Twist()
        if math.fabs(err_yaw)>self.yaw_precision_:
            velocity.angular.z = 0.3 if err_yaw > 0 else -0.4
        self.pub.publish(velocity)
        if math.fabs(err_yaw) <= self.yaw_precision_:
            self.change_state(1)

    def slope(self,ix,iy,gx,gy):
        m = (ix - gx)/(iy - gy)
        return m
    
    def go_to_goal(self,x,y):
        
        self.obstacle_ref = 0.3

        desired_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = desired_yaw_ - self.yaw_
        err_dist =math.sqrt(pow(y - self.position_.y,2)+ pow(x - self.position_.x,2))
        #print('err_dist',err_dist)


        if err_dist > self.dist_precision_:
            velocity = Twist()
            velocity.linear.x = 0.3 #### 0.7 ###
            self.pub.publish(velocity)
        else:
            self.change_state(5)
        if math.fabs(err_yaw) > self.yaw_precision_:
            self.change_state(0)
        if self.regions_['front'] < self.obstacle_ref:
            self.change_state(2)
    def coordinate_conv(self,th,xp,yp,xr,yr):
        Rot_mat = np.array([[math.cos(th),-math.sin(th),xr],
        [math.sin(th),math.cos(th),yr],[0,0,1] ] )

        trans_point = np.array([[xp],[yp],[1]])

        conv = np.dot(Rot_mat,trans_point)
        return conv

    
    def Find_sudden_point(self):
        rate = rospy.Rate(20)
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.pub.publish(velocity)
        self.m2 = self.slope(self.position_.x,self.position_.y,self.goal_pos_x,self.goal_pos_y)
        #print('m1',self.m1)
        #print('m2',self.m2)

        if self.m2 == 0 or self.m1 > self.m2 :
            print('scanning from 0')
            for i in range (len(self.scan_value[0])-1):
                #self.scan_value[self.scan_value[i] == inf] = 7  #####################
                c = self.scan_value[0][i] - self.scan_value[0][i-1]
                print('c',c )

                if abs(c) > 0.25:
                    self.rad_d = round((self.scan_value[0][i]),2) #i+1
                    self.the_d = (i)#-25) #i+1
                    print('idistance',self.rad_d)
                    #print('iangle',self.the_d)
                    break

        if self.m1 < self.m2 : 
            print('scanning from 359')       
            for i in range ((len(self.scan_value[0])-1),-1,-1):
                #self.scan_value[self.scan_value[i] == inf] = 7 ########################
                c = self.scan_value[0][i] - self.scan_value[0][i-1]
                print('c',c )
                
                if abs(c) > 0.25:
                    self.rad_d = round((self.scan_value[0][i]),2) #i+1
                    if i >= 347:
                        self.the_d = abs(347 - i)
                    else:
                        self.the_d = (i+6) #i+1
                    print('idistance',self.rad_d)
                    print('iangle',self.the_d)
                    break

        self.x_point =  self.Get_distance_((self.rad_d),self.the_d)[0] 
        #print('disX',self.position_.x)

        self.y_point =   self.Get_distance_((self.rad_d),self.the_d)[1] 
        #print('disY',self.position_.y)
        self.x_pos = self.coordinate_conv(self.yaw_,self.x_point,self.y_point,self.position_.x,self.position_.y)[0][0]
        self.y_pos = self.coordinate_conv(self.yaw_,self.x_point,self.y_point,self.position_.x,self.position_.y)[1][0]
        
################# Along diagonal movement ###############

        if self.x_pos > 0 and self.y_pos > 0 :
            self.x_pos1 = self.x_pos + self.delta
            self.y_pos1 = self.y_pos + self.delta
            self.change_state(3)
        elif self.x_pos > 0 and self.y_pos < 0 :
            self.x_pos1 = self.x_pos + self.delta
            self.y_pos1 = self.y_pos - self.delta
            self.change_state(3)
        elif self.x_pos < 0 and self.y_pos < 0 :
            self.x_pos1 = self.x_pos - self.delta
            self.y_pos1 = self.y_pos - self.delta
            self.change_state(3)
        elif self.x_pos < 0 and self.y_pos > 0 :
            self.x_pos1 = self.x_pos - self.delta
            self.y_pos1 = self.y_pos - self.delta
            self.change_state(3)

############## Along axis movement######################
        elif self.x_pos > 0 and self.y_pos == 0 :
            self.x_pos1 = self.x_pos + self.delta
            self.change_state(3)
        elif self.x_pos < 0 and self.y_pos == 0 :
            self.x_pos1 = self.x_pos - self.delta
            self.change_state(3)
        elif self.x_pos == 0 and self.y_pos > 0 :
            self.y_pos1 = self.y_pos + self.delta
            self.change_state(3)
        elif self.x_pos == 0 and self.y_pos < 0 :
            self.y_pos1 = self.y_pos - self.delta
            self.change_state(3)
       # print('yaw',self.yaw_)
        #print('radians',self.scan_value[0])
        print('X original',self.x_pos)
        print('Y original',self.y_pos)
        #print('Y duplicate',self.y_pos1)


    def normalised_yaw(self,angle):

        if (math.fabs(angle)>math.pi):
            angle = angle - (2*math.pi*angle)/(math.fabs(angle))
        return angle

    def rotate_head(self,x,y):
        #print('x_pos',x)
        desired_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = self.normalised_yaw(desired_yaw_ - self.yaw_)
        velocity = Twist()
        if math.fabs(err_yaw)>self.yaw_precision_:
            velocity.angular.z = 0.3 if err_yaw > 0 else -0.4
        self.pub.publish(velocity)
        if math.fabs(err_yaw) <= self.yaw_precision_:
            self.change_state(4)

    def go_straight(self,x,y):
        

        desired_yaw_ = math.atan2(y - self.position_.y,x - self.position_.x )
        err_yaw = desired_yaw_ - self.yaw_
        err_dist =math.sqrt(pow(y - self.position_.y,2)+ pow(x - self.position_.x,2))
        print('err_dist',err_dist)


        if err_dist > self.dist_precision_:
            velocity = Twist()
            velocity.linear.x = 0.3 #### 0.7 ###
            self.pub.publish(velocity)
        else:
            self.change_state(0)

        if math.fabs(err_yaw) > self.yaw_precision_:
            self.change_state(3)
        #elif self.regions_['front'] == self.inf_rep:
        if self.regions_['front'] < self.obstacle_ref:
            self.change_state(2)
            #self.change_state(0)
        

    def call_pos(self,msg):
        self.position_ = msg.pose[1].position

        rot = msg.pose[1].orientation
        (r,p,self.yaw_) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])


    def scanner(self,msg):
        self.scan_value = msg.ranges
        self.scan_value = np.array([self.scan_value])
        #print('oiiu',len(self.scan_value[0]))
        #self.scan_value = np.array([self.scan_value])
        #self.scan_value[self.scan_value == inf] = self.inf_rep 

        self.regions_ = {
        'front' : msg.ranges[0] #min(min(msg.ranges[0:1]+msg.ranges[358:]),4) ####6 354 #### ####2nd pref 0-15 350: ####
        }
        

    def change_state(self,state):
        self.state_ = state
          
    def Eucli_dist(self,cx,cy,px,py):
        d = pow((cx-px),2) + pow((cy-py),2)
        return sqrt(d)

    def Get_distance_(self,r,th):
        x_point = round(r * math.cos(math.radians(th)),2)
        print('x_point',x_point)
        y_point = round(r * math.sin(math.radians(th)),2)
        return [x_point,y_point]
    #def Get_arc_distance(self,x,y):
        #dx = self.goal_pos_x
    

if __name__ == '__main__':
    rospy.init_node('Avoid_Wall')
    Point_to_Point_detection()
