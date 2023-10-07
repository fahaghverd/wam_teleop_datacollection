#!/usr/bin/python3


from operator import truediv
import numpy as np
import rospy 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from wam_msgs.msg import RTJointPos, RTJointVel
from wam_srvs.srv import JointMove
from wam_srvs.srv import Hold
from std_srvs.srv import Empty

import json
#import pickle
import os
import rosservice
#import pygame
#import keyboard
import glob

oint_state_data = []

EE_pose_data = []

p = 0

key_pressed = []

POS_READY = [
    0.002227924477643431, 
    -0.1490540623980915, 
    -0.04214558734519736, 
    1.6803055108189549, 
    0.06452207850075688, 
    -0.06341508205589094, 
    0.01366506663019359,
]

def go_ready_pos():
        """Move WAM to a desired ready position.
        """
        joint_move(POS_READY)

def joint_move(pos_goal: np.ndarray):
        """Move WAM to a desired position.
        q is a numpy array of length 7 that specifies the joint angles
        """
  # Communicate with /wam/joint_move service on control computer
        rospy.wait_for_service('/wam/joint_move')
        try:
            #print('found service')
            joint_move_service = rospy.ServiceProxy('/wam/joint_move', JointMove)
            joint_move_service(pos_goal)
            #print('called move_q')
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def joint_vel_cmd(vel_goal: np.ndarray,
                  jnt_vel_pub):
        msg = RTJointVel()
        # Publish to ROS
        msg.velocities = vel_goal
        #print(rospy.get_time())
        jnt_vel_pub.publish(msg)

def clip_velocity(vel, max_norm):
        vel_norm = np.linalg.norm(vel)
        if vel_norm > max_norm:
            print("clipped vel")
            return vel/vel_norm*max_norm # velocity rescaled to have max norm
        else:
            return vel

def joint_pos_cmd(pos_goal: np.ndarray, jnt_pos_pub):
                      
        msg = RTJointPos()
        # Publish to ROSfrom sensor_msgs.msg import JointState
        msg.joints = pos_goal
        msg.rate_limits = np.array([500.0]*7)
        jnt_pos_pub.publish(msg)


class DataRecorder(object):
    def __init__(self):
        rospy.Subscriber('/wam/joint_states', JointState, self.cb_joint_state)
        rospy.Subscriber('/wam/pose', PoseStamped, self.cb_ee_pose)
        rospy.Subscriber('keyboard_command', String, self.cb_keyboard)
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp',Empty) 
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp',Empty)
        self. joint = rospy.ServiceProxy('/wam/hold_joint_pos',Hold) 
        
        self.started = False
        self.stationary = False
        self.end_traj = False
        self.end_task = False
        self.stop_session = False

        self.joint_state_data = []
        self.ee_pose_data = []


    
    def cb_joint_state(self, data : JointState):
        joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
                'position' : data.position,
                'velocity' : data.velocity,
                'effort' : data.effort}
        if self.collect:
            self.joint_state_data.append(joint_state)
    
    def cb_ee_pose(self, data : PoseStamped):
        EE_pose = {'position' : (data.pose.position.x, data.pose.position.y, data.pose.position.z),
                'orientation': (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,  data.pose.orientation.w)}
        if self.collect:
            self.ee_pose_data.append(EE_pose)

    def cb_keyboard(self, key_pressed : String):
        if key_pressed.data == '0':
             self.close_grasp() #if key '0' close grasp
        
        if key_pressed.data == '1':
            self.open_grasp() #if key '0' close grasp  
             
        if key_pressed.data == 's' and self.picked == False:
            print('Data collecting started')
            self.joint(False) #To be able to move the arm
            self.started = True

        if key_pressed.data == 'p' and self.picked == False:  # if key 'p' is pressed 
                self. joint(False) #To be able to move the arm
                print('Pick Trj Collected')
                out_file1 = open("/home/robot/DMP/DMP_data_joint_pick_{}.json".format(pick_id), "w")
                json.dump(self.joint_state_data, out_file1)
                #print(joint_state_data)
                out_file1.close()
                out_file2 = open("/home/robot/DMP/DMP_data_EE_pick_{}.json".format(pick_id), "w")
                json.dump(self.ee_pose_data, out_file2)
                out_file2.close()
                self.picked = True
                self.joint_state_data = []
                self.ee_pose_data = []
            
        if key_pressed.data == 'q' and self.picked:
                self.close_grasp()
                self. joint(False) #To be able to move the arm
                print('Place Trj Collected')
                #print(joint_state_data)
                out_file1 = open("/home/robot/DMP/DMP_data_joint_place_{}.json".format(place_id), "w")
                json.dump(self.joint_state_data, out_file1)
                out_file1.close()
                out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(place_id), "w")
                json.dump(self.ee_pose_data, out_file2)
                out_file2.close()
                self.joint_state_data = []
                self.ee_pose_data = []
                self.picked = False
                self.collect = False
                
        if key_pressed.data == 'g':
            go_ready_pos()
            self.joint(False) #To be able to move the arm

        
        def record_traj_point(self, mid=None, sub_task = 0, pic_top=[], pic_front=[], pic_angled=[]):
            '''
            Filters out data when the robot is stationary
            Returns dict of form:
            {"position": [-1.0, -1.0], "target": [-1.0, -1.0], 
            "thetas": [-0.017273, 15.160774, -179.994904, -130.040115, 0.035609, 54.996262, 89.982719], 
            "velocity": []}

            '''
            
            q_dot = self.vel[:7]

            # skip adding this data to the traj if q_dot is too small
            if np.linalg.norm(q_dot, ord=np.inf) < self.epsilon:
                if self.started and not self.stationary:  # if the robot just stopped
                    print('stationary')         
                    self.stationary = True
                return None

            # everything after this point executes if the robot is not stationary
            self.stationary = False
            
            if not self.started:
                print('Motion detected!')  # this prints the first time movement is detected at the start of a new traj
                self.started = True


            q = list(self.position[:7])
            gripper = [self.position[7]]
            q_dot = list(q_dot)
            cart_pose = list(self.cartesian_pose)

            if mid is not None:
                #print("mid is not none")
                traj_point = [{'Top':pic_top, 'Front': pic_front, 'Angled': pic_angled}, {'position':cart_pose, 'target':mid, 'thetas':q, 'gripper':gripper, 'velocity':q_dot}]
            else:
                traj_point = {'position':cart_pose, 'target':None, 'thetas':q, 'gripper':gripper, 'velocity':q_dot, 'sub_task': sub_task}


            return traj_point 
            
