#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# multi_goals3.py
# https://www.programcreek.com/python/?code=tianbot%2Ftianbot_racecar%2Ftianbot_racecar-master%2Fracecar_navigation%2Fscript%2Fmulti_goals.py
# https://www.programcreek.com/python/example/70252/geometry_msgs.msg.PoseStamped
#
# https://edu.gaitech.hk/turtlebot/map-navigation.html
#
# 速度 平滑化
# https://www.codetd.com/ja/article/10672026
#
# ROS Pythonでオイラー角とクォータニオンの相互変換
# https://yura2.hateblo.jp/entry/2016/05/18/ROS_Python%E3%81%A7%E3%82%AA%E3%82%A4%E3%83%A9%E3%83%BC%E8%A7%92%E3%81%A8%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%E3%81%AE%E7%9B%B8%E4%BA%92%E5%A4%89%E6%8F%9B
#
# プログラムからナビゲーションを実行する方法 
# https://robot.isc.chubu.ac.jp/?p=1261
#

import rospy
import string
import math
import time
import sys

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import math

def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])



class MultiGoals:
    def __init__(self, goalList, retry, map_frame):
        self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        # params & variables
        self.goalList = goalList

        self.retry = retry
        self.goalId = 0
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0
        # Publish the first goal
        time.sleep(1)
        self.mloop()

    def statusCB(self, data):
        if data.status.status == 3: # reached
            self.sts=2
        else:
            self.sts=3
            rospy.loginfo("statusCB error ! data.status.status=%d", data.status.status) 


    def mloop(self):
        self.sts=0
        self.prev_id=0
        self.r_ct=0

        self.pub_sub(self.goalId)
        self.goalId = self.goalId + 1 
        self.sts=1

        while(1):
            if self.sts==2:
                self.sts=0
                go=1
                self.r_ct =0

                if self.goalId <= (len(self.goalList)-1):
                    id = self.goalId
                    self.goalId = self.goalId + 1
                else:
                    if self.retry == 1:
                        self.goalId = 0 
                        id = self.goalId
                    else:
                        go=0
                        self.sts=9

                if go==1:
                    self.prev_id=id
                    self.pub_sub(id)
                    self.sts=1

            elif self.sts==3:
                self.r_ct = self.r_ct + 1
                if self.r_ct < 2:
                    self.pub_sub(self.prev_id)
                    self.sts=1
                else:
                    self.sts=9

            time.sleep(1)
            if self.sts==9:
                break
        sys.exit()


    def pub_sub(self,id):
        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalList[id][0]
        self.goalMsg.pose.position.y = self.goalList[id][1]

        #euler_to_quarternion(Vector3(0.0, 0.0, math.pi / 2.0))
        q=euler_to_quaternion(Vector3(0.0, 0.0, self.goalList[id][2]))
        self.goalMsg.pose.orientation.x = q.x
        self.goalMsg.pose.orientation.y = q.y
        self.goalMsg.pose.orientation.z = q.z
        self.goalMsg.pose.orientation.w = q.w


        self.pub.publish(self.goalMsg) 
        rospy.loginfo("Initial goal published! Goal ID is: %d", id) 


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('multi_goals', anonymous=True)


        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = int(rospy.get_param('~retry', '0'))


        goalList =[[0.0,0.0, 0.0],
                    [0.0,0.0, math.pi / 2.0],
                    [0.0,0.0, math.pi ],
                    [0.0,0.0, math.pi * 1.5 ],
                    [0.0,0.0, 0.0],

                    [1.0,0.0, 0.0],
                    [1.0,0.0, math.pi / 2.0],
                    [1.0,0.0, math.pi ],
                    [1.0,0.0, math.pi * 1.5 ],
                    [1.0,0.0, 0.0],

                    [2.0,0.0, 0.0],
                    [2.0,0.0, math.pi / 2.0],
                    [2.0,0.0, math.pi ],

                    [1.0,0.0, math.pi ],
                    [1.0,0.0, math.pi * 1.5 ],
                    [1.0,0.0, 0.0],
                    [1.0,0.0, math.pi / 2.0],
                    [1.0,0.0, math.pi ],

                    [0.0,0.0, math.pi ],
                    [0.0,0.0, 0.0]]
        print goalList
        #sys.exit()

        if len(goalList) > 0:          
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalList, retry, map_frame)          
            rospy.spin()
        else:
            rospy.errinfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")
    