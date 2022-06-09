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
# multi_goals4_cmd_vel.py
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
# 起動点順は、
# rtabmap_ros_my/launch/demo_turtlebot3_navigation_nishi2.launch 参照
# $ rosrun ros_start multi_goals4.py
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

#--
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid,Odometry

from std_srvs.srv import Empty

import cmd_vel2 as cmd_v2


class GetMap:
    def __init__(self):
        self.map_info = None
        self.map_data = None
        self.grid = None
        self.resolution = None
        self.line_w = 8  # ラインの幅 -> grid size [dot]  0.05[m] * 8 = 40[cm]
        self.car_r = 2    # ロボットの回転半径 [dot]
        self.match_rviz = True      # True / Flase  -> Rviz の画像と同じにする / しない

 
    def get(self):
        map_msg=None
        cnt=30
        while map_msg is None and cnt >=0:
            try:
                map_msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
            except:
                pass
            cnt-=1
        if map_msg != None:
            self.map_info = map_msg.info
            print 'map_info='
            print self.map_info
            self.free_thresh = int(0.196 * 255)
            self.resolution = self.map_info.resolution
            print 'self.resolution=',self.resolution

            self.org_x = self.map_info.origin.position.x
            self.org_y = self.map_info.origin.position.y
            print 'self.org_x=',self.org_x
            print 'self.org_y=',self.org_y

            x_size = 421 * self.resolution
            y_size = 421 * self.resolution

            print 'x_size,y_size=',x_size,y_size

            #print 'map_msg.data[0:10]=',map_msg.data[1:10]
            print 'self.free_thresh=',self.free_thresh
            self.conv_fmt2(map_msg)
            print 'ok'
        else:
            print 'error'


    def conv_fmt2(self,map_msg):
        m_grid = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        print 'm_grid.shape=',m_grid.shape

        #print m_grid
        np.place(m_grid,m_grid == -1, 255)    # replace all -1 to 255 
        m_grid = m_grid.astype(np.uint8)

        # ここで、Rviz の画像に合わせてみる。
        if self.match_rviz == True:
            #m_grid = np.rot90(m_grid)   # 90度回転
            #m_grid = np.fliplr(m_grid)
            a_trans = m_grid.transpose()
            m_grid = a_trans
            m_grid = np.rot90(m_grid)   # 90度回転
            m_grid = np.rot90(m_grid)   # 90度回転

        if False:
            cv2.imwrite('xxx3.jpg', m_grid)
            cv2.namedWindow("image",cv2.WINDOW_NORMAL)
            cv2.imshow("image", m_grid)

            cv2.waitKey(0)
            cv2.destroyAllWindows()

        self.map_data = m_grid

        height = int(map_msg.info.height / self.line_w)
        width = int(map_msg.info.width / self.line_w)

        print 'map_msg.info.height,map_msg.info.width=',map_msg.info.height,map_msg.info.width
        print 'height,width=',height,width

        #self.grid = cv2.resize(m_grid,(width,height)) # resize map_msg.data
        self.grid = self.np_resize(m_grid,(width,height))

        #np.place(self.grid,self.grid <= self.free_thresh ,0)
        #np.place(self.grid,self.grid > self.free_thresh ,255)

        print 'self.grid.shape=',self.grid.shape

        if False:
            cv2.imwrite('xxx4.jpg', self.grid)
            cv2.namedWindow("image",cv2.WINDOW_NORMAL)
            cv2.imshow("image", self.grid)

            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def conv_dot2grid(self,x,y):
        xi= int(x / self.resolution)
        yi= int(y/ self.resolution)
        gx = int(xi / self.line_w)
        gy = int(yi / self.line_w)
        return gx,gy

    '''
    convert Grid to Meter World
    '''
    def conv_grid2meter(self,gx,gy,f_or_l):
        if self.match_rviz == True:
            return self.conv_grid2meter_m_p(gx,gy,f_or_l)
        else:
            return self.conv_grid2meter_m_m(gx,gy,f_or_l)

    '''
    Rviz と同じ図形 にした場合。
    左上: M(+x,+y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(-x,-y)
    '''
    def conv_grid2meter_m_p(self,gx,gy,f_or_l):
        #x = (210 - (gx * self.line_w + self.line_w/2)) * self.resolution
        x =  -(gx * self.line_w + self.line_w/2) * self.resolution - self.org_x # chnaged by nishi 2020.12.2
        #y = (210 - (gy * self.line_w + self.line_w/2)) * self.resolution
        y =  -(gy * self.line_w + self.line_w/2) * self.resolution - self.org_y  # chnaged by nishi 2020.12.2
        return x,y

    '''
    Map データの図形 : y =  x の線対称の図形(x,y の入れ替え)
    左上: M(-x,-y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(+x,+y)
    '''
    def conv_grid2meter_m_m(self,gx,gy,f_or_l):
        #x = (-210 + (gx * self.line_w + self.line_w/2)) * self.resolution
        x = (gx * self.line_w + self.line_w/2) * self.resolution + self.org_x   # chnaged by nishi 2020.12.2
        #y = (-210 + (gy * self.line_w + self.line_w/2)) * self.resolution
        y = (gy * self.line_w + self.line_w/2) * self.resolution + self.org_y   # chnaged by nishi 2020.12.2
        return y,x

    def get_rotation_matrix(self,rad):
        """
        指定したradの回転行列を返す
        """
        rot = np.array([[np.cos(rad), -np.sin(rad)],
                        [np.sin(rad), np.cos(rad)]])
        return rot

    def np_resize(self,m_grid,req_size):
        (w,h) = req_size
        (w0,h0) = m_grid.shape
        size_x = int(w0 / w)
        size_y = int(h0 / h)
        o_dt = np.zeros((w, h),dtype='uint8')

        for i in range(w):
            for j in range(h):
                #av = np.mean(m_grid[i*size_x:(i+1)*size_x,j*size_y:(j+1)*size_y])
                av = np.max(m_grid[i*size_x:(i+1)*size_x,j*size_y:(j+1)*size_y])    # changed by nishi 2020.12.2
                #if av < 255:
                #    print 'av=',av
                if math.isnan(av) == False:
                    o_dt[i,j]= int(av)
                    if o_dt[i,j] < 255:
                        print 'o_dt[',i,',',j,']=',o_dt[i,j]
                else:
                    print 'GetMap.np_resize() : resize error'
        return o_dt

def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])



class MultiGoals:
    def __init__(self, map_frame,get_map,use_sim_time):
        self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        # params & variables
        #self.goalList = goalList

        self.goalId = 0
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0


        self.goalMsg.pose.position.x = 0.0
        self.goalMsg.pose.position.y = 0.0
        self.goalMsg.pose.position.z = 0.0

        self.get_map=get_map
        self.use_sim_time=use_sim_time

        self.cmd_ctl=cmd_v2.cmd_control()

        # Publish the first goal
        time.sleep(1)
        #self.mloop()

    '''
    mloop_ex(self,goalList)
        goalList: ゴールリスト
    '''
    def mloop_ex(self,goalList):
        # params & variables
        self.goalList = goalList
        self.goalId = 0
        self.mloop()

    '''
    move(self,dist,deg)
        dist: 移動距離
        deg: 方向 [度]
    '''
    def move(self,dist,deg):
        self.goalId = 0
        y = dist * math.sin(math.radians(deg)) + self.goalMsg.pose.position.y
        x = dist * math.cos(math.radians(deg)) + self.goalMsg.pose.position.x
        oz = math.radians(deg)
        self.goalList = [[0, x,y, oz]]
        self.mloop()

    '''
    m_move(self,m_list)
      m_list
        [[func,dist,deg],...]
        func: 0 -> move dist,deg
              1 -> sleep
              2 -> get map
              50 -> set Navigation mode
              99 -> end
    '''
    def m_move(self,m_list):
        cur=0
        while(1):
            if cur >= len(m_list):
                break
            if m_list[cur][0] == 9:
                break
            if m_list[cur][0] != 0:
                self.goalId = 0
                self.goalList = [[m_list[cur][0], 0, 0]]
                self.mloop()
            else:
                dist = m_list[cur][1]
                deg = m_list[cur][2]
                self.move(dist,deg)
            cur += 1

    '''
    statusCB(self, data)    
    '''
    def statusCB(self, data):
        if data.status.status == 3: # reached
            now_xy=self.get_odom()
            self.sts=2
        else:
            self.sts=3
            rospy.loginfo("statusCB error ! data.status.status=%d", data.status.status) 

    '''
    mloop(self)
      self.goalList =[[func,x,y,d_yaw],....] or [[func,dist,d_yaw],....]
        func,x,y,d_yaw
        func: 0 -> move point x,y, and rotate d_yaw
              1 -> move point x,y only
              2 -> rotate d_yaw only
              21 -> sleep
              22 -> get map
              50 -> set Navigation mode
              99 -> end
        func,dist,d_yaw
        func: 0 -> move dist and rotate d_yaw
              21 -> sleep
              22 -> get map
              50 -> set Navigation mode
              60 -> course_correct ON
              61 -> course_correct OFF
              62 -> after_correct_wait ON
              63 -> after_correct_wait OFF
              99 -> end
    '''
    def mloop(self):
        while(1):
            if self.goalId <= (len(self.goalList)-1):
                if self.goalList[self.goalId][0] == 0 or self.goalList[self.goalId][0] == 1 or self.goalList[self.goalId][0] == 2:
                    self.mloop_sub()
                elif self.goalList[self.goalId][0] == 21:
                    time.sleep(1)
                elif self.goalList[self.goalId][0] == 22:
                    self.get_map.get()
                elif self.goalList[self.goalId][0] == 50:
                    #rospy.set_param('/rtabmap/rtabmap/Mem/IncrementalMemory',False)
                    self.call_service()
                    print 'set Navigation Mode'

                elif self.goalList[self.goalId][0] == 60:   # course correct ON
                    self.cmd_ctl.course_correct = True
                    print 'course correct ON'

                elif self.goalList[self.goalId][0] == 61:   # course correct OFF
                    self.cmd_ctl.course_correct = False
                    print 'course correct OFF'

                elif self.goalList[self.goalId][0] == 62:   # after_correct_wait ON
                    self.cmd_ctl.after_correct_wait = True
                    print 'after_correct_wait ON'

                elif self.goalList[self.goalId][0] == 63:   # after_correct_wait OFF
                    self.cmd_ctl.after_correct_wait = False
                    print 'after_correct_wait OFF'

                elif self.goalList[self.goalId][0] == 99:
                    break
            else:
                break
            #time.sleep(1)

            self.goalId = self.goalId + 1 


    def mloop_sub(self):
        self.sts=0
        self.r_ct =0
        t_type=0
        # 0 : Idle
        # 1 : Waiting for reached or error event
        # 2 : reached
        # 3 : error
        # 9 : end

        if len(self.goalList[self.goalId]) != 4:
            t_type=1

        while(1):
            if self.sts==0:
                if t_type == 0:
                    x = self.goalList[self.goalId][1]
                    y = self.goalList[self.goalId][2]
                    d_yaw = self.goalList[self.goalId][3]
                    if self.goalList[self.goalId][0] == 0:
                        self.cmd_ctl.move_abs(x,y,d_yaw)
                        self.goalMsg.pose.position.y = y
                        self.goalMsg.pose.position.x = x
                    elif self.goalList[self.goalId][0] == 1:
                        self.cmd_ctl.go_abs(x,y)
                        self.goalMsg.pose.position.y = y
                        self.goalMsg.pose.position.x = x
                    elif self.goalList[self.goalId][0] == 2:
                        self.cmd_ctl.rotate_abs(d_yaw)

                else:
                    dist = self.goalList[self.goalId][1]
                    d_yaw = self.goalList[self.goalId][2]
                    self.cmd_ctl.move(dist,d_yaw)
                    self.goalMsg.pose.position.y += dist * math.sin(math.radians(d_yaw))
                    self.goalMsg.pose.position.x += dist * math.cos(math.radians(d_yaw))

                r_yaw = math.radians(d_yaw)
                q0 = tf.transformations.quaternion_from_euler(0.0, 0.0, r_yaw)
                self.goalMsg.pose.orientation.x = q0[0]
                self.goalMsg.pose.orientation.y = q0[1]
                self.goalMsg.pose.orientation.z = q0[2]
                self.goalMsg.pose.orientation.w = q0[3]


                print "goal published! Goal ID is:", self.goalId
                #self.pub_sub(x,y,yaw)
                self.sts=2

            elif self.sts==2:
                return
            elif self.sts==3:
                self.r_ct = self.r_ct + 1
                diff_xyz=self.get_odom()
                if abs(diff_xyz[0]) <= 0.09 and abs(diff_xyz[1]) <= 0.09:
                    print 'differ x-y is ok'

                if self.r_ct < 3:
                    if t_type == 0:
                        x = self.goalList[self.goalId][1]
                        y = self.goalList[self.goalId][2]
                        yaw = self.goalList[self.goalId][3]
                    else:
                        dist = self.goalList[self.goalId][1]
                        deg = self.goalList[self.goalId][2]
                        y = dist * math.sin(math.radians(deg)) + self.goalMsg.pose.position.y
                        x = dist * math.cos(math.radians(deg)) + self.goalMsg.pose.position.x
                        yaw = math.radians(deg)

                    print "goal published! Goal ID is:", self.goalId
                    self.pub_sub(x,y,yaw)
                    self.sts=1
                else:
                    print 'error retry over'
                    return
            time.sleep(1)   


    def pub_sub(self,x,y,yaw):
        self.goalMsg.header.stamp = rospy.Time.now()
        #self.goalMsg.pose.position.x = self.goalList[id][1]
        self.goalMsg.pose.position.x = x
        #self.goalMsg.pose.position.y = self.goalList[id][2]
        self.goalMsg.pose.position.y = y

        #euler_to_quarternion(Vector3(0.0, 0.0, math.pi / 2.0))
        #q=euler_to_quaternion(Vector3(0.0, 0.0, self.goalList[id][3]))
        #q0 = tf.transformations.quaternion_from_euler(0.0, 0.0, self.goalList[id][3])
        q0 = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self.goalMsg.pose.orientation.x = q0[0]
        self.goalMsg.pose.orientation.y = q0[1]
        self.goalMsg.pose.orientation.z = q0[2]
        self.goalMsg.pose.orientation.w = q0[3]

        x =round(self.goalMsg.pose.position.x,4)
        y =round(self.goalMsg.pose.position.y,4)
        z =round(self.goalMsg.pose.position.z,4)

        ox=0.0
        oy=0.0
        oz=yaw

        #qx =round(q0[0],4)
        #qy =round(q0[1],4)
        #qz =round(q0[2],4)
        #qw =round(q0[3],4)

        self.pub.publish(self.goalMsg)
        #rospy.loginfo("Initial goal published! Goal ID is: %d", id) 
        print 'go x,y,z:ox,oy,oz =',x, y, z, ':', ox, oy, round(oz,4)

    def get_odom(self):
        odom_msg=None
        cnt=30
        x=None
        y=None
        z=None
        ox=None
        oy=None
        oz=None

        while odom_msg is None and cnt >=0:
            try:
                if self.use_sim_time == True:
                    odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                else:
                    odom_msg = rospy.wait_for_message('/odom_fox', Odometry, timeout=5)
            except:
                pass
            cnt-=1
        if odom_msg != None:
            # x[M] ,y[M]
            x =self.goalMsg.pose.position.x - odom_msg.pose.pose.position.x
            y =self.goalMsg.pose.position.y - odom_msg.pose.pose.position.y
            z =self.goalMsg.pose.position.z - odom_msg.pose.pose.position.z

            #ox =round(self.goalMsg.pose.orientation.x - odom_msg.pose.pose.orientation.x,4)
            #oy =round(self.goalMsg.pose.orientation.y - odom_msg.pose.pose.orientation.y,4)
            #oz =round(self.goalMsg.pose.orientation.z - odom_msg.pose.pose.orientation.z,4)
            #ow =round(self.goalMsg.pose.orientation.w - odom_msg.pose.pose.orientation.w,4)

            pe = tf.transformations.euler_from_quaternion((self.goalMsg.pose.orientation.x, self.goalMsg.pose.orientation.y, self.goalMsg.pose.orientation.z, self.goalMsg.pose.orientation.w))
            #print 'pe[0],pe[1],pe[2]=',pe[0],pe[1],pe[2]
            oe = tf.transformations.euler_from_quaternion((odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w))
            ox = pe[0]-oe[0]
            oy = pe[1]-oe[1]
            oz = pe[2]-oe[2]

            print 'differ x,y,z:ox,oy,oz =',round(x,4),round(y,4),round(z,4),':',round(ox,4),round(oy,4),round(oz,4)
            print 'ok'
        else:
            print 'error'
        return x,y,z,ox,oy,oz

    def call_service(self):
        rospy.wait_for_service('/rtabmap/set_mode_localization')
        try:
            service = rospy.ServiceProxy('/rtabmap/set_mode_localization', Empty)
            response = service()
        except rospy.ServiceException, e:
            print "Service call faild: %s" % e


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('multi_goals', anonymous=True)

        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = int(rospy.get_param('~retry', '0'))

        use_sim_time=rospy.get_param('/use_sim_time')

        # func,x,y,d_yaw
        # func: 0 -> move point x,y and rotate d_yaw
        #       1 -> move point x,y only
        #       2 -> rotate d_yaw only
        #       21 -> sleep
        #       22 -> get map
        #       50 -> set Navigation mode
        #       99 -> end
        goalList =[[60,0.0, 0.0],      # course correct ON
                   [0,0.0,0.0, 0],     # go (0.0,0.0) and rotate 0
                   [2,0.0,0.0, 90],    # rotate 90
                   [2,0.0,0.0, 180 ],  # rotate 180
                   [2,0.0,0.0, 270],   # rotate 270
                   [2,0.0,0.0, 360],   # rotate 360
                   #[2,0.0,0.0, 0.0],

                   [0,1.0,0.0, 0],      # go (1.0,0.0) and rotate 0
                   [2,1.0,0.0, 90],     # rotate 90
                   [2,1.0,0.0, 180],    # rotate 180
                   [2,1.0,0.0, 270],    # rotate 270
                   [2,1.0,0.0, 360],    # rotate 360
                   #[2,0.0,0.0, 0],

                   [0,2.0,0.0, 0],      # go (2.0,0.0) and rotate 0
                   [2,2.0,0.0, 90],     # rotate 90

                   [0,2.0,0.4, 90],     # go (2.0,0.4) and rotate 90
                   [2,2.0,0.4, 180],    # rotate 180
                   [2,2.0,0.4, 270],    # rotate 270

                   [0,2.0,0.0, 270],    # go (2.0,0.0) and rotate 270
                   [2,2.0,0.0, -180],   # rotate -180

                   [50,0.0,0.0, 0],     # set Navigation mode

                    #[2,0.0,0.0, 0],
                    #[99,0.0,0.0, 0],

                    #[50,0.0,0.0, 0],

                   [0,1.0,0.0, -180 ],  # go (1.0,0.0) and rotate -180
                   [2,1.0,0.0, 270 ],   # rotate 270
                   [2,1.0,0.0, 360],    # rotate 360
                   [2,1.0,0.0, 90],     # rotate 90
                   [2,1.0,0.0, 180],    # rotate 180
                    #[2,0.0,0.0, 0.0],

                   [0,0.0,0.0, 180],    # (0.0,0.0) and rotate 180
                   [2,0.0,0.0, 270],    # rotate 270
                   [2,0.0,0.0, 360],    # rotate 360
                   [22,0.0,0.0, 0]]


        m_goalList =[[60,0.0, 0.0],      # course correct ON
                    [0, 0.0, 0],       # [0,0]
                    [0, 0.0, 90],
                    [0, 0.0, 180],
                    [0, 0.0, 270],
                    [0, 0.0, 360],
                    #[2,0.0,0.0, 0.0],

                    [0, 1.0, 0],        # [1,0]
                    [0, 0.0, 90],
                    [0, 0.0, 180],
                    [0, 0.0, 270],
                    [0, 0.0, 360],
                    #[2,0.0,0.0, 0.0],

                    [0, 1.0, 0],        # [2,0]
                    [0, 0.0, 90],       # [2,0]

                    [0, 0.4, 90],       # [2,0.4]
                    [0,0.0, 180],
                    [0,0.0, 270 ],

                    [0,0.4,270],         # [2,0]
                    [0,0.0, -180 ],

                    [50,0.0, 0.0],

                    [60,0.0, 0.0],      # course correct ON
                    #[62,0.0, 0.0],      # after_correct_wait ON
                    #[99,0.0,0.0, 0.0],

                    #[50,0.0,0.0, 0.0],

                    [0,1.0, 180 ],     # [1.5,0]
                    [0,0.0, 270],
                    [0,0.0, 360],
                    [0,0.0, 90],
                    [0,0.0, 180 ],
                    #[2,0.0,0.0, 0.0],

                    [0,1.0, 180 ],      # [0,0]
                    [0,0.0, 270],
                    [0,0.0, 360],
                    [22,0.0, 0.0]]


        m_goalList2 =[[0, 0.0, 0],       # [0,0]
                    [0, 1.0, 0],        # [1,0]
                    #[2,0.0,0.0, 0.0],
                    [0, 1.0, 0],        # [2,0]
                    #[0, 1.0, 0],        # [3,0]
                    ]


        m_goalList3 =[[60,0.0, 0.0],      # course correct ON
                    [0, 0.0, 0],       # [0,0]
                    [0, 0.0, 90],
                    [0, 0.0, 180],
                    [0, 0.0, 270],
                    [0, 0.0, 360],
                    #[2,0.0,0.0, 0.0],

                    [0, 0.0, 90],        # [1,0]
                    [0, 1.0, 90],
                    [0, 0.0, 180],
                    [0, 0.0, 270],

                    [0, 2.0, 270],
                    [0, 0.0, -180],
                    [0, 0.0, -270],
                    [0, 1.0, -270],

                    #[2,0.0,0.0, 0.0],

                    [0,0.0, -360]]


        # 1平米を動く 0.8 x 0.8
        m_square_1 =[[0, 0.0, 90],  # (0,0)
                   [0, 0.4, 90],   # (0,0.4)
                   [0, 0.0, -360],
                   [0, 0.8, 0],     # (0.8,0.4)
                   [0, 0.0, -90],
                   [0, 0.2, -90],   # (0.8,0.2)
                   [0, 0.0, -180],
                   [0, 0.8, -180],  # (0.0,0.2)
                   [0, 0.0, 270], 
                   [0, 0.2, 270],   # (0.0,0.0)
                   #[0, 0.0, 0],   # 
                   #[0, 0.8, 0],   #  (0.8,0.0)
                   #[0, 0.0, -90],  
                   #[0, 0.2, -90],   # (0.8,-0.2)
                   #[0, 0.0, -180],
                   #[0, 0.8, -180],  # (0.0,-0.2)
                   #[0, 0.0, 270],   
                   #[0, 0.2, 270],   # (0.0,-0.4)
                   #[0, 0.0, 0],   
                   #[0, 0.8, 0],   # (0.8,-0.4)
                   ]

        m_rotate_1 =[[0, 0, 0],
                     [0, 0, 45],
                     [0, 0, 90],  # (0,0)
                     [0, 0, 180],
                     [0, 0, 270],
                     [0, 0, 360],
                     #[0, 1.0, -90],   # (0,0.4)
                    ]


        m_rotate_2 =[[0, 0, -0],
                     [0, 0, -90],  # (0,0)
                     [0, 0, -180],
                     [0, 0, -270],
                     [0, 0, -360],
                    ]

        #print goalList

        get_map=GetMap()

        #get_map.get()

        #print("get_map")

        #sys.exit()

        # Constract MultiGoals Obj
        rospy.loginfo("Multi Goals Executing...")
        mg_ex = MultiGoals(map_frame,get_map,use_sim_time)   
        mg_ex.mloop_ex(goalList)
        #mg_ex.mloop_ex(m_rotate_1)
        #mg_ex.mloop_ex(m_rotate_2)
        #mg_ex.mloop_ex(m_square_1)
        #mg_ex.mloop_ex(m_goalList2)
        #mg_ex.mloop_ex(m_goalList)
        #mg_ex.mloop_ex(m_goalList3)

        # 最後の Publish 完了を待つ
        rate = rospy.Rate(1)   # 1[Hz]
        rate.sleep()

        sys.exit()
        #rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")
    