#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
cmd_vel2.py

http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line
https://kato-robotics.hatenablog.com/entry/2019/02/18/053255
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information
http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
http://docs.ros.org/en/lunar/api/tf/html/python/transformations.html

'''
from operator import truediv
from re import T
import rospy
from geometry_msgs.msg import Twist

#geometry_msgs/Transform transform
from geometry_msgs.msg import TransformStamped

import time
import sys
import tf2_ros
import tf
import math

class  cmd_control():
    def __init__(self):
        self.deg = 57.29577951308232    # [deg/rad]
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        self.theta=0.0

        self.course_correct = False     # コース補正
        self.after_correct_wait = False # コース補正後 Stop for Debug

        # リスナーの登録
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.base_tf = TransformStamped()
        time.sleep(1)

        i=3
        rate = rospy.Rate(50)   # 0.3 [Sec]
        while(i >= 0):
            t=self.get_tf(func=0)
            self.base_tf.transform = t.transform
            rate.sleep()
            self.pub.publish(self.vel_msg)
            i-=1

    '''
    move()
    自分からの相対位置へ移動
      dist: 自分からの距離
      d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
    '''
    def move(self,dist,d_yaw):
        self.get_tf(func=1)
        r_yaw = d_yaw/self.deg
        # 目的地を計算
        y = self.base_tf.transform.translation.y + dist * math.sin(r_yaw)
        x = self.base_tf.transform.translation.x + dist * math.cos(r_yaw)
        self.rotate_abs(d_yaw)
        if dist != 0.0:
            self.go_abs(x,y)

    '''
    move_abs()
      x,y: 絶対番地への移動(基準座標)
      d_yaw: 基準座標での角度。 [degree]
    '''
    def move_abs(self,x,y,d_yaw):
        self.rotate_abs(d_yaw)
        self.go_abs(x,y)

    '''
    get_tf(func=0)
    '''
    def get_tf(self,func=0):
        #rate = rospy.Rate(50)   # 50[Hz]
        #rate = rospy.Rate(60)   # 60[Hz]
        rate = rospy.Rate(70)    # 70[Hz]
        while(1):
            try:
                #t = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
                t = self.tfBuffer.lookup_transform('map','base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            break

        if func==0:
            return t

        self.base_tf.transform = t.transform

        #print ('{0:.3f}, {1:.3f}, {2:.3f}'.format(
        #    t.transform.translation.x,
        #    t.transform.translation.y,
        #    t.transform.translation.z
        #))
        #print('{0:.3f}, {1:.3f}, {2:.3f}, {3:.3f}'.format(
        #    t.transform.rotation.x,
        #    t.transform.rotation.y,
        #    t.transform.rotation.z,
        #    t.transform.rotation.w
        #))
        if func==1:
            # angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
            q = [self.base_tf.transform.rotation.z,self.base_tf.transform.rotation.x,self.base_tf.transform.rotation.y,self.base_tf.transform.rotation.w]
            self.angles = tf.transformations.euler_from_quaternion(q)
            self.dx = self.angles[1]*self.deg
            self.dy = self.angles[2]*self.deg
            self.dz = self.angles[0]*self.deg

            #self.angles = self.QuaternionToEuler(q)
            #self.dx = self.angles[0]*self.deg
            #self.dy = self.angles[1]*self.deg
            #self.dz = self.angles[2]*self.deg
            #print "dx,dy,dz=",round(self.dx,3),round(self.dy,3),round(self.dz,3)

        elif func==2:
            q = [self.base_tf.transform.rotation.z,self.base_tf.transform.rotation.x,self.base_tf.transform.rotation.y,self.base_tf.transform.rotation.w]
            self.angles = tf.transformations.euler_from_quaternion(q)
            self.rx = self.angles[1]
            self.ry = self.angles[2]
            self.rz = self.angles[0]
            print "rx,ry,rz=",round(self.rx,3),round(self.ry,3),round(self.rz,3)

    '''
    def QuaternionToEuler(self,q):
        q0q0 = q[0]*q[0]
        q1q1 = q[1]*q[1]
        q2q2 = q[2]*q[2]
        q3q3 = q[3]*q[3]
        dq0 = 2.0*q[0]
        dq1 = 2.0*q[1]
        dq2 = 2.0*q[2]
        dq1q2 = dq1 * q[2]
        dq1q3 = dq1 * q[3]
        dq0q2 = dq0 * q[2]
        dq0q3 = dq0 * q[3]
        dq0q1 = dq0 * q[1]
        dq2q3 = dq2 * q[3]

        thx = math.atan2(dq0q1+dq2q3, q0q0+q3q3-q1q1-q2q2)
        thy = math.asin(dq0q2-dq1q3)
        thz = math.atan2(dq1q2+dq0q3, q0q0+q1q1-q2q2-q3q3)

        return thx,thy,thz
    '''

    '''
    go_abs(x,y,isForward=True,speed=0.05)
    直進する。
    '''
    def go_abs(self,x,y,isForward=True,speed=0.05):
        print "go abs"

        if(isForward):
            i_spped = abs(speed)
        else:
            i_spped = -abs(speed)

        self.vel_msg.linear.x = i_spped
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        rate = rospy.Rate(30)   # 30 [Hz]
        start_t = self.get_tf(func=0)
        off_x = x - start_t.transform.translation.x
        off_y = y - start_t.transform.translation.y
        off_z=0.0

        start_distance = math.sqrt(off_x*off_x+off_y*off_y+off_z*off_z)
        current_distance=0.0

        print 'start_distance=',round(start_distance,3)

        i = 0
        ex_f=False

        #Loop to move the turtle in an specified distance
        while(current_distance < start_distance):
            #print "pub cmd_vel"

            #Publish the velocity
            self.pub.publish(self.vel_msg)

            rate.sleep()

            self.get_tf(func=1)

            off_x = self.base_tf.transform.translation.x - start_t.transform.translation.x
            off_y = self.base_tf.transform.translation.y - start_t.transform.translation.y
            off_z=0.0

            current_distance = math.sqrt(off_x*off_x+off_y*off_y+off_z*off_z)

            i+=1
            if i > 5:
                # 自分からの目的地の方角
                off_target_x = x - self.base_tf.transform.translation.x
                off_target_y = y - self.base_tf.transform.translation.y

                theta_d = math.degrees(math.atan2(off_target_y,off_target_x))   # degree

                # 自分の方向を減算
                theta_ad = theta_d - self.dz
                # 後ろ向き
                if abs(theta_ad) > 180:
                    if theta_ad > 1.0:
                        theta_ad -= 360.0
                    else:
                        theta_ad += 360.0

                print 'self.dz=',round(self.dz,3),' theta_d=',round(theta_d,3),' theta_ad=',round(theta_ad,3)

                if ex_f == True:
                    sys.exit()

                if self.course_correct == True:
                    # 10 [cm] 以上距離がある 時に方向を補正
                    if (start_distance - current_distance) > 0.1 and current_distance > 0.1 :
                        if abs(theta_ad) > 5.0:
                            self.rotate_off(theta_ad,speed=3.0)
                            self.vel_msg.linear.x = i_spped
                            #sys.exit()
                            #ex_f=True
                            i=i
                            if self.after_correct_wait == True:

                                print "after_correct_wait"
                                self.get_tf(func=1)
                                print "dx,dy,dz=",round(self.dx,3),round(self.dy,3),round(self.dz,3)
                                while(1):
                                    rate.sleep()

                print 'current_distance=',round(current_distance,3)

                i=0

        print "stop"
        #After the loop, stops the robot
        self.vel_msg.linear.x = 0.0
        #Force the robot to stop
        self.pub.publish(self.vel_msg)


    '''
    go_straight(speed,distance,isForward=True)
    '''
    def go_straight(self,speed,distance,isForward=True):
        #velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #vel_msg = Twist()

        print "go straight"

        #Checking if the movement is forward or backwards
        if(isForward):
            self.vel_msg.linear.x = abs(speed)
        else:
            self.vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        #Setting the current time for distance calculus
        #t0 = rospy.Time.now().to_sec()

        current_distance = 0.0

        rate = rospy.Rate(30)   # 30 [Hz]

        start_t = self.get_tf(func=0)

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #print "pub cmd_vel"

            #Publish the velocity
            self.pub.publish(self.vel_msg)

            rate.sleep()

            cur_t=self.get_tf(func=0)

            off_x=cur_t.transform.translation.x - start_t.transform.translation.x
            off_y=cur_t.transform.translation.y - start_t.transform.translation.y
            off_z=cur_t.transform.translation.z - start_t.transform.translation.z

            current_distance = math.sqrt(off_x*off_x+off_y*off_y+off_z*off_z)

        print "stop"
        #After the loop, stops the robot
        self.vel_msg.linear.x = 0.0
        #Force the robot to stop
        self.pub.publish(self.vel_msg)

    '''
    rotate_abs()
        stop_dz(d_theta) : [deg] 基本座標上の角度
        speed :  5.0  [deg/s]
    '''
    def rotate_abs(self,stop_dz,speed=5.0):
        # 目的の角度と速度を設定
        # stop_dz = 180.0 # [deg]
        # speed = 10.0 # [deg/s]

        print 'start rotate_abs stop_dz=',stop_dz

        # turn plus
        if stop_dz >= 0.0:
            turn_plus = 1   # reverse clock(left) rotate
            #self.vel_msg.angular.z = speed * 3.1415 / 180.0 # [rad]
            #self.vel_msg.angular.z = speed # [rad]
            self.vel_msg.angular.z = speed / self.deg  # [rad]

        # turn minus
        else:
            turn_plus = -1   # regura clock(right) rotate
            #self.vel_msg.angular.z = speed * 3.1415 / -180.0 # [rad]
            self.vel_msg.angular.z = speed / self.deg * -1.0  # [rad]

        print 'turn_plus=',turn_plus


        rz_dlt = abs(speed / self.deg) * 0.25
        #z_wind = abs(speed / self.deg) * 2.0
        rz_wind = rz_dlt * 3.0

        # Twist 型のデータ
        #t = Twist()
        #t.linear.x = 0
        self.vel_msg.linear.x = 0.0

        stop_rz = math.radians(stop_dz)
        # 180度表現に変換
        if abs(stop_rz) > math.radians(180):
            if stop_rz > 0.0:
                stop_rz -= math.radians(360)
            else:
                stop_rz +=  math.radians(360)

        self.get_tf(func=2)
        #print 'rx,ry,rz=',self.rx,self.ry,self.rz

        rz_off = stop_rz - self.rz

        if abs(rz_off) <= rz_dlt:
            return

        print 'dz_off=',math.degrees(rz_off)

        # 逆回りが近い
        if abs(rz_off) > math.radians(180.0):
            if rz_off > 0.0:
                rz_off -= math.radians(360)
            else:
                rz_off += math.radians(360)

        if abs(rz_off) <= math.radians(10.0):      # 角度差が 10.0 度以内 であれば、補正回転にする
            print 'adjust angle'
            # 左回り
            if rz_off >= 0.0:
                self.vel_msg.angular.z = abs(self.vel_msg.angular.z)
                turn_plus = 1
            # 右回り
            else:
                self.vel_msg.angular.z = abs(self.vel_msg.angular.z) * -1.0
                turn_plus = -1
        else:
            print '>non adjust angle'

        print 'turn_plus=',turn_plus

        #print 'test1'
        #sys.exit()

        # stop_rz を目指す
        #rate = rospy.Rate(30)   # 30 [Hz]
        rate = rospy.Rate(40)   # 40 [Hz]
        rz_min = 10.0
        ok_f = False
        while(1):
            self.pub.publish(self.vel_msg)

            self.get_tf(func=2)
            print 'stop_rz=',round(stop_rz,3)

            # 最も近い角度で終了します
            if ok_f == True:
                print "ok nearly"
                break
            # 目的の角度です。
            if abs(self.rz - stop_rz) <= rz_dlt:
                print "ok just"
                break

            #rz_off = abs(abs(stop_rz) - abs(self.rz))
            rz_off = stop_rz - self.rz
            if rz_off > math.radians(180.0):
                rz_off = -(math.radians(360.0) - rz_off)
            elif rz_off < math.radians(-180.0):
                rz_off += math.radians(360.0)

            rz_off=abs(rz_off)
            if rz_off <= rz_wind:
                # 最小の角度を求める
                #print 'rz_off=',rz_off 
                if rz_off <= rz_min:
                    rz_min= rz_off
                # 最小の角度を通り過ぎ
                else:
                    # 1クロック逆戻り
                    self.vel_msg.angular.z *= -1.0
                    ok_f = True

            rate.sleep()

        print "stop"
        self.vel_msg.angular.z = 0.0 # [rad]
        #Force the robot to stop
        self.pub.publish(self.vel_msg)
        rate.sleep()

        self.get_tf(func=2)


    '''
    rotate_off()
        d_theta : [deg] ロボット座標上の角度
        speed :  5.0  [deg/s]
    '''
    def rotate_off(self,d_theta,speed=5.0):
        # 目的の角度と速度を設定
        #d_theta = 180.0 # [deg]
        #speed = 10.0 # [deg/s]

        print 'start rotate_off d_theta=',d_theta

        rz_dlt = abs(math.radians(speed)) * 0.25
        rz_wind = rz_dlt * 3.0

        r_theta = math.radians(d_theta)
        if abs(r_theta) <= rz_dlt:
            return

        if d_theta >= 0.0:
            turn_plus = 1   # reverse clock(left) rotate
            #self.vel_msg.angular.z = speed * 3.1415 / 180.0 # [rad]
            #self.vel_msg.angular.z = speed # [rad]
            self.vel_msg.angular.z = math.radians(speed)  # [rad]

        else:
            turn_plus = -1   # clock(right) rotate
            #self.vel_msg.angular.z = speed * 3.1415 / -180.0 # [rad]
            self.vel_msg.angular.z = math.radians(speed) * -1.0  # [rad]

        self.get_tf(func=2)
        #print 'rx,ry,rz=',self.rx,self.ry,self.rz

        print 'r_theta=',round(r_theta,3)

        # Twist 型のデータ
        #t = Twist()
        #t.linear.x = 0
        self.vel_msg.linear.x = 0.0

        stop_rz= self.rz + r_theta

        # 180度表現に変換
        if abs(stop_rz) > math.radians(180):
            if stop_rz > 0.0:
                stop_rz -= math.radians(360)
            else:
                stop_rz +=  math.radians(360)

        # stop_rz を目指す
        #rate = rospy.Rate(30)   # 30 [Hz]
        rate = rospy.Rate(40)   # 40 [Hz]
        rz_min = 10.0
        ok_f = False
        while(1):
            self.pub.publish(self.vel_msg)

            self.get_tf(func=2)
            print 'stop_rz=',round(stop_rz,3)

            # 最も近い角度で終了します
            if ok_f == True:
                print "ok nearly"
                break
            
            # 目的の角度です。
            if abs(stop_rz - self.rz) <= rz_dlt:
                print "ok just"
                break

            rz_off = stop_rz - self.rz
            if rz_off > math.radians(180.0):
                rz_off = -(math.radians(360.0) - rz_off)
            elif rz_off < math.radians(-180.0):
                rz_off += math.radians(360.0)

            rz_off=abs(rz_off)
            if rz_off <= rz_wind:
                #print "rz_off=",rz_off," rz_min=",rz_min
                # 最小の角度を求める
                if rz_off <= rz_min:
                    rz_min = rz_off
                # 最小の角度を通り過ぎ
                else:
                    # 1クロック逆戻り
                    self.vel_msg.angular.z *= -1.0
                    ok_f = True
                    #break

            rate.sleep()

        print "stop"
        self.vel_msg.angular.z = 0.0 # [rad]
        #Force the robot to stop
        self.pub.publish(self.vel_msg)
        rate.sleep()

        self.get_tf(func=2)


if __name__ == "__main__":
    try:    
        # ROS Init    
        # Starts a new node
        rospy.init_node('cmd_vel2', anonymous=True)

        cmd_ctl=cmd_control()

        test_id = 3

        #Testing our function
        if test_id==1:
            cmd_ctl.go_straight(0.1,1.0,True)

        if test_id==2:
            #cmd_ctl.rotate_abs(-30.0,5.0)
            #cmd_ctl.rotate_abs(-25.0,5.0)

            #cmd_ctl.rotate_abs(30.0,5.0)
            #cmd_ctl.rotate_abs(25.0,5.0)

            #cmd_ctl.rotate_abs(180.0,5.0)
            #cmd_ctl.rotate_abs(175.0,5.0)


            #cmd_ctl.rotate_abs(-180.0,5.0)
            #cmd_ctl.rotate_abs(-175.0,5.0)


            #cmd_ctl.rotate_abs(179.0,5.0)
            #cmd_ctl.rotate_abs(190.0,5.0)

            #cmd_ctl.rotate_abs(180.0,5.0)
            cmd_ctl.rotate_abs(-180.0,5.0)
            #cmd_ctl.rotate_abs(270.0,5.0)

        if test_id==3:
            cmd_ctl.rotate_off(-5.0,speed=3.0)
            cmd_ctl.get_tf(func=1)
            print "dx,dy,dz=",round(cmd_ctl.dx,3),round(cmd_ctl.dy,3),round(cmd_ctl.dz,3)

            cmd_ctl.rotate_off(-5.0,speed=3.0)
            cmd_ctl.get_tf(func=1)
            print "dx,dy,dz=",round(cmd_ctl.dx,3),round(cmd_ctl.dy,3),round(cmd_ctl.dz,3)


        rate = rospy.Rate(1)   # 1[Hz]
        rate.sleep()

        sys.exit()

        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")
