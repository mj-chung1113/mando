#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import atan2, sin
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from simple_pid import PID
from tf.transformations import euler_from_quaternion

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        
        self.ego_vehicle_status = EgoVehicleStatus()
        
        self.current_position = Odometry() 
        self.vehicle_yaw = 0 
        
        #자차 제원 
        self.vehicle_length = 3
        self.lfd = 5

        # PID 컨트롤러 설정
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid.output_limits = (-1, 1)  # -1 (최대 감속)에서 1 (최대 가속) 사이의 출력 범위 설정

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                # path의 절반 지점 찾기
                mid_index = len(self.path.poses) // 2
                mid_pose = self.path.poses[mid_index].pose.position

                # local_path_point 설정
                local_path_point = [mid_pose.x, mid_pose.y]

                theta = atan2(local_path_point[1], local_path_point[0])
                
                # steering
                self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                default_vel = 45*(1-0.7*normalized_steer)

                # 현재 속도와 목표 속도 간의 오차 계산
                current_velocity = self.ego_vehicle_status.velocity.x
                self.pid.setpoint = default_vel
                speed_error = self.pid(current_velocity)
                
                # PID 출력에 따른 accel/brake 제어
                if speed_error > 0:
                    self.ctrl_cmd_msg.accel = min(speed_error, 1.0)
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = min(abs(speed_error), 1.0)

                #self.ctrl_cmd_msg.velocity = default_vel  # velocity는 직접적으로 사용되지 않음
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    '''
    def mission_callback(self, msg):
        self.mission_info = msg
    '''
    
    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg):
        self.ego_vehicle_status = msg  # EgoVehicleStatus 업데이트

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
