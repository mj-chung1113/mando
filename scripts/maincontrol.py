#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.ego_vehicle_status = EgoVehicleStatus()

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.path = Path()

        # 자차 제원 
        self.vehicle_length = 3
        self.lfd = 10

        self.min_lfd = 10
        self.max_lfd = 50
        self.lfd_gain = 0.5
        self.target_velocity = 25  #차량의 한계속도. 수정 필요, 구간에 따라 동적으로 조절할 예정 

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.5) #3.6을 나누는 이유는, 로스에서는 m/s단위로 사용,우리는 km/h쓰니깐 단위변환하려고..
        while True:
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.path, 50)
                break
            else:
                rospy.loginfo('Waiting path data')

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                prev_time = time.time()
                if self.ego_vehicle_status.velocity.x * 3.6 <= 15: 
                    # Pure Pursuit 사용
                    steering = self.calc_pure_pursuit()
                else:
                    # Stanley 사용
                    steering = self.calc_stanley()
                    
                self.current_waypoint = self.get_current_waypoint(self.path)
                normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6 * (1 - 0.8 * normalized_steer)
                
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                    self.ctrl_cmd_msg.brake = 0.3
                else: 
                    rospy.loginfo("No found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                output = self.pid.pid(self.target_velocity, self.ego_vehicle_status.velocity.x * 3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # 제어입력 메세지 Publish
                rospy.loginfo("Control command published")
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.is_global_path = True
        self.path = msg  

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        self.ego_vehicle_status = msg  # EgoVehicleStatus 업데이트    
        self.is_status = True

    def get_current_waypoint(self, path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i, pose in enumerate(path.poses):
            dx = self.current_postion.x - pose.pose.position.x
            dy = self.current_postion.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self):
        # 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (self.ego_vehicle_status.velocity.x) * 3.6 * self.lfd_gain
        rospy.loginfo(3.6 * self.ego_vehicle_status.velocity.x)
        if self.lfd < self.min_lfd:
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd:
            self.lfd = self.max_lfd
        rospy.loginfo(self.lfd)
        
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        # Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)

        return steering
    #추가된 파트 (0912) 스탠리는 동적 모델이 적용되는 조금 더 빠른 속도에서 pure pursuit보다 유리하다는 평이 많아 추가함. 
    #15키로 이하에서 pure pursuit, 초과할 경우 stanley를 사용하려구 함. 지욱이형이 잘 깎아주삼 
    def calc_stanley(self):
        # Stanley 제어 알고리즘을 사용하여 steering 각도를 계산합니다.
        
        k = 0.3  # steering gain 클수록 급하게 꺾기가 가능, 작을수록 완만하고 안정적으로 꺾음. 
        # 이거를 상수가 아닌 변수로 두는 방법이 좋을 것같아. 도로 곡률이 클 수록 크게, 속도가 빠를 수록 작게.. 동적으로 조절할 수 있도록 하면 좋을듯 
        
        self.is_look_forward_point = False

        # 좌표 변환 행렬 생성
        translation = [self.current_postion.x, self.current_postion.y]
        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        # 가장 가까운 포인트와의 차이 계산
        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        # 간단한 Stanley 제어 각도 계산 예제
        theta = atan2(local_path_point[1], local_path_point[0])
        e = local_path_point[1]
        steering = theta + atan2(k * e, self.ego_vehicle_status.velocity.x * 3.6)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.1
        self.i_gain = 0.01
        self.d_gain = 0.05
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        # PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output


class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            # 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a * a + b * b - c)

            # 곡률 기반 속도 계획
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass