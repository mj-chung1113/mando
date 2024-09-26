#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from mando.msg import mission 
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('pure_pursuit', anonymous=True)

        # Subscriber: 경로, 위치(odometry), 차량 상태를 구독
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("/stop_signal", Bool, self.stop_callback)

        # Publisher: 차량 제어 명령을 발행
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.lfd_pub = rospy.Publisher('/lfd', Float32, queue_size=2)

        # 제어 명령 초기화
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.ego_vehicle_status = EgoVehicleStatus()

        # 상태 플래그 초기화
        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False
        self.is_look_forward_point = False
        self.stop_signal = False 
        self.stop_signal_received = False  # 첫 번째 신호 처리
        self.stop_time = 0
        # 전방 목표 지점과 현재 위치 저장 변수
        self.forward_point = Point()
        self.current_postion = Point()
        self.path = Path()

        # 자차 제원 
        self.vehicle_length = 3  # 차량의 wheel base = 3m, pure pursuit 계산 시 길이 말하는것 
        self.lfd = 20

        self.min_lfd = 6
        self.max_lfd = 40
        self.lfd_gain = 0.5
        self.target_velocity = 30  # 차량의 한계속도. 수정 필요, 구간에 따라 동적으로 조절할 예정 

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.4)  # km/h -> m/s 변환

        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                # 곡률 기반 속도 계획을 반복적으로 실행, 현재 위치와 차량 헤딩 정보를 전달
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.path, 15, self.current_postion, self.vehicle_yaw)
                #self.velocity_list = self.vel_planning.curvedBaseVelocity1(self.path, point_num=2)
                
                if self.ego_vehicle_status.velocity.x * 3.6 <= 15:
                    # Pure Pursuit 사용
                    steering = self.calc_pure_pursuit()
                else:
                    # Stanley 사용
                    steering = self.calc_stanley()

                self.current_waypoint = self.get_current_waypoint(self.path)
                normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6 * (1 - 0.85 * normalized_steer)

                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering

                else: 
                    rospy.loginfo("No found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.brake = 0.3

                # 급정지 필요시
                if self.stop_signal:
                    # stop_signal이 처음 들어왔을 때 3초간 강제 정지
                    if not self.stop_signal_received:
                        self.stop_signal_received = True  # 첫 번째 신호 처리
                        self.stop_time = rospy.Time.now()  # 시작 시간 기록
                        rospy.loginfo(f"Initial stop! Braking for 3 seconds.")
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0
                    else:
                        # 3초가 지난 후 동작 (3초 동안 제동 유지)
                        elapsed_time = rospy.Time.now() - self.stop_time
                        if elapsed_time.to_sec() >= 3:
                            rospy.loginfo(f"3 seconds passed, maintaining stop signal.")
                            self.ctrl_cmd_msg.accel = 0.0
                            self.ctrl_cmd_msg.brake = 1.0
                else:
                    # stop_signal이 꺼지면 다시 원래 동작으로 돌아옴
                    self.stop_signal_received = False  # stop signal 리셋
                    output = self.pid.pid(self.target_velocity, self.ego_vehicle_status.velocity.x * 3.6)
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        rospy.loginfo(f"acc: {output}")
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output
                        rospy.loginfo(f"brake: {output}")

                    # 제어입력 메세지 Publish
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    self.lfd_pub.publish(self.lfd)
                    output = 0
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
    
    #mission number 수신용 콜백함수 
    def mission_callback(self, msg):
        try:
            self.mission_info = msg
        except ValueError as e:
            rospy.logerr(f"Invalid driving mode value: {self.mission_info.mission_num}")
    
    def stop_callback(self,msg):
        self.stop_signal = msg.data

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
        self.lfd = min(max(self.min_lfd, self.lfd), self.max_lfd)
        #rospy.loginfo(self.lfd)

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])
        
        # 경로를 차량 기준으로 변환
        det_trans_matrix = np.linalg.inv(trans_matrix)

        # 경로 상에서 Look Ahead Distance에 도달하는 지점 찾기
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
    
    # Stanley 제어 알고리즘
    def calc_stanley(self):
        # 스탠리 제어 알고리즘을 차량의 앞바퀴 기준으로 수정
        self.lfd = (self.ego_vehicle_status.velocity.x) * 3.6 * self.lfd_gain
        self.lfd = min(max(self.min_lfd, self.lfd), self.max_lfd)

        #stanley 동적 gain
        if len(self.path.poses) > 3:
            current_waypoint = self.get_current_waypoint(self.path)
            if current_waypoint < len(self.path.poses) - 2:
                curvature = self.calculate_curvature_3points(
                    self.path.poses[current_waypoint],
                    self.path.poses[current_waypoint + 1],
                    self.path.poses[current_waypoint + 2]
                )
                k = self.calculate_stanley_gain(curvature)
            else:
                rospy.loginfo("Not enough points ahead to calculate curvature, using default gain.")
                k = 0.2
        else:
            rospy.loginfo("Not enough points to calculate curvature, using default values.")
            k = 0.2

        self.is_look_forward_point = False

        # 차량의 앞바퀴 위치를 계산 (차량의 중심으로부터 wheel_base 만큼 앞쪽으로 이동)
        front_axle_x = self.current_postion.x + cos(self.vehicle_yaw) * self.vehicle_length
        front_axle_y = self.current_postion.y + sin(self.vehicle_yaw) * self.vehicle_length
        front_axle_position = [front_axle_x, front_axle_y]

        translation = [front_axle_x, front_axle_y]
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

        # 간단한 Stanley 제어 각도 계산 예제
        theta = atan2(local_path_point[1], local_path_point[0])
        e = local_path_point[1]
        steering = theta + atan2(k * e, self.ego_vehicle_status.velocity.x * 3.6)

        return steering
    
    # 3개의 점을 사용해 곡률 계산
    def calculate_curvature_3points(self, p1, p2, p3):
        x1, y1 = p1.pose.position.x, p1.pose.position.y
        x2, y2 = p2.pose.position.x, p2.pose.position.y
        x3, y3 = p3.pose.position.x, p3.pose.position.y

        A = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        B = sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
        C = sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)

        # 삼각형 넓이 계산
        s = (A + B + C) / 2
        area_squared = s * (s - A) * (s - B) * (s - C)

        # 곡률 계산
        if area_squared <= 0:
            return 0

        area = sqrt(area_squared)
        radius = (A * B * C) / (4 * area)
        curvature = 1 / radius

        return curvature

    # 스탠리 알고리즘에서 사용하는 동적 게인 계산
    def calculate_stanley_gain(self, max_curvature):
        speed = abs(self.ego_vehicle_status.velocity.x * 3.6)  # 속도를 km/h로 변환
        if speed == 0:
            speed_gain = 1.0
        else:
            speed_gain = 1 / (speed + 1e-3)  # 속도가 클수록 스티어링 게인 낮아짐

        curvature_gain = max_curvature  # 경로 상의 곡률에 따른 게인 조정
        k = 0.2 + (speed_gain * curvature_gain)

        # 게인의 상한/하한 설정
        min_k = 0.1
        max_k = 2.0
        k = np.clip(k, min_k, max_k)

        return k

class pidControl:
    def __init__(self):
        self.p_gain = 0.05
        self.i_gain = 0.02
        self.d_gain = 0.05
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.025
        self.max_i_control = 1.0  # 적분 항의 최대값 설정
        self.min_i_control = -1.0 # 적분 항의 최소값 설정

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel
        
        # 비례 제어(P)
        p_control = self.p_gain * error

        # 적분 제어(I) - 적분 포화 방지
        self.i_control += self.i_gain * error * self.controlTime
        self.i_control = max(min(self.i_control, self.max_i_control), self.min_i_control)

        # 미분 제어(D)
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        # PID 출력 계산
        output = p_control + self.i_control + d_control

        # 다음 주기를 위한 오류 값 저장
        self.prev_error = error

        return output


class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction
    # 경로 기반으로 곡률을 계산하여 속도 계획 생성
    
    def curvedBaseVelocity1(self, global_path, point_num=2):
        out_vel_plan = []

        curvature_threshold = 0.5  # 곡률 임계값 설정
        slow_down_distance = 150  # 미리 속도 감소할 거리 (웨이포인트 개수 기준)
        look_ahead_distance = 200  # 앞쪽 웨이포인트에서 곡률을 확인할 거리
        reduced_speed = 15 / 3.6  # 속도 20km/h로 제한

        # 초기 몇 개의 웨이포인트는 최대 속도로 설정
        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        # 경로 전체를 따라 곡률 계산 및 속도 설정
        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a * a + b * b - c)

            # 곡률에 따른 최대 속도 계산
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed

            # 곡률이 큰 구간에 대해 속도 제한
            if r < 1 / curvature_threshold:
                v_max = min(v_max, reduced_speed)

            # 앞으로 곡률이 클 경우 미리 속도 줄이기
            ahead_curvature_exceeded = False
            for lookahead in range(i, min(i + look_ahead_distance, len(global_path.poses) - point_num)):
                ahead_x_list = []
                ahead_y_list = []
                for box in range(-point_num, point_num):
                    ahead_x = global_path.poses[lookahead + box].pose.position.x
                    ahead_y = global_path.poses[lookahead + box].pose.position.y
                    ahead_x_list.append([-2 * ahead_x, -2 * ahead_y, 1])
                    ahead_y_list.append((-ahead_x * ahead_x) - (ahead_y * ahead_y))

                ahead_x_matrix = np.array(ahead_x_list)
                ahead_y_matrix = np.array(ahead_y_list)
                ahead_x_trans = ahead_x_matrix.T

                ahead_a_matrix = np.linalg.inv(ahead_x_trans.dot(ahead_x_matrix)).dot(ahead_x_trans).dot(ahead_y_matrix)
                ahead_a = ahead_a_matrix[0]
                ahead_b = ahead_a_matrix[1]
                ahead_c = ahead_a_matrix[2]
                ahead_r = sqrt(ahead_a * ahead_a + ahead_b * ahead_b - ahead_c)

                if ahead_r < 1 / curvature_threshold:
                    ahead_curvature_exceeded = True
                    break

            # 곡률이 큰 구간에 대해 속도 감소 적용
            if ahead_curvature_exceeded:
                for j in range(max(0, i - slow_down_distance), i):
                    out_vel_plan[j] = min(out_vel_plan[j], reduced_speed)

            out_vel_plan.append(v_max)

        # 경로 마지막 구간에서는 속도 줄이기
        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

    def curvedBaseVelocity(self, global_path, point_num, current_position, vehicle_yaw):
        out_vel_plan = []

        # 차량의 현재 위치 및 헤딩 정보
        for i in range(0, len(global_path.poses)):
            # 경로의 30번째 점을 기준으로 헤딩 계산
            if i + 30 < len(global_path.poses):
                target_point = global_path.poses[i + 30].pose.position
            else:
                # 경로의 끝 부분에서는 마지막 점을 사용
                target_point = global_path.poses[-1].pose.position

            # 현재 차량 위치와 30번째 경로점 사이의 각도 계산
            dx = target_point.x - current_position.x
            dy = target_point.y - current_position.y
            target_yaw = atan2(dy, dx)  # 차량이 가야할 방향(목표점과의 각도)

            # 현재 차량의 헤딩과 목표 헤딩 간의 차이 (heading error)
            heading_error = abs(vehicle_yaw - target_yaw)

            # 각도 차이를 0 ~ pi로 정규화 (차량이 뒤로 회전할 필요는 없으므로)
            if heading_error > pi:
                heading_error = 2 * pi - heading_error

            # 각도 차이에 따라 속도 계획 (단위: m/s)
            if heading_error > pi / 6:  # 각도 차이가 30도 이상일 때
                v_max = self.car_max_speed * 0.5  # 속도 절반으로 줄임
            elif heading_error > pi / 12:  # 각도 차이가 15도 이상일 때
                v_max = self.car_max_speed * 0.75  # 속도를 75%로 줄임
            else:
                v_max = self.car_max_speed  # 각도 차이가 작으면 최대 속도 유지

            out_vel_plan.append(v_max)

        # 경로 마지막 부분에서 감속
        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(0.5 * self.car_max_speed)

        # 마지막 10개의 점에서는 속도를 0으로 설정
        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        #rospy.loginfo(f"v max at 20th point: {out_vel_plan[20]}")
        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass