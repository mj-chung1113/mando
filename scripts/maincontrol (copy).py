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
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest
from enum import Enum

# 기어 상태를 나타내는 Enum 클래스 추가
class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

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
        rospy.Subscriber("/warn_signal", Float32, self.warn_callback)

        # Publisher: 차량 제어 명령을 발행
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.lfd_pub = rospy.Publisher('/lfd', Float32, queue_size=2)

        #기어 관련 
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.gear_change_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

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
        self.warn_signal = 0.0
        self.stop_signal_received = False  # 첫 번째 신호 처리
        self.stop_time = 0

        # 전방 목표 지점과 현재 위치 저장 변수
        self.forward_point = Point()
        self.current_postion = Point()
        self.path = Path()

        # 자차 제원 
        self.vehicle_length = 3  # 차량의 wheel base = 3m, pure pursuit 계산 시 길이 말하는것 
        self.lfd = 10

        self.min_lfd = 5
        self.max_lfd = 40
        self.lfd_gain = 0.3
        self.target_velocity = 40  # 차량의 한계속도. 수정 필요, 구간에 따라 동적으로 조절할 예정 

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.4)  # km/h -> m/s 변환

        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                # 곡률 기반 속도 계획을 반복적으로 실행, 현재 위치와 차량 헤딩 정보를 전달
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.path, 15, self.current_postion, self.vehicle_yaw)
                
                if self.ego_vehicle_status.velocity.x * 3.6 <= 5:
                    # Pure Pursuit 사용
                    steering = self.calc_pure_pursuit()
                else:
                    # Stanley 사용
                    steering = self.calc_stanley()

                self.current_waypoint = self.get_current_waypoint(self.path)
                normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                normalized_warn = 1-0.30 * self.warn_signal * self.warn_signal
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6 * (1 - 0.1 * normalized_steer)
                self.target_velocity = self.target_velocity * normalized_warn
                
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else: 
                    rospy.loginfo("No found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.brake = 0.3

                # 미션 종료 처리
                if self.is_mission_end:
                    rospy.loginfo("미션이 종료되었습니다.")
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                    rospy.sleep(2)#2초 대기

                    # 기어를 P단으로 변경
                    self.change_gear_to_p()

                    break  # 미션 종료 시 루프 탈출

                ## 정지 필요시
                if self.stop_signal:
                    if not self.stop_signal_received:
                        self.stop_signal_received = True  # 첫 번째 정지 신호 처리
                        self.stop_time = rospy.Time.now()  # 시작 시간 기록
                        rospy.loginfo("Initial stop! Braking for 4 seconds.")
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0
                        # 제어입력 메시지 Publish
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                        self.lfd_pub.publish(self.lfd)
                    else:
                        elapsed_time = rospy.Time.now() - self.stop_time
                        if elapsed_time.to_sec() >= 4:
                            rospy.loginfo("4 seconds passed, maintaining stop signal.")
                            self.ctrl_cmd_msg.accel = 0.0
                            self.ctrl_cmd_msg.brake = 1.0
                else:
                    self.stop_signal_received = False  # stop signal 리셋
                    output = self.pid.pid(self.target_velocity, self.ego_vehicle_status.velocity.x * 3.6)
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        rospy.loginfo(f"Acceleration: {output}")
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output
                        rospy.loginfo(f"Brake: {output}")

                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    self.lfd_pub.publish(self.lfd)

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
            if self.mission_info.mission_num == 11:  # 예시로 미션 번호가 11이면 종료로 처리
                self.is_mission_end = True
            else:
                self.is_mission_end = False
        except ValueError as e:
            rospy.logerr(f"Invalid driving mode value: {self.mission_info.mission_num}")

    def stop_callback(self,msg):
        self.stop_signal = msg.data
    def warn_callback(self,msg):
        self.warn_signal = msg.data 
    
    def change_gear_to_p(self):
        """기어를 P단으로 변경"""
        try:
            gear_cmd = MoraiEventCmdSrvRequest()
            gear_cmd.request.option = 3  # 기어 변경 옵션
            gear_cmd.request.ctrl_mode = 3  # 제어 모드 설정 (기어 변경 제어)
            gear_cmd.request.gear = Gear.P.value  # P 기어로 변경

            response = self.gear_change_service(gear_cmd)

            # 요청한 기어 상태가 응답의 기어 상태와 동일한지 확인
            if response.response.gear == Gear.P.value:
                rospy.loginfo("기어가 P단으로 변경되었습니다.")
            else:
                rospy.logerr("기어를 P단으로 변경하는데 실패했습니다.")
        except rospy.ServiceException as e:
            rospy.logerr(f"기어 변경 서비스 호출 실패: {e}")

    def get_current_waypoint(self, path):
        min_dist = float('inf')        
        current_waypoint = -1
        lookahead_distance = self.lfd  # 속도에 따른 Look Ahead Distance를 사용
        max_lookahead_distance = 40  # 차량이 절대 넘어가면 안 되는 거리 (필요시 조정)

        for i, pose in enumerate(path.poses):
            dx = self.current_postion.x - pose.pose.position.x
            dy = self.current_postion.y - pose.pose.position.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))

            # Look Ahead Distance 내에서 가장 가까운 경로점 선택
            if lookahead_distance <= dist <= max_lookahead_distance and dist < min_dist:
                min_dist = dist
                current_waypoint = i

        return current_waypoint

    def calc_pure_pursuit(self):
        # 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (self.ego_vehicle_status.velocity.x) * 3.6 * self.lfd_gain
        self.lfd = min(max(self.min_lfd, self.lfd), self.max_lfd)
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
        rospy.loginfo(self.lfd)
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
                k = 0.23
        else:
            rospy.loginfo("Not enough points to calculate curvature, using default values.")
            k = 0.23

        self.is_look_forward_point = False

        # 차량의 앞바퀴 위치를 계산 (차량의 중심으로부터 wheel_base 만큼 앞쪽으로 이동)
        front_axle_x = self.current_postion.x + cos(self.vehicle_yaw) * 0
        front_axle_y = self.current_postion.y + sin(self.vehicle_yaw) * 0
        front_axle_position = [front_axle_x, front_axle_y]

        translation = [front_axle_x, front_axle_y]
        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        closest_point = None  # 가장 가까운 포인트
        closest_dis = float('inf')  # 가장 가까운 거리 초기화
        
        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0:  # 차량 앞쪽에 있는 포인트만 고려
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))

                # LFD 조건을 만족하는 포인트가 있으면 바로 사용
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
                
                # 가장 가까운 포인트를 추적
                if dis < closest_dis:
                    closest_dis = dis
                    closest_point = path_point

        # LFD 조건을 만족하는 포인트가 없을 때, 가장 가까운 포인트 사용
        if not self.is_look_forward_point and closest_point is not None:
            self.forward_point = closest_point
            self.is_look_forward_point = True
            rospy.logwarn("No point found that satisfies LFD. Using closest point as forward point.")

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
        # 곡률 값에 상한선과 하한선 적용
        min_curvature = 0.001  # 최소 곡률 값 (직선에 가까운 경우)
        max_curvature = 0.3    # 최대 곡률 값 (급격한 회전)

        curvature = max(min_curvature, min(curvature, max_curvature))
        return curvature

    # 스탠리 알고리즘에서 사용하는 동적 게인 계산
    def calculate_stanley_gain(self, max_curvature):
        speed = abs(self.ego_vehicle_status.velocity.x * 3.6)  # 속도를 km/h로 변환
        if speed == 0:
            speed_gain = 1.0
        else:
            speed_gain = 1 / (speed + 1e-3)  # 속도가 클수록 스티어링 게인 낮아짐

        curvature_gain = max_curvature  # 경로 상의 곡률에 따른 게인 조정
        k = 0.23 + (speed_gain * curvature_gain)
        
        # 게인의 상한/하한 설정
        min_k = 0.15
        max_k = 0.7
        k = np.clip(k, min_k, max_k)
        rospy.loginfo(f"현재 gain : {k}")
        return k

class pidControl:
    def __init__(self):
        self.p_gain = 0.32
        self.i_gain = 0.5
        self.d_gain = 0.016
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
                v_max = self.car_max_speed * 0.65  
            elif  pi / 6 >= heading_error > pi / 9:  # 각도 차이가 20도 이상일 때
                v_max = self.car_max_speed * 0.75  
            elif pi / 9 >= heading_error > pi / 12:  # 각도 차이가 15도 이상일 때
                v_max = self.car_max_speed * 0.85  
            elif pi / 12>= heading_error > pi / 20:  # 각도 차이가 9도 이상일 때
                v_max = self.car_max_speed * 0.95 
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